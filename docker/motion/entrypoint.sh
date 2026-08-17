#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

source_root="${MOTION_SOURCE_ROOT:-/repo}"
workspace="${MOTION_WORKSPACE:-/workspace}"

if [[ ! -d "${source_root}/ros2_ws/src" ]]; then
  repository_url="${MOTION_REPOSITORY_URL:-}"
  repository_ref="${MOTION_REPOSITORY_REF:-v5_dev}"
  if [[ -z "${repository_url}" ]]; then
    echo "FAIL: ${source_root}/ros2_ws/src 不存在，且未设置 MOTION_REPOSITORY_URL" >&2
    exit 2
  fi
  mkdir -p "$(dirname "${source_root}")"
  git clone --branch "${repository_ref}" --single-branch "${repository_url}" "${source_root}"
fi

if [[ ! -f "${source_root}/ros2_ws/src/robot_interfaces/robot_motion_interfaces/package.xml" ]]; then
  echo "FAIL: 缺少中央 robot_interfaces 源码。请在宿主仓库执行：" >&2
  echo "  cd ${source_root}/ros2_ws && vcs import src < src/dependencies.repos" >&2
  exit 2
fi

robot_interfaces_lock="${source_root}/ros2_ws/src/dependencies.lock.yaml"
expected_robot_interfaces_sha="$(awk '$1 == "commit:" {print $2; exit}' "${robot_interfaces_lock}")"
actual_robot_interfaces_sha="$(git -C "${source_root}/ros2_ws/src/robot_interfaces" rev-parse HEAD)"
if [[ ! "${expected_robot_interfaces_sha}" =~ ^[0-9a-f]{40}$ ]] || \
   [[ "${actual_robot_interfaces_sha}" != "${expected_robot_interfaces_sha}" ]]; then
  echo "FAIL: robot_interfaces 未处于锁定提交。" >&2
  echo "  expected=${expected_robot_interfaces_sha:-missing}" >&2
  echo "  actual=${actual_robot_interfaces_sha:-missing}" >&2
  exit 2
fi

mkdir -p \
  "${workspace}/build" \
  "${workspace}/install" \
  "${workspace}/log" \
  "${HOME:-${workspace}/home}" \
  "${ROS_LOG_DIR:-${workspace}/ros_logs}" \
  "${MOTION_OUTPUT_ROOT:-/motion_data}"

source_paths=(
  "${source_root}/ros2_ws/src"
  "${source_root}/tools/demonstration0720/armmotion/ros2_ws/src"
)

if [[ "${MOTION_SKIP_BUILD:-false}" != "true" ]]; then
  echo "[motion] Release 构建开始: source=${source_root} workspace=${workspace}" >&2
  colcon --log-base "${workspace}/log" build \
    --base-paths "${source_paths[@]}" \
    --build-base "${workspace}/build" \
    --install-base "${workspace}/install" \
    --symlink-install \
    --packages-up-to alfa_robot_moveit_config armmotion_demo \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DPython3_EXECUTABLE=/usr/bin/python3
fi

source "${workspace}/install/setup.bash"
export MOTION_SOURCE_WS="${source_root}/ros2_ws"
export ARMMOTION_SOURCE_WS="${source_root}/ros2_ws"
export ARMMOTION_OUTPUT_ROOT="${MOTION_OUTPUT_ROOT:-/motion_data}"
export ALFA_ROS_SETUP="${workspace}/install/setup.bash"

case "${MOTION_ROLE:-server}" in
  task)
    exec ros2 run armmotion_demo manual_domain_task "$@"
    ;;
  shell)
    exec bash "$@"
    ;;
  server)
    mock_pid=""
    server_pid=""
    cleanup() {
      trap - EXIT INT TERM
      [[ -z "${mock_pid}" ]] || kill -INT "${mock_pid}" 2>/dev/null || true
      [[ -z "${server_pid}" ]] || kill -INT "${server_pid}" 2>/dev/null || true
      for _ in $(seq 1 50); do
        running=false
        for pid in "${mock_pid}" "${server_pid}"; do
          if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            running=true
          fi
        done
        [[ "${running}" == "true" ]] || break
        sleep 0.1
      done
      for pid in "${mock_pid}" "${server_pid}"; do
        [[ -z "${pid}" ]] || kill -KILL "${pid}" 2>/dev/null || true
      done
      wait 2>/dev/null || true
    }
    trap 'cleanup; exit 0' INT TERM
    trap cleanup EXIT
    if [[ "${MOTION_RT_MODE:-external}" == "mock" ]]; then
      ros2 run armmotion_demo mock_current_rt_control --ros-args \
        -p execution_time_scale:="${MOTION_MOCK_TIME_SCALE:-0.0}" &
      mock_pid=$!
    fi
    dry_run="${MOTION_DRY_RUN:-true}"
    ros2 run armmotion_demo domain_motion_server --ros-args \
      -p dry_run:="${dry_run}" \
      -p allow_partial_domain_test:=true \
      -p source_ws:="${source_root}/ros2_ws" \
      -p output_root:="${MOTION_OUTPUT_ROOT:-/motion_data}" &
    server_pid=$!
    wait "${server_pid}"
    ;;
  *)
    echo "FAIL: 未知 MOTION_ROLE=${MOTION_ROLE}" >&2
    exit 2
    ;;
esac
