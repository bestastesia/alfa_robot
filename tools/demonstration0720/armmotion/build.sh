#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
REPO_SOURCE_WS="$(cd "${ROOT}/../../.." && pwd)/ros2_ws"
ROBOT_INTERFACES_ROOT="${REPO_SOURCE_WS}/src/robot_interfaces"
ROBOT_INTERFACES_LOCK="${REPO_SOURCE_WS}/src/dependencies.lock.yaml"
if [[ -f "${REPO_SOURCE_WS}/install/setup.bash" ]]; then
  source "${REPO_SOURCE_WS}/install/setup.bash"
fi
if ! ros2 pkg prefix robot_motion_interfaces >/dev/null 2>&1; then
  echo "缺少中央 robot_motion_interfaces。先在 ${REPO_SOURCE_WS} 执行：" >&2
  echo "  vcs import src < src/dependencies.repos" >&2
  echo "  colcon build --packages-select robot_interfaces_qos robot_system_interfaces robot_motion_interfaces" >&2
  exit 2
fi
EXPECTED_ROBOT_INTERFACES_SHA="$(awk '$1 == "commit:" {print $2; exit}' "${ROBOT_INTERFACES_LOCK}")"
ACTUAL_ROBOT_INTERFACES_SHA="$(git -C "${ROBOT_INTERFACES_ROOT}" rev-parse HEAD)"
if [[ ! "${EXPECTED_ROBOT_INTERFACES_SHA}" =~ ^[0-9a-f]{40}$ ]] || \
   [[ "${ACTUAL_ROBOT_INTERFACES_SHA}" != "${EXPECTED_ROBOT_INTERFACES_SHA}" ]]; then
  echo "robot_interfaces SHA 不匹配：" >&2
  echo "  expected=${EXPECTED_ROBOT_INTERFACES_SHA:-missing}" >&2
  echo "  actual=${ACTUAL_ROBOT_INTERFACES_SHA:-missing}" >&2
  exit 2
fi
set -u
cd "${ROOT}/ros2_ws"
ALL_PACKAGES=(
  robot_motion_core
  motion_internal_interfaces
  robot_motion_scene_service
  robot_motion_runtime
  alfa_robot_analytic_ik
  alfa_robot_description
  alfa_robot_execution_bridge
  alfa_robot_rerun
  alfa_robot_moveit_config
  armmotion_demo
)
PACKAGES=()
for package in "${ALL_PACKAGES[@]}"; do
  if [[ -f "src/${package}/package.xml" ]]; then
    PACKAGES+=("${package}")
  fi
done
colcon build \
  --symlink-install \
  --packages-select "${PACKAGES[@]}" \
  --allow-overriding "${PACKAGES[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
