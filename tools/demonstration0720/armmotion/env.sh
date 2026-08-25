#!/usr/bin/env bash
set -euo pipefail

ARMMOTION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ARMMOTION_ROOT
ARMMOTION_OVERLAY_WS="${ARMMOTION_ROOT}/ros2_ws"
export ARMMOTION_SOURCE_WS="${ARMMOTION_OVERLAY_WS}"
REPO_SOURCE_WS="$(cd "${ARMMOTION_ROOT}/../../.." && pwd)/ros2_ws"
if [[ ! -f "${ARMMOTION_SOURCE_WS}/src/alfa_robot_moveit_config/scripts/extract_sequence_rerun.py" \
      && -f "${REPO_SOURCE_WS}/src/alfa_robot_moveit_config/scripts/extract_sequence_rerun.py" ]]; then
  export ARMMOTION_SOURCE_WS="${REPO_SOURCE_WS}"
fi
export ARMMOTION_OUTPUT_ROOT="${ARMMOTION_ROOT}/data"
export ROS_LOG_DIR="${ARMMOTION_ROOT}/logs"

detect_rt_control_domain_id() {
  local pid domain
  while read -r pid; do
    [[ -n "${pid}" && -r "/proc/${pid}/environ" ]] || continue
    domain="$(tr '\0' '\n' < "/proc/${pid}/environ" | sed -n 's/^ROS_DOMAIN_ID=//p' | head -n 1)"
    if [[ "${domain}" =~ ^[0-9]+$ ]] && (( domain <= 232 )); then
      printf '%s\n' "${domain}"
      return 0
    fi
  done < <(pgrep -u "$(id -u)" -f '/controller_manager/ros2_control_node' 2>/dev/null || true)
  return 1
}

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  detected_rt_domain="$(detect_rt_control_domain_id || true)"
  if [[ -n "${detected_rt_domain}" ]]; then
    export ROS_DOMAIN_ID="${detected_rt_domain}"
    echo "检测到活动 rt-control：ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >&2
  fi
fi
ARMMOTION_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ARMMOTION_TRAJECTORY_ACTION="${ARMMOTION_TRAJECTORY_ACTION:-/whole_body_jtc/follow_joint_trajectory}"
export ARMMOTION_TRAJECTORY_ACTION
mkdir -p "${ARMMOTION_OUTPUT_ROOT}" "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
if [[ "${ARMMOTION_SOURCE_WS}" != "${ARMMOTION_OVERLAY_WS}" \
      && -f "${ARMMOTION_SOURCE_WS}/install/setup.bash" ]]; then
  source "${ARMMOTION_SOURCE_WS}/install/setup.bash"
fi
if [[ -f "${ARMMOTION_OVERLAY_WS}/install/setup.bash" ]]; then
  source "${ARMMOTION_OVERLAY_WS}/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ARMMOTION_ROS_DOMAIN_ID}"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export FASTRTPS_DEFAULT_PROFILES_FILE="${ARMMOTION_ROOT}/config/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE}"
