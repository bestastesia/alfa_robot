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
mkdir -p "${ARMMOTION_OUTPUT_ROOT}" "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/ar/lhy_dev/env.sh ]]; then
  source /home/ar/lhy_dev/env.sh
fi
if [[ "${ARMMOTION_SOURCE_WS}" != "${ARMMOTION_OVERLAY_WS}" \
      && -f "${ARMMOTION_SOURCE_WS}/install/setup.bash" ]]; then
  source "${ARMMOTION_SOURCE_WS}/install/setup.bash"
fi
if [[ -f "${ARMMOTION_OVERLAY_WS}/install/setup.bash" ]]; then
  source "${ARMMOTION_OVERLAY_WS}/install/setup.bash"
fi
set -u
