#!/usr/bin/env bash
set -euo pipefail

# 保留历史命令名，但统一转发到当前 14 轴 rt-control 工具。
# 旧 move_all_joints_abs.py 维护过重复的方向和升降映射，禁止再调用。
CURRENT_MOTION_ENTRY="${MOTION_DOMAIN_ROOT:-/home/ar/motion_domain_current}/run_jog_to_pose.sh"
if [[ -x "${CURRENT_MOTION_ENTRY}" ]]; then
  exec "${CURRENT_MOTION_ENTRY}" "$@" --no-rerun
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${REPO_ROOT}/ros2_ws/install/setup.bash"
fi
set -u

exec /usr/bin/python3 \
  "${REPO_ROOT}/ros2_ws/src/alfa_robot_execution_bridge/scripts/jog_to_pose.py" \
  "$@" --no-rerun
