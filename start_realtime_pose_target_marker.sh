#!/usr/bin/env bash
set -eo pipefail

if [[ "${ALFA_RT_TRACKING_CONFIRM:-}" != "HOLD_SEQUENCE_ACCEPTED" ]]; then
  printf '%s\n' \
    'Refusing to expose the draggable target.' \
    'After rt_status shows RUNNING and last_accepted_sequence>=1, export:' \
    '  ALFA_RT_TRACKING_CONFIRM=HOLD_SEQUENCE_ACCEPTED' >&2
  exit 3
fi

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
workspace_root="$(cd "$(dirname "$script_path")" && pwd)"
source /opt/ros/humble/setup.bash
rt_underlay="${ALFA_RT_UNDERLAY:-$(find /home/ar -maxdepth 3 -type f \
  -path '/home/ar/motion2_test_rolling_native_ws_*_clean/install/setup.bash' \
  -printf '%p\n' 2>/dev/null | sort -V | tail -n 1)}"
if [[ -n "$rt_underlay" && -f "$rt_underlay" ]]; then
  source "$rt_underlay"
fi
if [[ -f /home/ar/motion_domain_current/ros2_ws/install/setup.bash ]]; then
  source /home/ar/motion_domain_current/ros2_ws/install/setup.bash
fi
COLCON_CURRENT_PREFIX="$workspace_root/ros2_ws/install"
source "$workspace_root/ros2_ws/install/local_setup.bash"
unset COLCON_CURRENT_PREFIX
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec ros2 run alfa_robot_benchmarks interactive_6d_target.py
