#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ALFA_V3_MOVEIT_DOMAIN_ID:-79}"

source "$ROOT/tools/ros_humble_env.sh"
if [[ ! -f "$ROOT/ros2_ws/install/alfa_robot_moveit_config/share/alfa_robot_moveit_config/launch/demo.launch.py" ]]; then
  "$ROOT/build_v3_moveit_demo.sh"
fi
source "$ROOT/ros2_ws/install/setup.bash"

exec ros2 launch alfa_robot_moveit_config demo.launch.py
