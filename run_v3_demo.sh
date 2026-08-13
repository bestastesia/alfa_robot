#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ALFA_V3_DOMAIN_ID:-78}"

source "$ROOT/tools/ros_humble_env.sh"
if [[ ! -f "$ROOT/ros2_ws/install/alfa_robot_description/share/alfa_robot_description/urdf/alfa_robot.urdf.xacro" ]]; then
  "$ROOT/build_v3_demo.sh"
fi
source "$ROOT/ros2_ws/install/setup.bash"

exec ros2 launch alfa_robot_description view_alfa_robot.launch.py
