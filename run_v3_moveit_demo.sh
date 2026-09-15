#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ALFA_V3_MOVEIT_DOMAIN_ID:-79}"

source "$ROOT/tools/ros_humble_env.sh"
if ! /usr/bin/python3 "$ROOT/tools/check_v3_demo_install.py"; then
  echo "检测到旧版或不完整 install，重新构建 V3.1.1 MoveIt Demo。"
  "$ROOT/build_v3_moveit_demo.sh"
fi
source "$ROOT/ros2_ws/install/setup.bash"
/usr/bin/python3 "$ROOT/tools/check_v3_demo_install.py"

exec ros2 launch alfa_robot_moveit_config demo.launch.py
