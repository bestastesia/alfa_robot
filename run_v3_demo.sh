#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ALFA_V3_DOMAIN_ID:-78}"

source "$ROOT/tools/ros_humble_env.sh"
if ! /usr/bin/python3 "$ROOT/tools/check_v3_demo_install.py" --description-only; then
  echo "检测到旧版或不完整 install，重新构建 V3.1.1 Description Demo。"
  "$ROOT/build_v3_demo.sh"
fi
source "$ROOT/ros2_ws/install/setup.bash"
/usr/bin/python3 "$ROOT/tools/check_v3_demo_install.py" --description-only

exec ros2 launch alfa_robot_description view_alfa_robot.launch.py
