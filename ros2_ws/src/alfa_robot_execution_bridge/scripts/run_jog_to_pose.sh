#!/usr/bin/env bash
set -eo pipefail
source /home/ar/lhy_dev/env.sh
exec /usr/bin/python3 /home/ar/lhy_dev/ros2_ws/src/alfa_robot_execution_bridge/scripts/jog_to_pose.py "$@"
