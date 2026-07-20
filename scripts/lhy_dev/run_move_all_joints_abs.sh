#!/usr/bin/env bash
set -eo pipefail

# 兼容历史入口，但实际实现强制走 alfa_robot_execution_bridge/joints.py 合同。
# 不允许再调用旧 scripts/move_all_joints_abs.py，它维护过重复且错误的方向/升降映射。
source /home/ar/lhy_dev/env.sh
exec /usr/bin/python3 \
  /home/ar/lhy_dev/ros2_ws/src/alfa_robot_execution_bridge/scripts/jog_to_pose.py \
  "$@"
