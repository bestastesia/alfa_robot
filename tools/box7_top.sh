#!/usr/bin/env bash
# Standalone simulation: start at box7 with earlier boxes already removed.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source tools/ros_humble_env.sh
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=199
exec ros2 launch alfa_robot_moveit_config \
  v3_box_wall_comfort_grasp_demo.launch.py \
  x:=0.50 box_id:=7 arm:=auto suction_mode:=top \
  wall_context:=sequence_prefix planning_seed:=104731 "$@"
