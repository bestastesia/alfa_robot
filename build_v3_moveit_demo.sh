#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/tools/ros_humble_env.sh"
cd "$ROOT/ros2_ws"
colcon build \
  --packages-up-to alfa_robot_moveit_config \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
