#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/tools/ros_humble_env.sh"
cd "$ROOT/ros2_ws"
colcon build \
  --packages-select alfa_robot_description \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
