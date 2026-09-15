#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/tools/ros_humble_env.sh"
/usr/bin/python3 "$ROOT/tools/sync_v311_description.py" --check-local
cd "$ROOT/ros2_ws"
colcon build \
  --packages-select alfa_robot_description \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select alfa_robot_description --event-handlers console_cohesion+
colcon test-result --test-result-base build --verbose
