#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/tools/ros_humble_env.sh"
/usr/bin/python3 "$ROOT/tools/sync_v311_description.py" --check-local
cd "$ROOT/ros2_ws"
colcon build \
  --packages-up-to alfa_robot_moveit_config \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
/usr/bin/python3 src/alfa_robot_moveit_config/test/test_v311_demo_contract.py
colcon test --packages-select alfa_robot_description alfa_robot_moveit_config \
  --event-handlers console_cohesion+
colcon test-result --test-result-base build --verbose
