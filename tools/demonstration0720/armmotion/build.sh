#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/ar/lhy_dev/env.sh ]]; then
  source /home/ar/lhy_dev/env.sh
fi
set -u
cd "${ROOT}/ros2_ws"
ALL_PACKAGES=(
  robot_motion_core
  robot_motion_interfaces
  robot_motion_scene_service
  alfa_robot_analytic_ik
  alfa_robot_description
  alfa_robot_execution_bridge
  alfa_robot_rerun
  alfa_robot_moveit_config
  armmotion_demo
)
PACKAGES=()
for package in "${ALL_PACKAGES[@]}"; do
  if [[ -f "src/${package}/package.xml" ]]; then
    PACKAGES+=("${package}")
  fi
done
colcon build \
  --symlink-install \
  --packages-select "${PACKAGES[@]}" \
  --allow-overriding "${PACKAGES[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
