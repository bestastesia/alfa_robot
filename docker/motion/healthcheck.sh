#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/robot_contract/setup.bash
source /opt/motion/setup.bash
set -u

exec ros2 run armmotion_demo motion_healthcheck --timeout-s "${MOTION_HEALTHCHECK_TIMEOUT_S:-4.0}"
