#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/robot_contract/setup.bash
source /opt/motion/setup.bash
set -u

if [[ $# -eq 0 ]]; then
  set -- server
fi

case "$1" in
  server)
    shift
    exec ros2 launch armmotion_demo motion_domain.launch.py \
      use_mock_rt_control:="${MOTION_USE_MOCK_RT_CONTROL:-false}" \
      dry_run:="${MOTION_DRY_RUN:-false}" \
      trajectory_action:="${MOTION_TRAJECTORY_ACTION:-/whole_body_jtc/follow_joint_trajectory}" \
      output_root:="${ARMMOTION_OUTPUT_ROOT:-/var/lib/robot/motion}" \
      max_updown_speed_m_s:="${ARMMOTION_MAX_UPDOWN_SPEED_M_S:-0.15}" \
      "$@"
    ;;
  shell)
    shift
    exec bash "$@"
    ;;
  release-info)
    exec cat /opt/motion-release/manifest.yaml
    ;;
  robot-runtime-doctor|contract-runtime-doctor)
    exec "$@"
    ;;
  *)
    exec "$@"
    ;;
esac
