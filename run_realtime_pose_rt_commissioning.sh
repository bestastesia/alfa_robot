#!/usr/bin/env bash
set -eo pipefail

stage="observe"
arm="left"
batch_rate_hz="30.0"
allow_provisional_limits="false"
enable_rviz="true"
enable_rerun="true"
supervised="false"

usage() {
  printf '%s\n' \
    "Usage: $0 observe|hold|track [--arm left|right] [--rate 30] [--supervised]" \
    "          [--allow-provisional-limits] [--no-rviz] [--no-rerun]" \
    "" \
    "observe: read /joint_states and visualize only; creates no /rt/update publisher" \
    "hold:    switch/open and send only a current-pose hold suffix" \
    "track:   keep current-pose hold until the target marker is started separately"
}

if (($#)) && [[ "$1" != --* ]]; then
  stage="$1"
  shift
fi

while (($#)); do
  case "$1" in
    --arm)
      shift
      arm="${1:-}"
      ;;
    --rate)
      shift
      batch_rate_hz="${1:-}"
      ;;
    --supervised)
      supervised="true"
      ;;
    --allow-provisional-limits)
      allow_provisional_limits="true"
      ;;
    --no-rviz)
      enable_rviz="false"
      ;;
    --no-rerun)
      enable_rerun="false"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "$stage" != "observe" && "$stage" != "hold" && "$stage" != "track" ]]; then
  printf 'stage must be observe, hold, or track.\n' >&2
  exit 2
fi
if [[ "$arm" != "left" && "$arm" != "right" ]]; then
  printf 'arm must be left or right.\n' >&2
  exit 2
fi
if [[ ! "$batch_rate_hz" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'rate must be numeric.\n' >&2
  exit 2
fi

if [[ "$stage" != "observe" ]]; then
  if [[ "$supervised" != "true" ]]; then
    printf '%s\n' 'hold/track requires --supervised and both Motion + ELECTRI operators present.' >&2
    exit 3
  fi
  if [[ "${ALFA_RT_COMMISSIONING_CONFIRM:-}" != "CURRENT_POSE_HOLD_ACKED" ]]; then
    printf '%s\n' \
      'Refusing command mode. Export:' \
      '  ALFA_RT_COMMISSIONING_CONFIRM=CURRENT_POSE_HOLD_ACKED' >&2
    exit 3
  fi
  export ALFA_REAL_ROLLING_CONFIRM=ELECTRI_102_SUPERVISED_ROLLING
fi

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
workspace_root="$(cd "$(dirname "$script_path")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ -z "${ALFA_RT_UNDERLAY:-}" ]]; then
  ALFA_RT_UNDERLAY="$(find /home/ar -maxdepth 3 -type f \
    -path '/home/ar/motion2_test_rolling_native_ws_*_clean/install/setup.bash' \
    -printf '%p\n' 2>/dev/null | sort -V | tail -n 1)"
fi
if [[ -z "$ALFA_RT_UNDERLAY" || ! -f "$ALFA_RT_UNDERLAY" ]]; then
  printf 'RT underlay not found; set ALFA_RT_UNDERLAY.\n' >&2
  exit 1
fi
source "$ALFA_RT_UNDERLAY"

# The RT test overlay also contains a historical Motion underlay path.  Source
# the selected current Motion tree afterwards so its robot model/config wins;
# the demo overlay below still owns the frozen rolling interface packages.
if [[ -z "${ALFA_MOTION_UNDERLAY:-}" ]]; then
  ALFA_MOTION_UNDERLAY=/home/ar/motion_domain_current/ros2_ws/install/setup.bash
fi
if [[ ! -f "$ALFA_MOTION_UNDERLAY" ]]; then
  printf 'Motion underlay not found: %s\n' "$ALFA_MOTION_UNDERLAY" >&2
  exit 1
fi
source "$ALFA_MOTION_UNDERLAY"

workspace_install="$workspace_root/ros2_ws/install"
if [[ ! -f "$workspace_install/local_setup.bash" ]]; then
  printf 'Demo install missing: %s\n' "$workspace_install/local_setup.bash" >&2
  exit 1
fi
COLCON_CURRENT_PREFIX="$workspace_install"
source "$workspace_install/local_setup.bash"
unset COLCON_CURRENT_PREFIX

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
printf 'ELECTRI-102 commissioning stage=%s arm=%s domain=%s\n' \
  "$stage" "$arm" "$ROS_DOMAIN_ID"
if [[ "$stage" == "observe" ]]; then
  printf '%s\n' 'READ ONLY: current 14-axis pose -> Motion IK initialization; no RT writer.'
else
  printf '%s\n' \
    'REAL COMMAND MODE: first 25 feedback samples must be stable.' \
    'The Open hold pose must match the captured pose within 0.25deg / 1mm.' \
    'The first accepted suffix is current-pose HOLD; target marker is not started.'
fi

exec ros2 launch alfa_robot_benchmarks realtime_6d_pose_rt_commissioning.launch.py \
  arm:="$arm" \
  commissioning_stage:="$stage" \
  batch_rate_hz:="$batch_rate_hz" \
  allow_provisional_limits:="$allow_provisional_limits" \
  enable_target_marker:=false \
  enable_rviz:="$enable_rviz" \
  enable_rerun:="$enable_rerun"
