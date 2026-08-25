#!/usr/bin/env bash
set -Eeo pipefail

arm="left"
batch_rate_hz="30"

usage() {
  printf '%s\n' \
    "Usage: ALFA_RT_COMMISSIONING_CONFIRM=CURRENT_POSE_HOLD_ACKED $0 [--arm left|right] [--rate 30]" \
    "" \
    "Starts the supervised real-time 6D pose controller, RViz target and Rerun." \
    "The draggable target is exposed only after the current-pose HOLD is ACKed."
}

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

if [[ "$arm" != "left" && "$arm" != "right" ]]; then
  printf 'arm must be left or right.\n' >&2
  exit 2
fi
if [[ ! "$batch_rate_hz" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'rate must be numeric.\n' >&2
  exit 2
fi
if [[ "${ALFA_RT_COMMISSIONING_CONFIRM:-}" != "CURRENT_POSE_HOLD_ACKED" ]]; then
  printf '%s\n' \
    'Real control requires the explicit supervised confirmation:' \
    '  ALFA_RT_COMMISSIONING_CONFIRM=CURRENT_POSE_HOLD_ACKED ./run_realtime_pose_control_with_rerun.sh' >&2
  exit 3
fi

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
workspace_root="$(cd "$(dirname "$script_path")" && pwd)"

source /opt/ros/humble/setup.bash
rt_underlay="${ALFA_RT_UNDERLAY:-$(find /home/ar -maxdepth 3 -type f \
  -path '/home/ar/motion2_test_rolling_native_ws_*_clean/install/setup.bash' \
  -printf '%p\n' 2>/dev/null | sort -V | tail -n 1)}"
if [[ -z "$rt_underlay" || ! -f "$rt_underlay" ]]; then
  printf 'RT underlay not found; set ALFA_RT_UNDERLAY.\n' >&2
  exit 1
fi
source "$rt_underlay"

motion_underlay="${ALFA_MOTION_UNDERLAY:-/home/ar/motion_domain_current/ros2_ws/install/setup.bash}"
if [[ ! -f "$motion_underlay" ]]; then
  printf 'Motion underlay not found: %s\n' "$motion_underlay" >&2
  exit 1
fi
source "$motion_underlay"

workspace_install="$workspace_root/ros2_ws/install"
if [[ ! -f "$workspace_install/local_setup.bash" ]]; then
  printf 'Demo install missing: %s\n' "$workspace_install/local_setup.bash" >&2
  exit 1
fi
COLCON_CURRENT_PREFIX="$workspace_install"
source "$workspace_install/local_setup.bash"
unset COLCON_CURRENT_PREFIX

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

if pgrep -u "$(id -u)" -f '/rolling_pose_client.py' >/dev/null \
  || pgrep -u "$(id -u)" -f 'realtime_6d_pose_rt_commissioning.launch.py' >/dev/null; then
  printf '%s\n' \
    'Refusing to start: an older real-time pose Demo is still running.' \
    'Press Ctrl+C once in its main terminal, wait for FJT_READY, then retry.' >&2
  exit 4
fi

controllers_raw="$(timeout 8 ros2 control list_controllers 2>&1 || true)"
controllers="$(sed -E $'s/\x1B\\[[0-9;]*[mK]//g' <<<"$controllers_raw")"
if ! grep -Eq '^whole_body_jtc[[:space:]].*[[:space:]]active[[:space:]]*$' <<<"$controllers" \
  || ! grep -Eq '^rolling_trajectory_controller[[:space:]].*[[:space:]]inactive[[:space:]]*$' <<<"$controllers"; then
  printf '%s\n' \
    'Refusing to start: rt-control is not in the required FJT_READY state.' \
    'Expected whole_body_jtc=active and rolling_trajectory_controller=inactive.' >&2
  printf '%s\n' "$controllers" >&2
  exit 5
fi

state_info="$(timeout 8 ros2 topic info /rt/rolling_joint_control/state --no-daemon 2>&1 || true)"
if ! grep -q 'Publisher count: 1' <<<"$state_info"; then
  printf '%s\n' \
    'Refusing to start: /rt/rolling_joint_control/state has no publisher on Domain 0.' >&2
  exit 6
fi

mkdir -p "$workspace_root/evidence"
export ALFA_RT_EVIDENCE_LOG="$workspace_root/evidence/track_$(date +%Y%m%d_%H%M%S).jsonl"

launch_pid=""
launch_pgid=""
marker_pid=""
marker_pgid=""
cleanup_started="false"

process_group_alive() {
  local process_group_id="$1"
  kill -0 -- "-$process_group_id" 2>/dev/null
}

wait_for_group_exit() {
  local process_group_id="$1"
  local attempt
  for attempt in {1..150}; do
    if ! process_group_alive "$process_group_id"; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

controller_is_fjt_ready() {
  local state_raw state
  state_raw="$(timeout 2 ros2 control list_controllers 2>&1 || true)"
  state="$(sed -E $'s/\x1B\\[[0-9;]*[mK]//g' <<<"$state_raw")"
  grep -Eq '^whole_body_jtc[[:space:]].*[[:space:]]active[[:space:]]*$' <<<"$state" \
    && grep -Eq '^rolling_trajectory_controller[[:space:]].*[[:space:]]inactive[[:space:]]*$' <<<"$state"
}

cleanup() {
  if [[ "$cleanup_started" == "true" ]]; then
    return
  fi
  cleanup_started="true"
  trap - INT TERM EXIT
  if [[ -n "$marker_pgid" ]] && process_group_alive "$marker_pgid"; then
    kill -INT -- "-$marker_pgid" 2>/dev/null || true
    if wait_for_group_exit "$marker_pgid"; then
      wait "$marker_pid" 2>/dev/null || true
    else
      kill -TERM -- "-$marker_pgid" 2>/dev/null || true
      printf 'Marker process group needed SIGTERM (pgid=%s).\n' "$marker_pgid" >&2
    fi
  fi
  if [[ -n "$launch_pgid" ]] && process_group_alive "$launch_pgid"; then
    printf '%s\n' 'Closing rolling session safely; wait for FJT_READY...'
    # Match Ctrl+C in a foreground terminal: every launch child, especially
    # rolling_pose_client.py, must receive SIGINT and run REQUEST_STOP ->
    # FINALIZE -> FJT_READY.  Signalling only ros2 launch leaves orphan writers.
    kill -INT -- "-$launch_pgid" 2>/dev/null || true
    if wait_for_group_exit "$launch_pgid"; then
      wait "$launch_pid" 2>/dev/null || true
    else
      kill -TERM -- "-$launch_pgid" 2>/dev/null || true
      printf '%s\n' \
        'Commissioning process group did not exit within 15s and needed SIGTERM.' >&2
    fi
  fi
  if [[ -n "$launch_pgid" ]]; then
    if controller_is_fjt_ready; then
      printf '%s\n' 'FJT_READY confirmed; the next supervised run may start.'
    else
      printf '%s\n' \
        'FJT_READY was not confirmed. Do not restart real-time control; inspect rt-control first.' >&2
    fi
  fi
}

trap 'cleanup; exit 130' INT TERM
trap cleanup EXIT

commission_args=(
  track
  --supervised
  --allow-provisional-limits
  --arm "$arm"
  --rate "$batch_rate_hz"
)
# ELECTRI's expanded commissioning wrapper accepts this argument in every
# stage.  Passing a non-empty value also avoids an empty ROS launch argument.
if "$workspace_root/run_realtime_pose_rt_commissioning.sh" --help 2>&1 | grep -q -- '--axis'; then
  commission_args+=(--axis right_joint4)
fi

printf 'Starting current-pose HOLD, RViz and Rerun on Domain 0...\n'
setsid "$workspace_root/run_realtime_pose_rt_commissioning.sh" "${commission_args[@]}" &
launch_pid=$!
launch_pgid=$launch_pid

ready_gate="$workspace_root/scripts/ik_benchmark/prototypes/realtime_6d_pose/wait_for_rt_track_ready.py"
if ! /usr/bin/python3 "$ready_gate" \
  --soak-seconds 3.0 --min-accepted 60 --timeout-seconds 20.0; then
  printf 'The rolling HOLD did not remain healthy; the Marker was not exposed.\n' >&2
  printf 'Evidence: %s\n' "$ALFA_RT_EVIDENCE_LOG" >&2
  exit 8
fi
if ! kill -0 "$launch_pid" 2>/dev/null; then
  wait "$launch_pid" || true
  printf 'Commissioning launch exited after the HOLD readiness check.\n' >&2
  exit 9
fi

export ALFA_RT_TRACKING_CONFIRM=HOLD_SEQUENCE_ACCEPTED
printf '%s\n' \
  'HOLD ACK received. Starting the draggable 6D target.' \
  'RViz: drag the orange marker. Rerun: observe actual/command/reference motion.' \
  'Press Ctrl+C once here to stop and return safely to FJT_READY.'
setsid "$workspace_root/start_realtime_pose_target_marker.sh" &
marker_pid=$!
marker_pgid=$marker_pid

wait "$launch_pid"
