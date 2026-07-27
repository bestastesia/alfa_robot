#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/env.sh"

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN=true
  shift
fi

exec ros2 run armmotion_demo algorithm_thread --ros-args \
  -p dry_run:="${DRY_RUN}" \
  -p source_ws:="${ARMMOTION_SOURCE_WS}" \
  -p output_root:="${ARMMOTION_OUTPUT_ROOT}" \
  -p trajectory_rate_hz:=30.0 \
  -p execution_speed_scale:=3.0 \
  -p max_joint_speed_deg_s:=10.0 \
  -p max_joint_acceleration_deg_s2:=60.0 \
  -p max_updown_speed_m_s:=0.05 \
  -p updown_acceleration_m_s2:=0.05 \
  -p updown_deceleration_m_s2:=0.05 \
  "$@"
