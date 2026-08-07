#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/env.sh"

mode="${1:---mock}"
if [[ "${mode}" == "--mock" ]]; then
  use_mock=true
elif [[ "${mode}" == "--hardware" ]]; then
  use_mock=false
else
  echo "用法：$0 --mock|--hardware" >&2
  exit 2
fi

if [[ "${mode}" == "--hardware" && "${MOTION_HARDWARE_CONFIRM:-}" != "ENABLE_MOTION_HARDWARE" ]]; then
  echo "拒绝实机启动：请先确认 rt-control READY，并设置 MOTION_HARDWARE_CONFIRM=ENABLE_MOTION_HARDWARE" >&2
  exit 3
fi

exec ros2 launch armmotion_demo motion_domain.launch.py \
  use_mock_rt_control:="${use_mock}" \
  dry_run:=false \
  source_ws:="${ARMMOTION_SOURCE_WS}" \
  output_root:="${ARMMOTION_OUTPUT_ROOT}/domain" \
  allow_partial_domain_test:=true
