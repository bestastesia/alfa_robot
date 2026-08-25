#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/env.sh"

exec /usr/bin/python3 \
  "${ARMMOTION_SOURCE_WS}/src/alfa_robot_execution_bridge/scripts/jog_to_pose.py" \
  "$@" --no-rerun
