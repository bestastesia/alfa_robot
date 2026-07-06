#!/usr/bin/env bash
set -euo pipefail

REPO=/mnt/mydisk/ALFA/alfa_robot
RRD=${1:-$REPO/data/curobo_web_demo/alfa_dual_arm_motion_web.rrd}
PORT=${PORT:-19090}

source /mnt/mydisk/anaconda3/etc/profile.d/conda.sh
conda activate curobo_py310

if [[ ! -f "$RRD" ]]; then
  echo "RRD not found, generating: $RRD"
  cd "$REPO"
  python scripts/curobo_web_demo/generate_alfa_dual_arm_web_rrd.py --save "$RRD"
fi

echo "Starting Rerun Web Viewer..."
echo "Open: http://127.0.0.1:${PORT}"
echo "Recording: $RRD"
exec rerun "$RRD" --web-viewer --web-viewer-port "$PORT"
