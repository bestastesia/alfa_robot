#!/usr/bin/env bash
set -eo pipefail
source /home/ar/lhy_dev/env.sh
exec /usr/bin/python3 /home/ar/lhy_dev/scripts/send_dual_grasp_sequence.py "$@"
