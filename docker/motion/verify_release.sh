#!/usr/bin/env bash
set -euo pipefail

image="${MOTION_IMAGE:?MOTION_IMAGE must identify the motion image}"

docker run --rm "${image}" robot-runtime-doctor
docker run --rm "${image}" contract-runtime-doctor
docker run --rm "${image}" release-info

user="$(docker image inspect --format '{{.Config.User}}' "${image}")"
if [[ "${user}" != "1000:1000" ]]; then
  echo "FAIL: runtime user=${user}, expected=1000:1000" >&2
  exit 1
fi

if docker run --rm "${image}" bash -lc \
  'test -e /repo || test -e /motion_build || test -e /motion_ws/src'; then
  echo "FAIL: source or build workspace leaked into runtime image" >&2
  exit 1
fi

MOTION_IMAGE="${image}" docker compose \
  -f "$(dirname "${BASH_SOURCE[0]}")/compose.yaml" config >/dev/null

echo "PASS: Motion release image satisfies static runtime checks"
