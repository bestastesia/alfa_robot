#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
base_image="${CONTRACT_RUNTIME_IMAGE:-robot/contract-runtime:interfaces-92d6ff2-20260827}"
version="${MOTION_VERSION:-0.1.0-rc1}"
image="${MOTION_IMAGE:-alfa-motion:${version}}"
git_sha="${MOTION_GIT_SHA:-$(git -C "${root}" rev-parse HEAD)}"
interfaces_sha="$(awk '$1 == "commit:" {print $2; exit}' "${root}/ros2_ws/src/dependencies.lock.yaml")"
proxy_url="${MOTION_BUILD_PROXY:-}"

build_args=()
if [[ -n "${proxy_url}" ]]; then
  build_args+=(
    --network host
    --build-arg "HTTP_PROXY=${proxy_url}"
    --build-arg "HTTPS_PROXY=${proxy_url}"
    --build-arg "http_proxy=${proxy_url}"
    --build-arg "https_proxy=${proxy_url}"
  )
fi

docker image inspect "${base_image}" >/dev/null

docker build \
  "${build_args[@]}" \
  --file "${root}/docker/motion/Dockerfile" \
  --build-arg "CONTRACT_RUNTIME_IMAGE=${base_image}" \
  --build-arg "MOTION_VERSION=${version}" \
  --build-arg "MOTION_GIT_SHA=${git_sha}" \
  --build-arg "ROBOT_INTERFACES_SHA=${interfaces_sha}" \
  --tag "${image}" \
  "${root}"

printf 'MOTION_IMAGE=%s\n' "${image}"
