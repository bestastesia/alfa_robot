#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image="${MOTION_IMAGE:?MOTION_IMAGE must identify the motion image}"
version="${MOTION_VERSION:-2.0.1}"
output_dir="${MOTION_RELEASE_DIR:-/tmp/alfa-motion-release/motion-domain-${version}}"
archive="${output_dir}/motion-domain-${version}.tar.zst"

mkdir -p "${output_dir}"
docker save "${image}" | zstd -T0 -19 -o "${archive}"
cp "${root}/docker/motion/compose.yaml" "${output_dir}/compose.yaml"
cp "${root}/docker/motion/DEPLOYMENT.md" "${output_dir}/DEPLOYMENT.md"
docker run --rm "${image}" release-info > "${output_dir}/motion-release-manifest.yaml"
docker image inspect --format '{{.Id}}' "${image}" > "${output_dir}/image-id.txt"
if (( $(stat -c '%s' "${archive}") > 1900 * 1024 * 1024 )); then
  split -b 1800M -d -a 2 "${archive}" "${archive}.part-"
  rm "${archive}"
fi
(
  cd "${output_dir}"
  find . -maxdepth 1 -type f ! -name SHA256SUMS -printf '%f\0' \
    | sort -z \
    | xargs -0 sha256sum > SHA256SUMS
)
printf '%s\n' "${output_dir}"
