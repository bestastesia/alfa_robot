#!/usr/bin/env bash
set -eo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(cd "$(dirname "${script_path}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"

source /opt/ros/humble/setup.bash
if [[ ! -f "${workspace}/install/setup.bash" ]]; then
  echo "未找到 ${workspace}/install/setup.bash，请先编译工作空间。" >&2
  exit 1
fi
source "${workspace}/install/setup.bash"
set -u

if [[ -z "${DISPLAY:-}" ]]; then
  display_socket="$(find /tmp/.X11-unix -maxdepth 1 -type s -name 'X*' -printf '%f\n' 2>/dev/null | sort -V | head -n 1)"
  if [[ -n "${display_socket}" ]]; then
    export DISPLAY=":${display_socket#X}"
  fi
fi
if [[ -z "${XAUTHORITY:-}" ]]; then
  if [[ -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
    export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
  elif [[ -f "${HOME}/.Xauthority" ]]; then
    export XAUTHORITY="${HOME}/.Xauthority"
  fi
fi

exec ros2 run alfa_robot_execution_bridge joint_teach_pendant "$@"
