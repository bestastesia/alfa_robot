#!/usr/bin/env bash
set -eo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(cd "$(dirname "${script_path}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"

ros_domain_id="${ROS_DOMAIN_ID:-}"
launcher_args=()

validate_ros_domain_id() {
  local value="$1"
  if [[ ! "${value}" =~ ^[0-9]+$ ]] || (( value > 232 )); then
    echo "ROS_DOMAIN_ID must be an integer in the Fast DDS-safe range 0..232: ${value}" >&2
    exit 2
  fi
}

detect_rt_control_domain_id() {
  local pid domain
  while read -r pid; do
    [[ -n "${pid}" && -r "/proc/${pid}/environ" ]] || continue
    domain="$(tr '\0' '\n' < "/proc/${pid}/environ" | sed -n 's/^ROS_DOMAIN_ID=//p' | head -n 1)"
    if [[ "${domain}" =~ ^[0-9]+$ ]] && (( domain <= 232 )); then
      printf '%s\n' "${domain}"
      return 0
    fi
  done < <(pgrep -u "$(id -u)" -f '/controller_manager/ros2_control_node' 2>/dev/null || true)
  return 1
}

while (($#)); do
  case "$1" in
    --ros-domain-id)
      shift
      if [[ $# -eq 0 ]]; then
        echo "--ros-domain-id requires an integer value" >&2
        exit 2
      fi
      ros_domain_id="$1"
      ;;
    --ros-domain-id=*)
      ros_domain_id="${1#*=}"
      ;;
    *)
      launcher_args+=("$1")
      ;;
  esac
  shift
done

if [[ -z "${ros_domain_id}" ]]; then
  ros_domain_id="$(detect_rt_control_domain_id || true)"
  if [[ -n "${ros_domain_id}" ]]; then
    echo "检测到活动 rt-control：ROS_DOMAIN_ID=${ros_domain_id}" >&2
  else
    ros_domain_id=0
  fi
fi
validate_ros_domain_id "${ros_domain_id}"
export ROS_DOMAIN_ID="${ros_domain_id}"

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

exec ros2 run alfa_robot_execution_bridge joint_teach_pendant "${launcher_args[@]}"
