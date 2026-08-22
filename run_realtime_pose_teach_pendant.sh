#!/usr/bin/env bash
set -eo pipefail

# PROTOTYPE one-command launcher for the Motion TX boundary preview.
arm="left"
batch_rate_hz="30.0"
enable_rviz="true"
enable_rerun="true"

usage() {
  printf '%s\n' \
    "Usage: $0 [--arm left|right] [--rate 10|30] [--no-rviz] [--no-rerun]" \
    "" \
    "30 Hz Motion TX preview:" \
    "  ROS_DOMAIN_ID=151 $0 --arm left --rate 30" \
    "" \
    "This launcher creates no /rt endpoint and does not simulate RT-Control."
}

while (($#)); do
  case "$1" in
    --arm)
      shift
      arm="${1:-}"
      ;;
    --rate)
      shift
      batch_rate_hz="${1:-}"
      ;;
    --mock)
      printf '%s\n' \
        '提示：--mock 已不再启动 rt-control Mock；当前只显示 Motion TX 消息。' >&2
      ;;
    --real|--observe-only|--allow-real-command|--allow-provisional-limits)
      printf '%s\n' \
        "当前 Demo 只到 Motion TX 边界，不再支持 $1。" \
        '如需真实联调，请使用单独的受监督 rolling 客户端。' >&2
      exit 2
      ;;
    --no-rviz)
      enable_rviz="false"
      ;;
    --no-rerun)
      enable_rerun="false"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '未知参数: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "${arm}" != "left" && "${arm}" != "right" ]]; then
  printf 'arm 必须是 left 或 right。\n' >&2
  exit 2
fi

if [[ ! "${batch_rate_hz}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
   ! awk -v rate="${batch_rate_hz}" 'BEGIN { exit !(rate >= 1.0 && rate <= 100.0) }'; then
  printf 'rate 必须是 1~100Hz 的数字，建议 10 或 30。\n' >&2
  exit 2
fi

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
workspace_root="$(cd "$(dirname "${script_path}")" && pwd)"
if [[ -z "${ALFA_MOTION_UNDERLAY:-}" ]]; then
  sibling_underlay="$(dirname "${workspace_root}")/motion_domain_current/ros2_ws/install/setup.bash"
  if [[ -f "${sibling_underlay}" ]]; then
    ALFA_MOTION_UNDERLAY="${sibling_underlay}"
  fi
fi

source /opt/ros/humble/setup.bash
if [[ -n "${ALFA_MOTION_UNDERLAY:-}" ]]; then
  if [[ ! -f "${ALFA_MOTION_UNDERLAY}" ]]; then
    printf 'ALFA_MOTION_UNDERLAY 不存在: %s\n' "${ALFA_MOTION_UNDERLAY}" >&2
    exit 1
  fi
  source "${ALFA_MOTION_UNDERLAY}"
fi
workspace_install="${workspace_root}/ros2_ws/install"
if [[ ! -f "${workspace_install}/local_setup.bash" ]]; then
  printf '未找到 %s，请先按 README 编译。\n' \
    "${workspace_install}/local_setup.bash" >&2
  exit 1
fi
COLCON_CURRENT_PREFIX="${workspace_install}"
source "${workspace_install}/local_setup.bash"
unset COLCON_CURRENT_PREFIX
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

printf 'PROTOTYPE Motion-TX-only arm=%s rate=%sHz ROS_DOMAIN_ID=%s\n' \
  "${arm}" "${batch_rate_hz}" "${ROS_DOMAIN_ID:-0}"
printf '%s\n' \
  'No RT-Control, no controller simulation, no /rt command publication.' \
  'Exact preview topic: /realtime_6d_pose/motion_tx_preview'

exec ros2 launch alfa_robot_benchmarks realtime_6d_pose_prototype.launch.py \
  arm:="${arm}" \
  batch_rate_hz:="${batch_rate_hz}" \
  enable_rviz:="${enable_rviz}" \
  enable_rerun:="${enable_rerun}"
