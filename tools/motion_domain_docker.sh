#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose_dir="${repo_root}/docker/motion"
compose=(docker compose --project-directory "${compose_dir}" -f "${compose_dir}/compose.yaml")

export MOTION_UID="${MOTION_UID:-$(id -u)}"
export MOTION_GID="${MOTION_GID:-$(id -g)}"

prepare_directories() {
  mkdir -p "${compose_dir}/.workspace" "${repo_root}/data/docker_motion"
}

usage() {
  cat <<'EOF'
用法：
  tools/motion_domain_docker.sh build
  tools/motion_domain_docker.sh start-mock
  tools/motion_domain_docker.sh start-dry
  MOTION_HARDWARE_CONFIRM=ENABLE_MOTION_HARDWARE tools/motion_domain_docker.sh start-external
  tools/motion_domain_docker.sh task --left ... --right ... --yes-execute [其他参数]
  tools/motion_domain_docker.sh status|logs|stop|shell

Mock 默认 ROS_DOMAIN_ID=142；现行 rt-control 联调固定 ROS_DOMAIN_ID=42。
该脚本不启动 rt-control、不调用 /rt/enable、不执行 fault reset。
EOF
}

command="${1:-}"
if [[ -z "${command}" ]]; then
  usage
  exit 2
fi
shift

case "${command}" in
  build)
    prepare_directories
    "${compose[@]}" build motion
    ;;
  start-mock)
    prepare_directories
    export MOTION_ROS_DOMAIN_ID="${MOTION_ROS_DOMAIN_ID:-142}"
    export MOTION_RT_MODE=mock
    export MOTION_DRY_RUN=false
    "${compose[@]}" up --build -d motion
    echo "Motion Mock 已启动：ROS_DOMAIN_ID=${MOTION_ROS_DOMAIN_ID}"
    ;;
  start-dry)
    prepare_directories
    export MOTION_ROS_DOMAIN_ID="${MOTION_ROS_DOMAIN_ID:-142}"
    export MOTION_RT_MODE=external
    export MOTION_DRY_RUN=true
    "${compose[@]}" up --build -d motion
    echo "Motion dry-run 已启动：ROS_DOMAIN_ID=${MOTION_ROS_DOMAIN_ID}"
    ;;
  start-external)
    if [[ "${MOTION_HARDWARE_CONFIRM:-}" != "ENABLE_MOTION_HARDWARE" ]]; then
      echo "拒绝启动：必须显式设置 MOTION_HARDWARE_CONFIRM=ENABLE_MOTION_HARDWARE" >&2
      exit 3
    fi
    prepare_directories
    export MOTION_ROS_DOMAIN_ID=42
    export MOTION_RT_MODE=external
    export MOTION_DRY_RUN=false
    "${compose[@]}" up --build -d motion
    echo "Motion 现行 rt-control 适配模式已启动。请确认 rt-control 独立显示 READY。"
    ;;
  task)
    prepare_directories
    export MOTION_ROS_DOMAIN_ID="${MOTION_ROS_DOMAIN_ID:-142}"
    "${compose[@]}" --profile manual run --rm --no-deps task "$@"
    ;;
  status)
    "${compose[@]}" ps
    ;;
  logs)
    "${compose[@]}" logs -f motion
    ;;
  stop)
    "${compose[@]}" down --remove-orphans
    ;;
  shell)
    "${compose[@]}" exec motion bash
    ;;
  *)
    usage
    exit 2
    ;;
esac
