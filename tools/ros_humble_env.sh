#!/usr/bin/env bash

strip_path_entry() {
  local source_value="${1:-}"
  local blocked_prefix="$2"
  local result=""
  local entry
  IFS=':' read -r -a entries <<< "$source_value"
  for entry in "${entries[@]}"; do
    [[ -z "$entry" || "$entry" == "$blocked_prefix"* ]] && continue
    result="${result:+$result:}$entry"
  done
  printf '%s' "$result"
}

export PATH="$(strip_path_entry "$PATH" /mnt/mydisk/anaconda3)"
export LD_LIBRARY_PATH="$(strip_path_entry "${LD_LIBRARY_PATH:-}" /mnt/mydisk/anaconda3)"
export PKG_CONFIG_PATH="$(strip_path_entry "${PKG_CONFIG_PATH:-}" /mnt/mydisk/anaconda3)"
unset CONDA_DEFAULT_ENV CONDA_EXE CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONHOME
hash -r

source /opt/ros/humble/setup.bash
