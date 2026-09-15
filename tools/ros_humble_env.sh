#!/usr/bin/env bash

strip_path_entry() {
  local source_value="${1:-}"
  local blocked_prefix="$2"
  local result=""
  local entry
  local -a entries
  IFS=':' read -r -a entries <<< "$source_value"
  for entry in "${entries[@]}"; do
    [[ -z "$entry" || "$entry" == "$blocked_prefix" || "$entry" == "$blocked_prefix/"* ]] && continue
    result="${result:+$result:}$entry"
  done
  printf '%s' "$result"
}

# Remove Conda paths before CMake chooses Python; unsetting CONDA_* alone is not enough.
for conda_root in /mnt/mydisk/anaconda3 "$HOME/miniconda3" "$HOME/anaconda3" \
  "${CONDA_PREFIX:-}" "${CONDA_EXE%/bin/conda}"; do
  [[ -z "$conda_root" ]] && continue
  for path_var in PATH LD_LIBRARY_PATH PKG_CONFIG_PATH PYTHONPATH; do
    export "$path_var=$(strip_path_entry "${!path_var}" "$conda_root")"
  done
done
unset conda_root path_var
unset CONDA_DEFAULT_ENV CONDA_EXE CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONHOME
hash -r

source /opt/ros/humble/setup.bash
