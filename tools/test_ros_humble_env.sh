#!/usr/bin/env bash
set -eo pipefail

# Include inherited Conda paths with no activation metadata, plus a custom install.
export CONDA_PREFIX=/tmp/custom-conda/envs/test
export CONDA_EXE=/tmp/custom-conda/bin/conda
for name in PATH LD_LIBRARY_PATH PKG_CONFIG_PATH PYTHONPATH; do
  export "$name=$HOME/miniconda3/envs/writ/bin:$HOME/miniconda3/bin:/tmp/custom-conda/lib:${!name}"
done
source "$(dirname "${BASH_SOURCE[0]}")/ros_humble_env.sh"
for name in PATH LD_LIBRARY_PATH PKG_CONFIG_PATH PYTHONPATH; do
  [[ "${!name}" != *"$HOME/miniconda3"* && "${!name}" != *'/tmp/custom-conda/'* ]]
done
[[ "$(strip_path_entry '/opt/conda/bin:/opt/conda-extra/bin:/usr/bin' /opt/conda)" == '/opt/conda-extra/bin:/usr/bin' ]]
[[ "$(command -v python3)" == /usr/bin/python3 ]]
python3 -c 'import catkin_pkg, ament_package'
echo 'ROS Humble environment check passed'
