#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$script_dir/layout.sh"
project_dir="$CUADC_PROJECT_DIR"
cd "$project_dir"

if [[ -f "$project_dir/../scripts/check_local_resources.py" ]]; then
  python3 "$project_dir/../scripts/check_local_resources.py" --project gazebo_sim --require
else
  sha256sum --check assets/targets/REFERENCE_SHA256SUMS
  sha256sum --check models/vision/MODEL_SHA256SUMS
fi

if [[ ! -x "${project_dir}/.venv/bin/python" ]] \
  || [[ ! -x "${project_dir}/.deps/runtime/ardupilot-4.6.3/arduplane" ]] \
  || [[ ! -f "${project_dir}/models/vision/task1_v4m_best.pt" ]] \
  || [[ ! -f "${project_dir}/models/vision/task2_sim_digits_v1.pt" ]]; then
  printf '[CUADC] 仿真环境尚未安装完整，请先运行 %s/bootstrap.sh\n' "$script_dir" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="$project_dir/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1

arguments=()
if [[ "${CUADC_HEADLESS:-0}" == "1" ]]; then
  arguments+=(--headless)
fi
arguments+=("$@")

exec "${project_dir}/.venv/bin/python" -m cuadc_sim.supervisor "${arguments[@]}"
