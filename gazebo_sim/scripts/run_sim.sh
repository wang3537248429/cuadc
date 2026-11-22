#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
用法：run_sim.sh [--no-vision] [仿真参数]

--no-vision 允许在未准备靶标图片的干净源码检出中启动基础仿真。
             该模式不会执行 YOLO 识别，完整验收仍需使用默认模式。
EOF
}

vision_required=1
arguments=()
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --no-vision)
      vision_required=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      arguments+=("$1")
      ;;
  esac
  shift
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$script_dir/layout.sh"
project_dir="$CUADC_PROJECT_DIR"
cd "$project_dir"

if [[ "$vision_required" == "1" && -f "$project_dir/../scripts/check_local_resources.py" ]]; then
  python3 "$project_dir/../scripts/check_local_resources.py" --project gazebo_sim --require
elif [[ "$vision_required" == "1" ]]; then
  sha256sum --check assets/targets/REFERENCE_SHA256SUMS
  sha256sum --check models/vision/MODEL_SHA256SUMS
fi

if [[ ! -x "${project_dir}/.venv/bin/python" ]] \
  || [[ ! -x "${project_dir}/.deps/runtime/ardupilot-4.6.3/arduplane" ]]; then
  printf '[CUADC] 仿真环境尚未安装完整，请先运行 %s/bootstrap.sh\n' "$script_dir" >&2
  exit 1
fi
if [[ "$vision_required" == "1" ]] \
  && { [[ ! -f "${project_dir}/models/vision/task1_v4m_best.pt" ]] \
    || [[ ! -f "${project_dir}/models/vision/task2_sim_digits_v1.pt" ]]; }; then
  printf '[CUADC] 视觉模型缺失；请先准备资源，或使用 %s --no-vision\n' "$0" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="$project_dir/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1

if [[ "${CUADC_HEADLESS:-0}" == "1" ]]; then
  arguments+=(--headless)
fi
if [[ "$vision_required" == "0" ]]; then
  arguments+=(--no-vision)
fi

exec "${project_dir}/.venv/bin/python" -m cuadc_sim.supervisor "${arguments[@]}"
