#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$SCRIPT_DIR/layout.sh"
PROJECT_DIR="$CUADC_PROJECT_DIR"
ONBOARD_VENV="$PROJECT_DIR/.venv-onboard"
WHEELHOUSE="$CUADC_ONBOARD_DIR/wheelhouse"
OFFLINE=0

if [[ "${1:-}" == "--offline" ]]; then
  OFFLINE=1
  shift
fi
if [[ "$#" -ne 0 ]]; then
  printf '[CUADC] 用法：%s [--offline]\n' "$0" >&2
  exit 2
fi

if [[ "$EUID" -eq 0 ]]; then
  printf '[CUADC] 请用普通用户运行；脚本只在安装 apt 包和用户组时调用 sudo。\n' >&2
  exit 1
fi
if [[ "$(uname -m)" != "aarch64" ]]; then
  printf '[CUADC] 实机安装只支持 Ubuntu 22.04 aarch64；当前为 %s。\n' "$(uname -m)" >&2
  exit 1
fi
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  printf '[CUADC] 需要 Ubuntu 22.04，当前为 %s %s。\n' "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2
  exit 1
fi

if [[ "$OFFLINE" -eq 0 ]]; then
  sudo apt-get update
  sudo apt-get install -y \
    python3-pip python3-venv python3-dev build-essential pkg-config \
    v4l-utils usbutils libgl1 libglib2.0-0 libopenblas0 libgomp1
else
  if [[ ! -d "$WHEELHOUSE" ]]; then
    printf '[CUADC] 离线安装缺少 wheelhouse；请在同架构树莓派联网时先运行 cache_onboard_dependencies.sh。\n' >&2
    exit 1
  fi
  if [[ ! -f "$WHEELHOUSE/SHA256SUMS" ]]; then
    printf '[CUADC] wheelhouse 缺少 SHA256SUMS，拒绝使用未校验的离线依赖。\n' >&2
    exit 1
  fi
  (cd "$WHEELHOUSE" && sha256sum --check SHA256SUMS)
  for command in python3 gcc pkg-config; do
    if ! command -v "$command" >/dev/null 2>&1; then
      printf '[CUADC] 离线系统缺少 %s；请恢复赛前 SD 卡镜像或预装系统依赖。\n' "$command" >&2
      exit 1
    fi
  done
fi

sudo usermod -aG dialout,video "$(id -un)"

python3 -m venv "$ONBOARD_VENV"
if [[ "$OFFLINE" -eq 1 ]]; then
  "$ONBOARD_VENV/bin/python" -m pip install \
    --no-index --find-links "$WHEELHOUSE" \
    pip wheel setuptools
  "$ONBOARD_VENV/bin/python" -m pip install \
    --no-index --find-links "$WHEELHOUSE" \
    -r "$CUADC_ONBOARD_REQUIREMENTS"
else
  "$ONBOARD_VENV/bin/python" -m pip install --upgrade pip wheel setuptools
  "$ONBOARD_VENV/bin/python" -m pip install -r "$CUADC_ONBOARD_REQUIREMENTS"
fi
"$ONBOARD_VENV/bin/python" -m pip install --no-deps -e "$PROJECT_DIR"

if ! (cd "$PROJECT_DIR" && sha256sum --status --check models/vision/MODEL_SHA256SUMS); then
  TASK1_TEMP="$(mktemp "$PROJECT_DIR/models/vision/.task1.XXXXXX.pt")"
  TASK2_TEMP="$(mktemp "$PROJECT_DIR/models/vision/.task2.XXXXXX.pt")"
  cleanup_models() {
    rm -f -- "$TASK1_TEMP" "$TASK2_TEMP"
  }
  trap cleanup_models EXIT
  gzip -cd "$PROJECT_DIR/models/vision/task1_v4m_best.pt.gz" > "$TASK1_TEMP"
  gzip -cd "$PROJECT_DIR/models/vision/task2_sim_digits_v1.pt.gz" > "$TASK2_TEMP"
  mv -f -- "$TASK1_TEMP" "$PROJECT_DIR/models/vision/task1_v4m_best.pt"
  mv -f -- "$TASK2_TEMP" "$PROJECT_DIR/models/vision/task2_sim_digits_v1.pt"
  trap - EXIT
fi
(cd "$PROJECT_DIR" && sha256sum --check models/vision/MODEL_SHA256SUMS)

printf '\n[CUADC] 树莓派伴随环境安装完成。\n'
printf '[CUADC] 重新登录一次，使 dialout/video 用户组生效。\n'
printf '[CUADC] 然后执行：%s/bin/cuadc-onboard check\n' "$ONBOARD_VENV"
