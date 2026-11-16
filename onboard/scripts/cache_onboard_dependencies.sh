#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$SCRIPT_DIR/layout.sh"
PROJECT_DIR="$CUADC_PROJECT_DIR"
WHEELHOUSE="$CUADC_ONBOARD_DIR/wheelhouse"

if [[ "$(uname -m)" != "aarch64" ]]; then
  printf '[CUADC] wheel 缓存必须在 Ubuntu 22.04 aarch64 树莓派上生成；当前为 %s。\n' "$(uname -m)" >&2
  exit 1
fi
mkdir -p "$WHEELHOUSE"
python3 -m pip download \
  --dest "$WHEELHOUSE" \
  pip wheel setuptools
python3 -m pip download \
  --dest "$WHEELHOUSE" \
  -r "$CUADC_ONBOARD_REQUIREMENTS"
(
  cd "$WHEELHOUSE"
  find . -maxdepth 1 -type f ! -name SHA256SUMS -print0 \
    | sort -z \
    | xargs -0 sha256sum > SHA256SUMS
)

printf '[CUADC] 离线 wheel 已缓存到 %s。\n' "$WHEELHOUSE"
printf '[CUADC] 比赛前请保留整个项目目录，并制作一份可恢复的 SD 卡镜像。\n'
