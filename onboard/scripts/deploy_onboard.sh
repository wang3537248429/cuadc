#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$SCRIPT_DIR/layout.sh"
PROJECT_DIR="$CUADC_PROJECT_DIR"
CONFIG_PATH="$CUADC_ONBOARD_CONFIG"
OFFLINE=0
START_NOW=1

usage() {
  printf '[CUADC] 用法：%s [--offline] [--no-start]\n' "$0"
  printf '  --offline   仅使用已校验的 wheelhouse，不访问网络\n'
  printf '  --no-start  安装并启用开机自启，但本次不做硬件自检也不立即启动\n'
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --offline)
      OFFLINE=1
      ;;
    --no-start)
      START_NOW=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '[CUADC] 未知参数：%s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "$EUID" -eq 0 ]]; then
  printf '[CUADC] 请以实际运行伴随程序的普通用户执行，不要直接使用 root。\n' >&2
  exit 1
fi
if [[ ! -f "$CONFIG_PATH" ]]; then
  printf '[CUADC] 缺少实机配置：%s\n' "$CONFIG_PATH" >&2
  exit 1
fi
if ! command -v sudo >/dev/null 2>&1; then
  printf '[CUADC] 一键部署需要 sudo 安装系统依赖和 systemd 单元。\n' >&2
  exit 1
fi

printf '[CUADC] 1/5 停止已安装的旧服务（首次安装会自动跳过）。\n'
if systemctl cat cuadc-onboard.target >/dev/null 2>&1; then
  sudo systemctl stop cuadc-onboard.target
fi

printf '[CUADC] 2/5 安装树莓派实机环境和两套 YOLO 模型。\n'
INSTALL_ARGUMENTS=()
if [[ "$OFFLINE" -eq 1 ]]; then
  INSTALL_ARGUMENTS+=(--offline)
fi
"$SCRIPT_DIR/install_onboard.sh" "${INSTALL_ARGUMENTS[@]}"

ONBOARD_BIN="$PROJECT_DIR/.venv-onboard/bin/cuadc-onboard"
printf '[CUADC] 3/5 校验实机配置和安全门限。\n'
"$ONBOARD_BIN" --config "$CONFIG_PATH" show-config >/dev/null

if [[ "$START_NOW" -eq 1 ]]; then
  printf '[CUADC] 4/5 检查飞控 USB、相机和当前任务模型。\n'
  if ! "$ONBOARD_BIN" --config "$CONFIG_PATH" check; then
    printf '[CUADC] 硬件自检未通过，systemd 服务未更新也未启动。\n' >&2
    printf '[CUADC] 接好 HEX Cube USB 和下视相机，确认 %s 的设备路径后重试。\n' "$CONFIG_PATH" >&2
    printf '[CUADC] 如只需预装到 SD 卡，请使用 --no-start。\n' >&2
    exit 1
  fi
else
  printf '[CUADC] 4/5 按 --no-start 跳过硬件自检和立即启动。\n'
fi

printf '[CUADC] 5/5 安装 systemd 并启用开机自启。\n'
"$SCRIPT_DIR/install_onboard_service.sh"

if [[ "$START_NOW" -eq 1 ]]; then
  if ! sudo systemctl restart cuadc-onboard.target; then
    printf '[CUADC] systemd 启动失败，请根据下方状态和日志排查。\n' >&2
    systemctl --no-pager --full status \
      cuadc-onboard-router.service cuadc-onboard.service || true
    exit 1
  fi
  if ! systemctl is-enabled --quiet cuadc-onboard.target; then
    printf '[CUADC] cuadc-onboard.target 未成功启用开机自启。\n' >&2
    exit 1
  fi
  if ! systemctl is-active --quiet cuadc-onboard-router.service \
      || ! systemctl is-active --quiet cuadc-onboard.service; then
    printf '[CUADC] 路由或算法服务没有进入 active 状态。\n' >&2
    systemctl --no-pager --full status \
      cuadc-onboard-router.service cuadc-onboard.service || true
    exit 1
  fi
  printf '\n[CUADC] 一键部署完成：路由和算法服务已启动，并已开机自启。\n'
  printf '[CUADC] 当前发布包仅支持主动模式；release.enabled=false 会拒绝启动投水流程且不改写航线。\n'
  printf '[CUADC] 状态：systemctl --no-pager --full status cuadc-onboard-router cuadc-onboard\n'
  printf '[CUADC] 日志：journalctl -u cuadc-onboard-router -u cuadc-onboard -f\n'
else
  printf '\n[CUADC] 预安装完成：已开机自启，本次未立即启动。\n'
  printf '[CUADC] 接好硬件后执行：sudo systemctl start cuadc-onboard.target\n'
fi
