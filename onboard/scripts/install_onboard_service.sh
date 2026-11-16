#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$SCRIPT_DIR/layout.sh"
PROJECT_DIR="$CUADC_PROJECT_DIR"
SERVICE_TEMPLATES=(
  "$CUADC_ONBOARD_SYSTEMD_DIR/cuadc-onboard-router.service.in"
  "$CUADC_ONBOARD_SYSTEMD_DIR/cuadc-onboard.service.in"
  "$CUADC_ONBOARD_SYSTEMD_DIR/cuadc-onboard.target.in"
)
SERVICE_NAMES=(
  "cuadc-onboard-router.service"
  "cuadc-onboard.service"
  "cuadc-onboard.target"
)
SERVICE_USER="$(id -un)"
SERVICE_GROUP="$(id -gn)"
UDEV_RULE_SOURCE="$PROJECT_DIR/udev/99-cuadc-flight-controller.rules"
UDEV_RULE_DESTINATION="/etc/udev/rules.d/99-cuadc-flight-controller.rules"
TEMP_FILES=("$(mktemp)" "$(mktemp)" "$(mktemp)")
trap 'rm -f -- "${TEMP_FILES[@]}"' EXIT

if [[ "$EUID" -eq 0 ]]; then
  printf '[CUADC] 请以实际运行伴随服务的普通用户执行本脚本。\n' >&2
  exit 1
fi
if [[ ! -x "$PROJECT_DIR/.venv-onboard/bin/cuadc-onboard" ]]; then
  printf '[CUADC] 缺少 .venv-onboard，请先执行 %s/install_onboard.sh。\n' "$SCRIPT_DIR" >&2
  exit 1
fi
mkdir -p \
  "$PROJECT_DIR/.runtime/onboard/tlogs" \
  "$PROJECT_DIR/.runtime/onboard/ultralytics" \
  "$PROJECT_DIR/.runtime/onboard/torch-cache" \
  "$PROJECT_DIR/.runtime/onboard/matplotlib" \
  "$PROJECT_DIR/runs"
for index in "${!SERVICE_TEMPLATES[@]}"; do
  sed \
    -e "s|@USER@|$SERVICE_USER|g" \
    -e "s|@GROUP@|$SERVICE_GROUP|g" \
    -e "s|@PROJECT_DIR@|$PROJECT_DIR|g" \
    -e "s|@CONFIG_PATH@|$CUADC_ONBOARD_CONFIG|g" \
    -e "s|@WAIT_SCRIPT@|$SCRIPT_DIR/wait_onboard_devices.sh|g" \
    "${SERVICE_TEMPLATES[$index]}" > "${TEMP_FILES[$index]}"
  sudo install -m 0644 \
    "${TEMP_FILES[$index]}" \
    "/etc/systemd/system/${SERVICE_NAMES[$index]}"
done
if [[ ! -f "$UDEV_RULE_SOURCE" ]]; then
  printf '[CUADC] 缺少飞控 udev 规则：%s\n' "$UDEV_RULE_SOURCE" >&2
  exit 1
fi
sudo install -m 0644 "$UDEV_RULE_SOURCE" "$UDEV_RULE_DESTINATION"
sudo udevadm control --reload-rules
# Apply the ModemManager ignore property to an already-connected Cube before
# starting the router; the same rule runs normally on every later boot.
sudo udevadm trigger \
  --action=change \
  --subsystem-match=tty \
  --property-match=ID_VENDOR_ID=2dae \
  --property-match=ID_MODEL_ID=1058
sudo udevadm settle --timeout=10
sudo systemctl daemon-reload
sudo systemctl enable cuadc-onboard.target

printf '[CUADC] 路由、伴随服务、CubePilot udev 规则和总 target 已安装，但尚未启动。\n'
printf '[CUADC] 完成配置和 check 后启动：sudo systemctl start cuadc-onboard.target\n'
