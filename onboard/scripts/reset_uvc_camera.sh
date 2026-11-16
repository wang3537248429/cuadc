#!/usr/bin/env bash
set -euo pipefail

CONFIG_PATH=""
PYTHON_BIN=""
TIMEOUT_S=60

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG_PATH="$2"
      shift 2
      ;;
    --python)
      PYTHON_BIN="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT_S="$2"
      shift 2
      ;;
    *)
      printf '[CUADC] 未知 UVC 复位参数：%s\n' "$1" >&2
      exit 2
      ;;
  esac
done

if [[ "$EUID" -ne 0 ]]; then
  printf '[CUADC] UVC 复位助手必须由 systemd 以 root 身份运行。\n' >&2
  exit 1
fi
if [[ -z "$CONFIG_PATH" || ! -f "$CONFIG_PATH" ]]; then
  printf '[CUADC] 缺少有效的 --config。\n' >&2
  exit 2
fi
if [[ -z "$PYTHON_BIN" || ! -x "$PYTHON_BIN" ]]; then
  printf '[CUADC] 缺少有效的 --python。\n' >&2
  exit 2
fi
if ! [[ "$TIMEOUT_S" =~ ^[0-9]+$ ]] || [[ "$TIMEOUT_S" -lt 1 ]]; then
  printf '[CUADC] --timeout 必须是正整数秒。\n' >&2
  exit 2
fi

CAMERA_DEVICE="$(
  "$PYTHON_BIN" - "$CONFIG_PATH" <<'PY'
import sys
from pathlib import Path
import yaml
from cuadc_onboard.camera_device import discover_camera_devices, is_auto_camera_device

config = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8")) or {}
configured = str(config.get("camera", {}).get("device", "auto"))
if is_auto_camera_device(configured):
    devices = discover_camera_devices()
    print(devices[0] if devices else "")
else:
    print(configured)
PY
)"

if [[ -z "$CAMERA_DEVICE" || ! -c "$CAMERA_DEVICE" ]]; then
  printf '[CUADC] UVC 设备尚不存在，交给后续设备等待处理：%s\n' "$CAMERA_DEVICE"
  exit 0
fi

SYS_PATH="$(udevadm info --query=path --name="$CAMERA_DEVICE")"
NODE="$(realpath -e "/sys$SYS_PATH")"
if [[ "$NODE" != */video4linux/* ]]; then
  printf '[CUADC] 拒绝复位非 video4linux 设备：%s\n' "$NODE" >&2
  exit 1
fi

USB_NODE=""
FOUND_UVC=0
while [[ "$NODE" != "/sys" && "$NODE" != "/" ]]; do
  if [[ -L "$NODE/driver" ]]; then
    DRIVER_NAME="$(basename "$(readlink -f "$NODE/driver")")"
    if [[ "$DRIVER_NAME" == "uvcvideo" ]]; then
      FOUND_UVC=1
    fi
  fi
  if [[ -f "$NODE/idVendor" && -f "$NODE/idProduct" ]]; then
    USB_NODE="$NODE"
    break
  fi
  NODE="$(dirname "$NODE")"
done

if [[ "$FOUND_UVC" -ne 1 || -z "$USB_NODE" ]]; then
  printf '[CUADC] 拒绝复位：%s 不是可定位父 USB 的 UVC 设备。\n' "$CAMERA_DEVICE" >&2
  exit 1
fi

USB_ID="$(basename "$USB_NODE")"
if ! [[ "$USB_ID" =~ ^[0-9]+-[0-9]+([.][0-9]+)*$ ]]; then
  printf '[CUADC] 拒绝不安全的 USB 设备标识：%s\n' "$USB_ID" >&2
  exit 1
fi
if [[ "$(basename "$(readlink -f "$USB_NODE/driver")")" != "usb" ]]; then
  printf '[CUADC] 拒绝复位非 usb 核心驱动设备：%s\n' "$USB_ID" >&2
  exit 1
fi

VENDOR="$(<"$USB_NODE/idVendor")"
PRODUCT="$(<"$USB_NODE/idProduct")"
printf '[CUADC] 复位 UVC 相机 USB %s（%s:%s）。\n' "$USB_ID" "$VENDOR" "$PRODUCT"

BOUND=0
restore_binding() {
  if [[ "$BOUND" -eq 0 ]]; then
    printf '%s' "$USB_ID" > /sys/bus/usb/drivers/usb/bind 2>/dev/null || true
  fi
}
trap restore_binding EXIT

printf '%s' "$USB_ID" > /sys/bus/usb/drivers/usb/unbind
sleep 1
printf '%s' "$USB_ID" > /sys/bus/usb/drivers/usb/bind
BOUND=1
trap - EXIT

udevadm settle --timeout="$TIMEOUT_S" || true
DEADLINE=$((SECONDS + TIMEOUT_S))
while [[ ! -c "$CAMERA_DEVICE" ]]; do
  if [[ "$SECONDS" -ge "$DEADLINE" ]]; then
    printf '[CUADC] UVC 复位后设备未恢复：%s\n' "$CAMERA_DEVICE" >&2
    exit 1
  fi
  sleep 1
done
sleep 2
printf '[CUADC] UVC 相机复位完成：%s\n' "$CAMERA_DEVICE"
