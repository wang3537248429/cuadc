#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$SCRIPT_DIR/layout.sh"
PROJECT_DIR="$CUADC_PROJECT_DIR"
CONFIG_PATH="$CUADC_ONBOARD_CONFIG"
WAIT_CAMERA=0
WAIT_SERIAL=0
TIMEOUT_S=30

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG_PATH="$2"
      shift 2
      ;;
    --camera)
      WAIT_CAMERA=1
      shift
      ;;
    --serial)
      WAIT_SERIAL=1
      shift
      ;;
    --timeout)
      TIMEOUT_S="$2"
      shift 2
      ;;
    *)
      printf '[CUADC] 未知设备等待参数：%s\n' "$1" >&2
      exit 2
      ;;
  esac
done

if [[ "$WAIT_CAMERA" -eq 0 && "$WAIT_SERIAL" -eq 0 ]]; then
  WAIT_CAMERA=1
  WAIT_SERIAL=1
fi
if ! [[ "$TIMEOUT_S" =~ ^[0-9]+$ ]] || [[ "$TIMEOUT_S" -lt 1 ]]; then
  printf '[CUADC] --timeout 必须是正整数秒。\n' >&2
  exit 2
fi

PYTHON_BIN="$PROJECT_DIR/.venv-onboard/bin/python"
if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN="python3"
fi
mapfile -t DEVICES < <(
  "$PYTHON_BIN" - "$CONFIG_PATH" "$WAIT_CAMERA" "$WAIT_SERIAL" <<'PY'
import sys
from pathlib import Path
import yaml

config = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8")) or {}
if sys.argv[2] == "1":
    print(config.get("camera", {}).get(
        "device",
        "auto",
    ))
if sys.argv[3] == "1":
    print(config.get("telemetry_router", {}).get(
        "serial_device",
        "/dev/cuadc-flight-controller",
    ))
PY
)

camera_ready() {
  local candidate driver_path
  shopt -s nullglob
  for candidate in /dev/v4l/by-id/*-video-index0; do
    if [[ -c "$candidate" ]]; then
      return 0
    fi
  done
  for candidate in /dev/video*; do
    if [[ ! -c "$candidate" ]]; then
      continue
    fi
    if [[ "$(cat "/sys/class/video4linux/${candidate##*/}/index" 2>/dev/null || true)" != "0" ]]; then
      continue
    fi
    driver_path="$(readlink -f "/sys/class/video4linux/${candidate##*/}/device/driver" 2>/dev/null || true)"
    if [[ "${driver_path##*/}" == "uvcvideo" ]]; then
      return 0
    fi
  done
  return 1
}

DEADLINE=$((SECONDS + TIMEOUT_S))
while true; do
  MISSING=()
  for device in "${DEVICES[@]}"; do
    if [[ "${device,,}" == "auto" ]]; then
      if ! camera_ready; then
        MISSING+=("auto USB UVC camera")
      fi
      continue
    fi
    if [[ ! -e "$device" ]]; then
      MISSING+=("$device")
    fi
  done
  if [[ "${#MISSING[@]}" -eq 0 ]]; then
    printf '[CUADC] 所需设备已就绪：%s\n' "${DEVICES[*]}"
    exit 0
  fi
  if [[ "$SECONDS" -ge "$DEADLINE" ]]; then
    printf '[CUADC] 等待设备超时：%s\n' "${MISSING[*]}" >&2
    exit 1
  fi
  sleep 1
done
