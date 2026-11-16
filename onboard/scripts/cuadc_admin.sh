#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_PATH="${PACKAGE_DIR}/config/onboard.yaml"
ONBOARD_BIN="${PACKAGE_DIR}/.venv-onboard/bin/cuadc-onboard"
STATE_PATH="${PACKAGE_DIR}/.runtime/onboard/state.json"
MISSION_PATH="${PACKAGE_DIR}/config/route_active_mode.waypoints"

usage() {
  printf '%s\n' \
    "用法: cuadc-admin <status|parameters|mission|check|edit|apply|logs>" \
    "  status      查看服务、飞控、相机、舵机和投水门禁状态" \
    "  parameters  查看最常修改的实验参数" \
    "  mission     校验机载保存的动态投水正式航线合同" \
    "  check       只读配置、串口、模型和相机自检" \
    "  edit        用 nano/EDITOR 修改配置，不自动重启" \
    "  apply       确认未解锁后自检并重启机载服务" \
    "  logs        查看最近 80 行机载日志"
}

show_state() {
  systemctl is-active cuadc-onboard.target cuadc-onboard-router.service cuadc-onboard.service
  if [[ ! -f "${STATE_PATH}" ]]; then
    printf '状态文件尚未生成: %s\n' "${STATE_PATH}"
    return
  fi
  python3 - "${STATE_PATH}" <<'PY'
import json
import sys

state = json.load(open(sys.argv[1], encoding="utf-8"))
telemetry = state.get("telemetry", {})
vision = state.get("vision", {})
summary = {
    "status": state.get("status"),
    "message": state.get("message"),
    "mission_mode": state.get("mission_mode"),
    "mission_contract": state.get("mission_reference", {}).get("contract"),
    "release_enabled": state.get("release_enabled"),
    "release_commanded": state.get("release_commanded"),
    "payload_servo": state.get("payload_servo"),
    "armed": telemetry.get("armed"),
    "auto": telemetry.get("auto"),
    "mission_seq": telemetry.get("mission_seq"),
    "gps": state.get("gps"),
    "camera_connected": vision.get("camera_connected"),
    "camera_device": vision.get("camera_device"),
    "camera_mode": [
        vision.get("camera_width"),
        vision.get("camera_height"),
        vision.get("camera_fps"),
    ],
    "inference_count": vision.get("inference_count"),
}
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY
}

show_parameters() {
  "${PACKAGE_DIR}/.venv-onboard/bin/python" - "${CONFIG_PATH}" <<'PY'
import json
import sys
import yaml

config = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
keys = ("camera", "vision", "release", "mission")
print(json.dumps({key: config.get(key, {}) for key in keys}, ensure_ascii=False, indent=2))
PY
}

show_mission() {
  "${PACKAGE_DIR}/.venv-onboard/bin/python" - "${MISSION_PATH}" <<'PY'
import json
import sys
from pathlib import Path

from cuadc_onboard.mission import assess_mission, load_qgc_waypoints

path = Path(sys.argv[1])
if not path.is_file():
    raise SystemExit(f"航线文件不存在: {path}")
mission = load_qgc_waypoints(path)
assessment = assess_mission(
    mission,
    payload_servo=8,
    min_release_pwm=1800,
)
print(json.dumps({
    "file": str(path),
    "waypoint_count": len(mission),
    "mission_mode": assessment.mode,
    "launch_allowed": assessment.launch_allowed,
    "contract": None if assessment.contract is None else assessment.contract.__dict__,
    "errors": assessment.errors,
    "warnings": assessment.warnings,
}, ensure_ascii=False, indent=2))
raise SystemExit(0 if assessment.launch_allowed else 1)
PY
}

ensure_disarmed() {
  if [[ ! -f "${STATE_PATH}" ]]; then
    printf '拒绝重启：缺少实时状态文件 %s\n' "${STATE_PATH}" >&2
    exit 2
  fi
  python3 - "${STATE_PATH}" <<'PY'
import json
import sys

state = json.load(open(sys.argv[1], encoding="utf-8"))
telemetry = state.get("telemetry", {})
if telemetry.get("armed") is not False:
    raise SystemExit("拒绝重启：飞控不是明确的未解锁状态")
if telemetry.get("auto") is not False:
    raise SystemExit("拒绝重启：飞控不是明确的非 AUTO 状态")
print("安全检查通过：armed=false，auto=false")
PY
}

command="${1:-}"
case "${command}" in
  status)
    show_state
    ;;
  parameters)
    show_parameters
    ;;
  mission)
    show_mission
    ;;
  check)
    "${ONBOARD_BIN}" --config "${CONFIG_PATH}" check
    ;;
  edit)
    "${EDITOR:-nano}" "${CONFIG_PATH}"
    printf '%s\n' "修改完成后执行: cuadc-admin apply"
    ;;
  apply)
    ensure_disarmed
    "${ONBOARD_BIN}" --config "${CONFIG_PATH}" check --skip-camera
    sudo systemctl stop cuadc-onboard.service
    "${ONBOARD_BIN}" --config "${CONFIG_PATH}" check
    sudo systemctl restart cuadc-onboard-router.service
    sudo systemctl start cuadc-onboard.service
    show_state
    ;;
  logs)
    journalctl -u cuadc-onboard.service -n 80 --no-pager
    ;;
  *)
    usage
    exit 2
    ;;
esac
