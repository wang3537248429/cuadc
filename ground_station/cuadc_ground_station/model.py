from __future__ import annotations

import copy
import math
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


COMPANION_COMPONENT_ID = 191
FLIGHT_CONTROLLER_COMPONENT_ID = 1

STATUS_NAMES_ZH = {
    0: "正在启动",
    1: "等待 GPS/HOME",
    2: "已就绪",
    3: "相机或识别失败",
    4: "航线合同无效",
    5: "投水舵机配置无效",
    6: "程序严重故障",
    7: "侦察选靶已记录",
    8: "使用保底航线",
    9: "动态攻击航线已写入",
    10: "投水命令已发送",
    11: "已观察到投水 PWM",
    12: "航线恢复校验失败",
    13: "飞行流程完成",
    14: "飞控健康故障（禁止投放）",
    15: "普通航线已通过（树莓派旁路）",
    16: "航线可飞，投水结构有误",
    17: "等待新航线稳定回读",
    99: "未知状态",
}

TASK1_TARGETS = {
    1: ("机枪兵", "machine_gunner"),
    2: ("火箭兵", "rocketeer"),
    3: ("多旋翼", "multirotor"),
    4: ("固定翼", "fixed_wing"),
    5: ("卡车", "truck"),
    6: ("防空炮", "air_defense"),
    7: ("坦克", "tank"),
    8: ("直升机", "helicopter"),
    9: ("战斗机", "fighter"),
    10: ("运输机", "transport"),
    11: ("侦察机", "recon"),
    12: ("轰炸机", "bomber"),
}

MISSION_STAGES_ZH = {
    0: "待开始 / HOME",
    1: "自动起飞",
    2: "前往侦察区 1",
    3: "前往侦察区 2",
    4: "前往侦察区 3",
    5: "侦察区入口",
    6: "扫描开始标记 9101",
    7: "侦察扫描 1",
    8: "侦察扫描 2",
    9: "侦察扫描 3",
    10: "侦察扫描 4",
    11: "扫描结束标记 9102",
    12: "选靶决策保护点",
    13: "动态半圈掉头槽",
    14: "动态释放航点槽",
    15: "投水动作槽",
    16: "动态退出航点槽",
    17: "返航下降 1",
    18: "进近降落",
    19: "自动降落",
}

MISSION_MODE_NAMES_ZH = {
    0: "等待航线稳定回读",
    1: "普通航线：树莓派旁路",
    2: "动态投水合同已通过",
    3: "航线可飞：投水合同已禁用",
    4: "侦察合同已通过（载荷永久旁路）",
}

FIELD_DEFAULTS: dict[str, int | float] = {
    "CU_STATUS": 99,
    "CU_TASK": 1,
    "CU_DETCNT": 0,
    "CU_DETVAL": -1,
    "CU_SELVAL": -1,
    "CU_REL_EN": 0,
    "CU_REL_CMD": 0,
    "CU_REL_OBS": 0,
    "CU_GPSFIX": 0,
    "CU_SATS": 0,
    "CU_MSEQ": 0,
    "CU_CAMOK": 0,
    "CU_CAMAUTO": 0,
    "CU_CAMW": 0,
    "CU_CAMH": 0,
    "CU_CAMFPS": 0,
    "CU_MPASS": 0,
    "CU_MMODE": 0,
    "CU_SCANST": -1,
    "CU_SCANEN": -1,
    "CU_RELSEQ": -1,
    "CU_LANDSEQ": -1,
    "CU_DETCF": -1.0,
    "CU_DETN": 0.0,
    "CU_DETE": 0.0,
    "CU_SELN": 0.0,
    "CU_SELE": 0.0,
    "CU_ALT": -1.0,
    "CU_AIRSPD": 0.0,
    "CU_HDOP": -1.0,
    "CU_INF": 0.0,
}

FLIGHT_FIELD_DEFAULTS: dict[str, int | float | str] = {
    "FC_MODE": "未知",
    "FC_ARMED": 0,
    "FC_SYSTEM_STATUS": -1,
    "FC_GPSFIX": 0,
    "FC_SATS": 0,
    "FC_HDOP": -1.0,
    "FC_LAT": 0.0,
    "FC_LON": 0.0,
    "FC_REL_ALT": 0.0,
    "FC_ALT": 0.0,
    "FC_AIRSPD": 0.0,
    "FC_GNDSPD": 0.0,
    "FC_CLIMB": 0.0,
    "FC_HEADING": -1.0,
    "FC_THROTTLE": 0,
    "FC_ROLL": 0.0,
    "FC_PITCH": 0.0,
    "FC_YAW": 0.0,
    "FC_BAT_V": -1.0,
    "FC_BAT_A": -1.0,
    "FC_BAT_REMAIN": -1,
    "FC_DROP_RATE": -1.0,
    "FC_RADIO_RSSI": -1,
    "FC_RADIO_REMOTE_RSSI": -1,
    "FC_MSEQ": -1,
    "FC_MTOTAL": -1,
}

ARDUPLANE_MODES = {
    0: "MANUAL",
    1: "CIRCLE",
    2: "STABILIZE",
    3: "TRAINING",
    4: "ACRO",
    5: "FBWA",
    6: "FBWB",
    7: "CRUISE",
    8: "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    13: "TAKEOFF",
    14: "AVOID_ADSB",
    15: "GUIDED",
    17: "QSTABILIZE",
    18: "QHOVER",
    19: "QLOITER",
    20: "QLAND",
    21: "QRTL",
    22: "QAUTOTUNE",
    23: "QACRO",
    24: "THERMAL",
    25: "LOITER_ALT_QLAND",
}


def decoded(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace").rstrip("\x00")
    return str(value).rstrip("\x00")


def target_name_zh(task: int, value: int) -> str:
    if value < 0:
        return "暂未识别"
    if task == 1:
        return TASK1_TARGETS.get(value, (f"未知靶标（编号 {value}）", "unknown"))[0]
    if task == 2 and 0 <= value <= 99:
        return f"{value:02d} 号数字靶"
    return f"未知靶标（编号 {value}）"


def mission_stage_zh(
    seq: int,
    values: dict[str, int | float] | None = None,
) -> str:
    """Describe a mission item using live relative contract positions.

    The legacy fixed map remains as a fallback for old TLOGs that predate the
    dynamic mission fields. New reports never assume a 20-item route.
    """

    if values is None or "CU_MMODE" not in values:
        return MISSION_STAGES_ZH.get(seq, f"航线序号 {seq}")
    mode = int(values.get("CU_MMODE", 0))
    if mode not in {2, 4}:
        return f"普通任务点 {seq}" if mode == 1 else f"航线序号 {seq}"
    scan_start = int(values.get("CU_SCANST", -1))
    scan_end = int(values.get("CU_SCANEN", -1))
    release_seq = int(values.get("CU_RELSEQ", -1))
    land_seq = int(values.get("CU_LANDSEQ", -1))
    exact = {
        scan_start: "扫描开始标记 9101",
        scan_end: "扫描结束标记 9102",
        scan_end + 1: "选靶决策保护点",
        land_seq: "自动降落",
    }
    if mode == 2:
        exact.update(
            {
                scan_end + 2: "动态半圈掉头槽",
                scan_end + 3: "动态释放航点槽",
                release_seq: "投水动作槽",
                release_seq + 1: "动态退出航点槽",
            }
        )
    if seq in exact and seq >= 0:
        return exact[seq]
    if scan_start < seq < scan_end:
        return f"侦察扫描任务点 {seq}"
    if mode == 2 and release_seq + 1 < seq < land_seq:
        return f"原始返航/降落航线任务点 {seq}"
    if mode == 4 and scan_end + 1 < seq < land_seq:
        return f"侦察后返航/降落任务点 {seq}"
    return f"航线序号 {seq}"


def mission_mode_name_zh(mode: int) -> str:
    return MISSION_MODE_NAMES_ZH.get(mode, f"未知航线模式（{mode}）")


def translate_event(raw: str) -> str:
    exact = {
        "CUADC onboard ready (RELEASE-DISABLED)": "机载程序拒绝载荷流程（主动开关未开启）",
        "CUADC onboard ready (ACTIVE)": "机载程序已就绪（主动投水模式）",
        "ONBOARD MISSION INVALID - AUTO RELEASE BLOCKED": "航线合同无效，自动投水已阻止",
        "USB camera or YOLO failed - mission fallback only": "USB 相机或识别模型失败，仅保留航线备用逻辑",
        "USB camera recovered": "USB 相机已恢复",
    }
    if raw in exact:
        return exact[raw]
    replacements = (
        ("Selected target", "已选定靶标"),
        ("release command sent", "已发送投水命令"),
        ("camera", "相机"),
        ("mission", "航线"),
    )
    translated = raw
    for source, destination in replacements:
        translated = translated.replace(source, destination)
    return translated


@dataclass
class EventRecord:
    received_at: float
    severity: int
    text_zh: str
    raw_text: str


@dataclass
class ReportSnapshot:
    values: dict[str, int | float] = field(default_factory=lambda: dict(FIELD_DEFAULTS))
    events: deque[EventRecord] = field(default_factory=lambda: deque(maxlen=100))
    flight_values: dict[str, int | float | str] = field(
        default_factory=lambda: dict(FLIGHT_FIELD_DEFAULTS)
    )
    flight_events: deque[EventRecord] = field(default_factory=lambda: deque(maxlen=50))
    last_report_at: float = 0.0
    last_heartbeat_at: float = 0.0
    last_flight_at: float = 0.0
    source_path: str = ""
    system_id: int | None = None
    flight_system_id: int | None = None
    component_id: int = COMPANION_COMPONENT_ID

    def clone(self) -> "ReportSnapshot":
        return copy.deepcopy(self)

    @property
    def task(self) -> int:
        return int(self.values.get("CU_TASK", 1))

    @property
    def detected_value(self) -> int:
        return int(self.values.get("CU_DETVAL", -1))

    @property
    def selected_value(self) -> int:
        return int(self.values.get("CU_SELVAL", -1))

    @property
    def latest_at(self) -> float:
        return max(self.last_report_at, self.last_heartbeat_at)

    def is_online(self, now: float | None = None, timeout_s: float = 5.0) -> bool:
        now = time.time() if now is None else now
        return self.latest_at > 0 and now - self.latest_at <= timeout_s

    def is_flight_controller_online(
        self, now: float | None = None, timeout_s: float = 5.0
    ) -> bool:
        now = time.time() if now is None else now
        return self.last_flight_at > 0 and now - self.last_flight_at <= timeout_s

    def grouped_zh(self, now: float | None = None, timeout_s: float = 5.0) -> dict[str, Any]:
        now = time.time() if now is None else now
        v = self.values
        f = self.flight_values
        flight_gps_fix = int(f.get("FC_GPSFIX", 0))
        flight_position_valid = flight_gps_fix >= 2
        flight_home_valid = flight_gps_fix >= 3
        detected = self.detected_value
        selected = self.selected_value
        confidence = float(v.get("CU_DETCF", -1.0))
        last_time = (
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.latest_at))
            if self.latest_at
            else "尚未收到"
        )
        return {
            "数据源": {
                "树莓派连接": "在线" if self.is_online(now, timeout_s) else "离线",
                "MAVLink身份": f"SYSID {self.system_id or '-'} / COMPID {self.component_id}",
                "飞控连接": (
                    "在线" if self.is_flight_controller_online(now, timeout_s) else "离线"
                ),
                "飞控MAVLink身份": f"SYSID {self.flight_system_id or '-'} / COMPID 1",
                "最后数据时间": last_time,
                "Mission Planner日志": self.source_path or "尚未找到",
            },
            "飞控MAVLink遥测": {
                "飞行模式": str(f.get("FC_MODE", "未知")),
                "解锁状态": "已解锁" if int(f.get("FC_ARMED", 0)) else "未解锁",
                "GPS定位类型": flight_gps_fix,
                "可见卫星数": int(f.get("FC_SATS", 0)),
                "HDOP": None if float(f.get("FC_HDOP", -1.0)) < 0 else round(float(f["FC_HDOP"]), 2),
                "纬度": round(float(f.get("FC_LAT", 0.0)), 7) if flight_position_valid else None,
                "经度": round(float(f.get("FC_LON", 0.0)), 7) if flight_position_valid else None,
                "相对HOME高度_m": round(float(f.get("FC_REL_ALT", 0.0)), 2) if flight_home_valid else None,
                "空速_m_s": round(float(f.get("FC_AIRSPD", 0.0)), 2),
                "地速_m_s": round(float(f.get("FC_GNDSPD", 0.0)), 2),
                "航向_deg": None if float(f.get("FC_HEADING", -1.0)) < 0 else round(float(f["FC_HEADING"]), 1),
                "横滚_deg": round(float(f.get("FC_ROLL", 0.0)), 1),
                "俯仰_deg": round(float(f.get("FC_PITCH", 0.0)), 1),
                "电池电压_V": None if float(f.get("FC_BAT_V", -1.0)) < 0 else round(float(f["FC_BAT_V"]), 2),
                "电池剩余_percent": None if int(f.get("FC_BAT_REMAIN", -1)) < 0 else int(f["FC_BAT_REMAIN"]),
                "通信丢包率_percent": None if float(f.get("FC_DROP_RATE", -1.0)) < 0 else round(float(f["FC_DROP_RATE"]), 2),
                "当前任务序号": int(f.get("FC_MSEQ", -1)),
                "任务总数": int(f.get("FC_MTOTAL", -1)),
                "最近飞控消息": self.flight_events[-1].text_zh if self.flight_events else "无",
            },
            "当前识别靶标": {
                "中文名称": target_name_zh(self.task, detected),
                "靶标编号": detected,
                "稳定目标数": int(v.get("CU_DETCNT", 0)),
                "置信度": None if confidence < 0 else round(confidence, 4),
                "HOME北向坐标_m": round(float(v.get("CU_DETN", 0.0)), 3),
                "HOME东向坐标_m": round(float(v.get("CU_DETE", 0.0)), 3),
            },
            "最终选靶": {
                "中文名称": target_name_zh(self.task, selected),
                "靶标编号": selected,
                "HOME北向坐标_m": round(float(v.get("CU_SELN", 0.0)), 3),
                "HOME东向坐标_m": round(float(v.get("CU_SELE", 0.0)), 3),
            },
            "任务与导航": {
                "识别任务": f"任务 {self.task}",
                "机载程序状态": STATUS_NAMES_ZH.get(int(v.get("CU_STATUS", 99)), "未知状态"),
                "航线序号": int(v.get("CU_MSEQ", 0)),
                "航线阶段": mission_stage_zh(int(v.get("CU_MSEQ", 0)), v),
                "航线通行": "已通过" if int(v.get("CU_MPASS", 0)) else "等待/未通过",
                "树莓派航线模式": mission_mode_name_zh(int(v.get("CU_MMODE", 0))),
                "GPS定位类型": int(v.get("CU_GPSFIX", 0)),
                "可见卫星数": int(v.get("CU_SATS", 0)),
                "HDOP": None if float(v.get("CU_HDOP", -1.0)) < 0 else round(float(v["CU_HDOP"]), 2),
                "相对HOME高度_m": round(float(v.get("CU_ALT", -1.0)), 2),
                "空速_m_s": round(float(v.get("CU_AIRSPD", 0.0)), 2),
                "累计推理次数": int(round(float(v.get("CU_INF", 0.0)))),
                "USB摄像头": (
                    f"{'在线' if int(v.get('CU_CAMOK', 0)) else '离线'} / "
                    f"{'自动选择' if int(v.get('CU_CAMAUTO', 0)) else '固定设备'} / "
                    f"{int(v.get('CU_CAMW', 0))}×{int(v.get('CU_CAMH', 0))} "
                    f"@ {int(v.get('CU_CAMFPS', 0))}fps"
                ),
            },
            "投水安全状态": {
                "主动投水": "允许" if int(v.get("CU_REL_EN", 0)) else "禁止（配置未开启）",
                "已发送命令": "是" if int(v.get("CU_REL_CMD", 0)) else "否",
                "已观察到PWM输出": "是" if int(v.get("CU_REL_OBS", 0)) else "否",
            },
        }


class ReportAggregator:
    def __init__(
        self,
        component_id: int = COMPANION_COMPONENT_ID,
        system_id: int | None = None,
    ) -> None:
        self.component_id = component_id
        self.system_id = system_id
        self.snapshot = ReportSnapshot(component_id=component_id)

    def set_source(self, path: Path | str) -> None:
        self.snapshot.source_path = str(path)

    def update(self, message: Any, received_at: float | None = None) -> bool:
        try:
            component = int(message.get_srcComponent())
            source_system = int(message.get_srcSystem())
        except (AttributeError, TypeError, ValueError):
            return False
        if self.system_id is not None and source_system != self.system_id:
            return False
        received_at = float(
            received_at
            if received_at is not None
            else getattr(message, "_timestamp", None) or time.time()
        )
        if component == FLIGHT_CONTROLLER_COMPONENT_ID:
            return self._update_flight_controller(message, source_system, received_at)
        if component != self.component_id:
            return False
        if self.snapshot.system_id not in (None, 0) and source_system == 0:
            return False
        if self.snapshot.system_id == 0 and source_system > 0:
            self.snapshot.values = dict(FIELD_DEFAULTS)
            self.snapshot.events.clear()
            self.snapshot.last_report_at = 0.0
            self.snapshot.last_heartbeat_at = 0.0
        self.snapshot.system_id = source_system
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            self.snapshot.last_heartbeat_at = received_at
            if "CU_STATUS" not in self.snapshot.values or self.snapshot.values["CU_STATUS"] == 99:
                self.snapshot.values["CU_STATUS"] = int(message.custom_mode)
            return True
        if message_type in {"NAMED_VALUE_INT", "NAMED_VALUE_FLOAT"}:
            name = decoded(message.name)
            if not name.startswith("CU_"):
                return False
            value: int | float
            if message_type == "NAMED_VALUE_INT":
                value = int(message.value)
            else:
                value = float(message.value)
            self.snapshot.values[name] = value
            self.snapshot.last_report_at = received_at
            return True
        if message_type == "STATUSTEXT":
            raw = decoded(message.text)
            event = EventRecord(
                received_at=received_at,
                severity=int(message.severity),
                text_zh=translate_event(raw),
                raw_text=raw,
            )
            if self.snapshot.events:
                previous = self.snapshot.events[-1]
                # The onboard service deliberately repeats READY status text.
                # Keep a state-change history instead of flooding the operator
                # with the same consecutive event every five seconds.  A READY
                # event after a different failure/recovery event is retained.
                if previous.raw_text == raw:
                    return False
            self.snapshot.events.append(event)
            return True
        return False

    def _update_flight_controller(
        self, message: Any, source_system: int, received_at: float
    ) -> bool:
        message_type = message.get_type()
        f = self.snapshot.flight_values
        changed = True
        if message_type == "HEARTBEAT":
            base_mode = int(getattr(message, "base_mode", 0))
            custom_mode = int(getattr(message, "custom_mode", 0))
            f["FC_ARMED"] = 1 if base_mode & 128 else 0
            f["FC_MODE"] = ARDUPLANE_MODES.get(custom_mode, f"MODE {custom_mode}")
            f["FC_SYSTEM_STATUS"] = int(getattr(message, "system_status", -1))
        elif message_type == "GPS_RAW_INT":
            f["FC_GPSFIX"] = int(getattr(message, "fix_type", 0))
            f["FC_SATS"] = int(getattr(message, "satellites_visible", 0))
            eph = int(getattr(message, "eph", 65535))
            f["FC_HDOP"] = -1.0 if eph == 65535 else eph / 100.0
        elif message_type == "GLOBAL_POSITION_INT":
            f["FC_LAT"] = float(getattr(message, "lat", 0)) / 1e7
            f["FC_LON"] = float(getattr(message, "lon", 0)) / 1e7
            f["FC_ALT"] = float(getattr(message, "alt", 0)) / 1000.0
            f["FC_REL_ALT"] = float(getattr(message, "relative_alt", 0)) / 1000.0
            heading = int(getattr(message, "hdg", 65535))
            if heading != 65535:
                f["FC_HEADING"] = heading / 100.0
        elif message_type == "VFR_HUD":
            f["FC_AIRSPD"] = float(getattr(message, "airspeed", 0.0))
            f["FC_GNDSPD"] = float(getattr(message, "groundspeed", 0.0))
            f["FC_HEADING"] = float(getattr(message, "heading", -1.0))
            f["FC_THROTTLE"] = int(getattr(message, "throttle", 0))
            f["FC_CLIMB"] = float(getattr(message, "climb", 0.0))
        elif message_type == "ATTITUDE":
            f["FC_ROLL"] = math.degrees(float(getattr(message, "roll", 0.0)))
            f["FC_PITCH"] = math.degrees(float(getattr(message, "pitch", 0.0)))
            f["FC_YAW"] = math.degrees(float(getattr(message, "yaw", 0.0)))
        elif message_type == "SYS_STATUS":
            voltage = int(getattr(message, "voltage_battery", 65535))
            current = int(getattr(message, "current_battery", -1))
            f["FC_BAT_V"] = -1.0 if voltage == 65535 else voltage / 1000.0
            f["FC_BAT_A"] = -1.0 if current < 0 else current / 100.0
            f["FC_BAT_REMAIN"] = int(getattr(message, "battery_remaining", -1))
            drop_rate = int(getattr(message, "drop_rate_comm", 65535))
            f["FC_DROP_RATE"] = -1.0 if drop_rate == 65535 else drop_rate / 100.0
        elif message_type == "MISSION_CURRENT":
            f["FC_MSEQ"] = int(getattr(message, "seq", -1))
            f["FC_MTOTAL"] = int(getattr(message, "total", -1))
        elif message_type == "RADIO_STATUS":
            f["FC_RADIO_RSSI"] = int(getattr(message, "rssi", -1))
            f["FC_RADIO_REMOTE_RSSI"] = int(getattr(message, "remrssi", -1))
        elif message_type == "STATUSTEXT":
            raw = decoded(getattr(message, "text", ""))
            event = EventRecord(
                received_at=received_at,
                severity=int(getattr(message, "severity", 6)),
                text_zh=raw,
                raw_text=raw,
            )
            if self.snapshot.flight_events and self.snapshot.flight_events[-1].raw_text == raw:
                return False
            self.snapshot.flight_events.append(event)
        else:
            changed = False
        if changed:
            self.snapshot.flight_system_id = source_system
            self.snapshot.last_flight_at = received_at
        return changed
