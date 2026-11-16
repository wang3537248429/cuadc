from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cuadc_ground_station.model import (  # noqa: E402
    ReportAggregator,
    STATUS_NAMES_ZH,
    mission_stage_zh,
    mission_mode_name_zh,
    target_name_zh,
    translate_event,
)


def test_flight_controller_health_fault_status_is_translated():
    assert STATUS_NAMES_ZH[14] == "飞控健康故障（禁止投放）"
    assert STATUS_NAMES_ZH[15] == "普通航线已通过（树莓派旁路）"


class Message(SimpleNamespace):
    def get_srcComponent(self):
        return self.component

    def get_srcSystem(self):
        return self.system

    def get_type(self):
        return self.message_type


def test_target_values_are_translated_to_competition_chinese_names():
    assert target_name_zh(1, 2) == "火箭兵"
    assert target_name_zh(1, 12) == "轰炸机"
    assert target_name_zh(2, 7) == "07 号数字靶"
    assert target_name_zh(1, -1) == "暂未识别"


def test_flight_controller_messages_are_ignored_and_component_191_is_grouped():
    aggregator = ReportAggregator()
    ignored = Message(
        component=1,
        system=1,
        message_type="NAMED_VALUE_INT",
        name="CU_DETVAL",
        value=12,
    )
    accepted = Message(
        component=191,
        system=1,
        message_type="NAMED_VALUE_INT",
        name="CU_DETVAL",
        value=2,
    )
    assert not aggregator.update(ignored, received_at=100.0)
    assert aggregator.update(accepted, received_at=101.0)
    grouped = aggregator.snapshot.grouped_zh(now=102.0)
    assert grouped["当前识别靶标"]["中文名称"] == "火箭兵"
    assert grouped["数据源"]["树莓派连接"] == "在线"


def test_repeated_ready_events_are_collapsed():
    aggregator = ReportAggregator()
    event = Message(
        component=191,
        system=1,
        message_type="STATUSTEXT",
        severity=5,
        text="CUADC onboard ready (RELEASE-DISABLED)",
    )
    assert aggregator.update(event, received_at=100.0)
    assert not aggregator.update(event, received_at=105.0)
    assert not aggregator.update(event, received_at=500.0)
    assert len(aggregator.snapshot.events) == 1
    assert "主动开关未开启" in aggregator.snapshot.events[0].text_zh


def test_route_sequence_is_explained_in_chinese():
    assert mission_stage_zh(6) == "扫描开始标记 9101"
    assert mission_stage_zh(11) == "扫描结束标记 9102"
    assert mission_stage_zh(15) == "投水动作槽"
    assert mission_stage_zh(19) == "自动降落"
    assert translate_event("USB camera recovered") == "USB 相机已恢复"


def test_dynamic_route_pass_and_relative_contract_are_explained():
    values = {
        "CU_MMODE": 2,
        "CU_SCANST": 9,
        "CU_SCANEN": 15,
        "CU_RELSEQ": 19,
        "CU_LANDSEQ": 27,
    }
    assert mission_stage_zh(9, values) == "扫描开始标记 9101"
    assert mission_stage_zh(12, values) == "侦察扫描任务点 12"
    assert mission_stage_zh(19, values) == "投水动作槽"
    assert mission_stage_zh(24, values) == "原始返航/降落航线任务点 24"
    assert mission_stage_zh(27, values) == "自动降落"
    assert mission_mode_name_zh(1) == "普通航线：树莓派旁路"

    aggregator = ReportAggregator()
    for name, value in (
        ("CU_MPASS", 1),
        ("CU_MMODE", 1),
        ("CU_MSEQ", 4),
        ("CU_CAMOK", 1),
        ("CU_CAMAUTO", 1),
        ("CU_CAMW", 1280),
        ("CU_CAMH", 720),
        ("CU_CAMFPS", 30),
    ):
        assert aggregator.update(
            Message(
                component=191,
                system=1,
                message_type="NAMED_VALUE_INT",
                name=name,
                value=value,
            ),
            received_at=100.0,
        )
    grouped = aggregator.snapshot.grouped_zh(now=101.0)
    assert grouped["任务与导航"]["航线通行"] == "已通过"
    assert grouped["任务与导航"]["树莓派航线模式"] == "普通航线：树莓派旁路"
    assert grouped["任务与导航"]["航线阶段"] == "普通任务点 4"
    assert grouped["任务与导航"]["USB摄像头"] == (
        "在线 / 自动选择 / 1280×720 @ 30fps"
    )


def test_recon_only_route_is_explained_without_release_slots():
    values = {
        "CU_MMODE": 4,
        "CU_SCANST": 2,
        "CU_SCANEN": 41,
        "CU_RELSEQ": -1,
        "CU_LANDSEQ": 50,
    }

    assert mission_stage_zh(2, values) == "扫描开始标记 9101"
    assert mission_stage_zh(20, values) == "侦察扫描任务点 20"
    assert mission_stage_zh(41, values) == "扫描结束标记 9102"
    assert mission_stage_zh(42, values) == "选靶决策保护点"
    assert mission_stage_zh(45, values) == "侦察后返航/降落任务点 45"
    assert mission_stage_zh(50, values) == "自动降落"
    assert mission_mode_name_zh(4) == "侦察合同已通过（载荷永久旁路）"


def test_flight_controller_mavlink_is_grouped_without_accepting_its_cu_fields():
    aggregator = ReportAggregator()
    heartbeat = Message(
        component=1,
        system=1,
        message_type="HEARTBEAT",
        base_mode=0,
        custom_mode=0,
        system_status=3,
    )
    gps = Message(
        component=1,
        system=1,
        message_type="GPS_RAW_INT",
        fix_type=3,
        satellites_visible=12,
        eph=85,
    )
    battery = Message(
        component=1,
        system=1,
        message_type="SYS_STATUS",
        voltage_battery=15770,
        current_battery=123,
        battery_remaining=99,
        drop_rate_comm=0,
    )
    assert aggregator.update(heartbeat, received_at=100.0)
    assert aggregator.update(gps, received_at=101.0)
    assert aggregator.update(battery, received_at=102.0)
    grouped = aggregator.snapshot.grouped_zh(now=103.0)
    assert grouped["飞控MAVLink遥测"]["飞行模式"] == "MANUAL"
    assert grouped["飞控MAVLink遥测"]["解锁状态"] == "未解锁"
    assert grouped["飞控MAVLink遥测"]["可见卫星数"] == 12
    assert grouped["飞控MAVLink遥测"]["电池电压_V"] == 15.77
    assert grouped["飞控MAVLink遥测"]["通信丢包率_percent"] == 0.0


def test_zero_system_companion_history_cannot_overwrite_live_vehicle():
    aggregator = ReportAggregator()
    live = Message(
        component=191,
        system=1,
        message_type="NAMED_VALUE_INT",
        name="CU_DETVAL",
        value=2,
    )
    stale = Message(
        component=191,
        system=0,
        message_type="NAMED_VALUE_INT",
        name="CU_DETVAL",
        value=12,
    )
    assert aggregator.update(live, received_at=100.0)
    assert not aggregator.update(stale, received_at=101.0)
    assert aggregator.snapshot.detected_value == 2
