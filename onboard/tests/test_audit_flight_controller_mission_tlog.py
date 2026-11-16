from __future__ import annotations

from dataclasses import replace
import hashlib
import importlib.util
from pathlib import Path
import struct
import sys
import time

from pymavlink import mavutil

from cuadc_onboard.config import OnboardConfig, RuntimeConfig
from cuadc_onboard.mission import (
    MissionItem,
    load_qgc_waypoints,
    save_patch_backup,
)


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/audit_flight_controller_mission_tlog.py"
SPEC = importlib.util.spec_from_file_location("cuadc_mission_tlog_audit", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
audit_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = audit_module
SPEC.loader.exec_module(audit_module)


def _encoder() -> mavutil.mavlink.MAVLink:
    return mavutil.mavlink.MAVLink(
        None,
        srcSystem=1,
        srcComponent=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
    )


def _heartbeat(encoder: mavutil.mavlink.MAVLink):
    return encoder.heartbeat_encode(
        mavutil.mavlink.MAV_TYPE_FIXED_WING,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        | mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
        | mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED,
        0,
        mavutil.mavlink.MAV_STATE_STANDBY,
    )


def _mission_count(
    encoder: mavutil.mavlink.MAVLink,
    count: int,
    target_component: int = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
):
    return encoder.mission_count_encode(1, target_component, count)


def _mission_item(
    encoder: mavutil.mavlink.MAVLink,
    item: MissionItem,
    target_component: int = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
):
    return encoder.mission_item_int_encode(
        1,
        target_component,
        item.seq,
        item.frame,
        item.command,
        item.current,
        item.autocontinue,
        item.param1,
        item.param2,
        item.param3,
        item.param4,
        int(round(item.x * 1e7)),
        int(round(item.y * 1e7)),
        item.z,
    )


def _write_tlog(path: Path, records: list[tuple[float, object]]) -> None:
    encoder = _encoder()
    payload = bytearray()
    for timestamp, message in records:
        microseconds = int(timestamp * 1_000_000) & ~3
        payload.extend(struct.pack(">Q", microseconds))
        payload.extend(message.pack(encoder))
    path.write_bytes(bytes(payload))


def _complete_cycle_records(
    encoder: mavutil.mavlink.MAVLink,
    items: list[MissionItem],
    started_at: float,
) -> list[tuple[float, object]]:
    records = [(started_at, _mission_count(encoder, len(items)))]
    records.extend(
        (started_at + 0.01 * (index + 1), _mission_item(encoder, item))
        for index, item in enumerate(items)
    )
    return records


def test_scan_uses_last_complete_cycle_for_component_191(tmp_path: Path):
    now = time.time()
    encoder = _encoder()
    first = [
        MissionItem(seq=0, frame=0, command=16),
        MissionItem(seq=1, frame=3, command=16, x=29.0, y=115.0, z=20.0),
    ]
    records = _complete_cycle_records(encoder, first, now - 4.0)
    # A complete transaction for Mission Planner itself must not displace the
    # companion download selected by the audit.
    records.extend(
        [
            (now - 3.0, _mission_count(encoder, 1, target_component=190)),
            (
                now - 2.9,
                _mission_item(encoder, first[0], target_component=190),
            ),
        ]
    )
    # Nor may a newer but incomplete companion transaction replace the last
    # transaction that actually returned every sequence.
    records.extend(
        [
            (now - 2.0, _mission_count(encoder, 3)),
            (now - 1.9, _mission_item(encoder, first[0])),
            (now - 0.2, _heartbeat(encoder)),
        ]
    )
    path = tmp_path / "router.tlog"
    _write_tlog(path, records)

    cycle, heartbeat = audit_module.scan_tlog(path)

    assert cycle is not None
    assert cycle.count == 2
    assert [item.seq for item in cycle.items] == [0, 1]
    assert cycle.target_component == 191
    assert heartbeat is not None
    assert not heartbeat.armed
    assert heartbeat.mode == "MANUAL"


def test_shadow_restore_requires_zero_delay_and_matching_backup():
    expected = MissionItem(
        seq=15,
        frame=2,
        command=mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        param1=8.0,
        param2=1800.0,
        param4=0.0,
    )
    neutralized = MissionItem(
        seq=15,
        frame=0,
        command=mavutil.mavlink.MAV_CMD_NAV_DELAY,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
    )

    restored, records, errors = audit_module.restore_shadow_semantics(
        [neutralized],
        [expected],
        [expected],
        2.0,
        enabled=False,
    )
    assert not errors
    assert restored == [expected]
    assert records[0]["seq"] == 15

    bad_backup = replace(expected, param2=1700.0)
    restored, records, errors = audit_module.restore_shadow_semantics(
        [neutralized],
        [expected],
        [bad_backup],
        2.0,
        enabled=False,
    )
    assert restored == [neutralized]
    assert not records
    assert "does not match" in errors[0]

    nonzero_delay = replace(neutralized, param1=1.0)
    restored, records, errors = audit_module.restore_shadow_semantics(
        [nonzero_delay],
        [expected],
        [expected],
        2.0,
        enabled=False,
    )
    assert restored == [nonzero_delay]
    assert not records
    assert not errors


def _audit_fixture(
    tmp_path: Path,
    *,
    age_s: float = 1.0,
    position_offset_deg: float = 0.0,
    neutralize_release: bool = True,
) -> tuple[dict, float]:
    now = time.time()
    expected_path = ROOT / "config/route_takeoff_search.waypoints"
    expected = load_qgc_waypoints(expected_path)
    live = list(expected)
    live[0] = replace(live[0], current=0, x=0.0, y=0.0, z=0.0)
    live[6] = replace(live[6], frame=0)
    live[11] = replace(live[11], frame=0)
    live[13] = replace(live[13], param3=-58.0)
    if neutralize_release:
        live[15] = MissionItem(
            seq=15,
            frame=0,
            command=mavutil.mavlink.MAV_CMD_NAV_DELAY,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
        )
    live[19] = replace(live[19], param4=1.0)
    if position_offset_deg:
        live[8] = replace(live[8], x=live[8].x + position_offset_deg)

    runtime_path = tmp_path / "runtime"
    save_patch_backup(runtime_path / "mission_patch_backup.json", [expected[15]])
    expected_hash = hashlib.sha256(expected_path.read_bytes()).hexdigest()
    base = OnboardConfig()
    config = replace(
        base,
        mission=replace(
            base.mission,
            expected_file=str(expected_path),
            expected_sha256=expected_hash,
            max_position_error_m=2.0,
        ),
        runtime=RuntimeConfig(directory=str(runtime_path)),
    )

    encoder = _encoder()
    started_at = now - age_s - 0.25
    records = [(started_at - 0.1, _heartbeat(encoder))]
    records.extend(_complete_cycle_records(encoder, live, started_at))
    records.append((now - 0.05, _heartbeat(encoder)))
    tlog_path = tmp_path / "cuadc-test.tlog"
    _write_tlog(tlog_path, records)
    return audit_module.audit(config, tlog_path, 30.0, now=now), now


def test_audit_accepts_fresh_hash_pinned_shadow_readback(tmp_path: Path):
    result, _ = _audit_fixture(tmp_path)

    assert result["ok"] is True
    assert result["count"] == 20
    assert result["armed"] is False
    assert result["mode"] == "MANUAL"
    assert result["transaction"]["fresh"] is True
    assert result["hash"]["matches"] is True
    assert result["reference"]["matched"] is True
    assert result["contract"]["fallback_release_seq"] == 15
    assert result["shadow_semantic_restorations"][0]["seq"] == 15
    assert result["errors"] == []


def test_audit_uses_wall_clock_after_scanning_a_growing_tlog(
    tmp_path: Path,
    monkeypatch,
):
    fixture_root = tmp_path / "fixture"
    _result, _ = _audit_fixture(fixture_root)
    tlog_path = fixture_root / "cuadc-test.tlog"
    cycle, heartbeat = audit_module.scan_tlog(tlog_path)
    assert cycle is not None

    clock = {"now": cycle.completed_at - 10.0}

    def delayed_scan(_path: Path):
        clock["now"] = cycle.completed_at + 1.0
        return cycle, heartbeat

    monkeypatch.setattr(audit_module, "scan_tlog", delayed_scan)
    monkeypatch.setattr(audit_module.time, "time", lambda: clock["now"])

    expected_path = ROOT / "config/route_takeoff_search.waypoints"
    expected_hash = hashlib.sha256(expected_path.read_bytes()).hexdigest()
    base = OnboardConfig()
    config = replace(
        base,
        mission=replace(
            base.mission,
            expected_file=str(expected_path),
            expected_sha256=expected_hash,
            max_position_error_m=2.0,
        ),
        runtime=RuntimeConfig(directory=str(fixture_root / "runtime")),
    )

    result = audit_module.audit(config, tlog_path, 30.0)

    assert result["ok"] is True
    assert result["fresh"] is True
    assert result["age_s"] == 1.0
    assert not any("future" in error for error in result["errors"])


def test_audit_rejects_stale_or_mismatched_readback(tmp_path: Path):
    stale_result, _ = _audit_fixture(tmp_path / "stale", age_s=60.0)
    assert stale_result["ok"] is False
    assert any("stale" in error for error in stale_result["errors"])

    mismatch_result, _ = _audit_fixture(
        tmp_path / "mismatch",
        position_offset_deg=0.001,
    )
    assert mismatch_result["ok"] is False
    assert mismatch_result["reference"]["matched"] is False
    assert any("position differs" in error for error in mismatch_result["errors"])


def test_shadow_audit_rejects_physical_do_set_servo(tmp_path: Path):
    result, _ = _audit_fixture(tmp_path, neutralize_release=False)

    assert result["ok"] is False
    assert result["reference"]["matched"] is True
    assert any(
        "shadow physical release slot" in error for error in result["errors"]
    )
