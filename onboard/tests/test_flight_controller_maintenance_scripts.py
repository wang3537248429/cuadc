from __future__ import annotations

from dataclasses import replace
import hashlib
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import collect_flight_controller_evidence as evidence  # noqa: E402
import download_flight_controller_crashdump as downloader  # noqa: E402
import install_flight_controller_mission as installer  # noqa: E402
import restore_crashdump_arming_guard as guard  # noqa: E402
from cuadc_onboard.mission import (  # noqa: E402
    MissionItem,
    compare_mission_reference,
    load_qgc_waypoints,
    validate_mission,
)


ROOT = Path(__file__).resolve().parents[1]


def test_active_only_installer_preserves_the_release_command_and_rejects_disabled():
    reference = load_qgc_waypoints(
        ROOT / "config/route_takeoff_search.waypoints"
    )
    contract, errors = validate_mission(reference, payload_servo=8, min_release_pwm=1800)
    assert contract is not None and not errors

    with pytest.raises(installer.MissionInstallError, match="active-only"):
        installer.physical_install_mission(reference, contract, False)
    with pytest.raises(installer.MissionInstallError, match="active-only"):
        installer.semantic_install_readback(reference, reference, contract, False)

    active = installer.physical_install_mission(reference, contract, True)
    assert active[contract.fallback_release_seq].command == 183
    assert len(active) == len(reference) == 20
    assert sum(item.command == 183 for item in active) == 1

    semantic = installer.semantic_install_readback(
        active,
        reference,
        contract,
        True,
    )
    semantic_errors, report = compare_mission_reference(semantic, reference)
    assert not semantic_errors
    assert report["matched"] is True


def test_installer_rejects_an_extra_servo_command_outside_the_contract():
    reference = load_qgc_waypoints(
        ROOT / "config/route_takeoff_search.waypoints"
    )
    contract, errors = validate_mission(reference, payload_servo=8, min_release_pwm=1800)
    assert contract is not None and not errors
    reference[1] = replace(reference[1], command=183, param1=8.0, param2=1800.0)

    with pytest.raises(installer.MissionInstallError, match="exactly one DO_SET_SERVO"):
        installer.physical_install_mission(reference, contract, True)


def test_active_installer_readback_does_not_reconstruct_a_neutralized_slot():
    reference = load_qgc_waypoints(
        ROOT / "config/route_takeoff_search.waypoints"
    )
    contract, errors = validate_mission(reference, payload_servo=8, min_release_pwm=1800)
    assert contract is not None and not errors
    release_seq = contract.fallback_release_seq
    readback = list(reference)
    readback[release_seq] = replace(readback[release_seq], command=93, param1=0.0)

    semantic = installer.semantic_install_readback(
        readback, reference, contract, True
    )
    errors, _report = compare_mission_reference(semantic, reference)
    assert errors


def test_guard_rejects_invalid_or_unconfirmed_crash_dump(tmp_path: Path):
    dump = tmp_path / "crash_dump.bin"
    dump.write_bytes(b"not-a-crash-dump")

    with pytest.raises(guard.GuardRestoreError, match="lowercase SHA-256"):
        guard.verify_preserved_dump(dump, "A" * 64)
    with pytest.raises(guard.GuardRestoreError, match="magic"):
        guard.verify_preserved_dump(
            dump,
            hashlib.sha256(dump.read_bytes()).hexdigest(),
        )

    payload = b"cC" + b"preserved-evidence"
    dump.write_bytes(payload)
    expected = hashlib.sha256(payload).hexdigest()
    assert guard.verify_preserved_dump(dump, expected) == (len(payload), expected)
    with pytest.raises(guard.GuardRestoreError, match="mismatch"):
        guard.verify_preserved_dump(dump, "0" * 64)


def test_all_maintenance_tools_share_one_exclusive_router_lock(tmp_path: Path):
    downloader_lock = downloader.acquire_diagnostic_lock(tmp_path)
    try:
        with pytest.raises(guard.GuardRestoreError, match="another maintenance"):
            guard.acquire_lock(tmp_path)
    finally:
        downloader_lock.close()

    guard_lock = guard.acquire_lock(tmp_path)
    try:
        with pytest.raises(
            downloader.CrashDumpDownloadError,
            match="another diagnostic MAVFTP",
        ):
            downloader.acquire_diagnostic_lock(tmp_path)
    finally:
        guard_lock.close()


def test_evidence_serializers_are_deterministic_and_reject_bad_mission():
    assert evidence.parameter_text({"Z_PARAM": 2.0, "A_PARAM": 1.25}) == (
        "A_PARAM          1.25\nZ_PARAM          2\n"
    )

    item = MissionItem(
        seq=0,
        current=1,
        frame=3,
        command=16,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        x=30.0,
        y=120.0,
        z=50.0,
        autocontinue=1,
    )
    text = evidence.mission_text([item])
    assert text.startswith("QGC WPL 110\n0\t1\t3\t16\t")

    with pytest.raises(evidence.EvidenceCollectionError, match="contiguous"):
        evidence.mission_text([item.__class__(**{**item.__dict__, "seq": 1})])
    with pytest.raises(evidence.EvidenceCollectionError, match="non-finite"):
        evidence.mission_text(
            [item.__class__(**{**item.__dict__, "param1": float("nan")})]
        )


def test_guard_rechecks_disarmed_immediately_before_parameter_write(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    events: list[str] = []
    receipts: list[dict] = []

    class FakeClient:
        target_system = 1
        target_component = 1

        def __init__(self, *_args, **_kwargs):
            self.connection = SimpleNamespace(close=lambda: events.append("close"))

        def connect(self, **_kwargs):
            events.append("connect")

        def read_parameter(self, name: str, **_kwargs) -> float:
            events.append(f"read:{name}")
            if name == "ARMING_CHECK":
                return 1.0
            return 1.0 if events.count("read:ARMING_CRSDP_IGN") == 1 else 0.0

        def set_parameters(self, values: dict[str, float]):
            assert values == {"ARMING_CRSDP_IGN": 0.0}
            events.append("set")

    heartbeat_number = 0

    def fake_wait_disarmed(_client, timeout=6.0):
        nonlocal heartbeat_number
        heartbeat_number += 1
        events.append(f"heartbeat:{heartbeat_number}")
        return {"armed": False, "system_status": 3}

    monkeypatch.setattr(
        guard,
        "parse_args",
        lambda: SimpleNamespace(
            config=tmp_path / "config.yaml",
            crash_dump=tmp_path / "crash.bin",
            confirm_crashdump_sha256="0" * 64,
        ),
    )
    monkeypatch.setattr(guard, "verify_preserved_dump", lambda *_args: (32, "0" * 64))
    monkeypatch.setattr(guard, "require_exclusive_router_access", lambda: None)
    monkeypatch.setattr(
        guard,
        "acquire_lock",
        lambda _path: SimpleNamespace(close=lambda: None),
    )
    monkeypatch.setattr(
        guard,
        "load_config",
        lambda _path: SimpleNamespace(
            flight_controller=SimpleNamespace(
                endpoint="memory:test",
                baud=115200,
                heartbeat_timeout_s=1.0,
            )
        ),
    )
    monkeypatch.setattr(guard, "MavlinkClient", FakeClient)
    monkeypatch.setattr(guard, "wait_disarmed", fake_wait_disarmed)
    monkeypatch.setattr(guard, "ROOT", tmp_path)
    monkeypatch.setattr(
        guard,
        "atomic_json",
        lambda _path, payload: receipts.append(payload),
    )

    assert guard.main() == 0
    assert events == [
        "connect",
        "heartbeat:1",
        "read:ARMING_CHECK",
        "read:ARMING_CRSDP_IGN",
        "heartbeat:2",
        "set",
        "read:ARMING_CRSDP_IGN",
        "heartbeat:3",
        "close",
    ]
    assert receipts[0]["heartbeat_before_mutation"]["armed"] is False


def test_evidence_rejects_too_short_ftp_timeout_before_connecting(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    monkeypatch.setattr(
        evidence,
        "parse_args",
        lambda: SimpleNamespace(
            timeout=5.0,
            config=tmp_path / "config.yaml",
            output_root=tmp_path,
        ),
    )
    with pytest.raises(evidence.EvidenceCollectionError, match="greater than 5"):
        evidence.main()
