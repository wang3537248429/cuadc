#!/usr/bin/env python3
"""离线验证机载编码 → MAVLink TLOG → 地面站中文解析，不连接设备。"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import struct
import sys
import time
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
for source in (ROOT / "onboard/src", ROOT / "ground_station"):
    sys.path.insert(0, str(source))

from pymavlink import mavutil
from cuadc_onboard.config import RuntimeConfig, load_config
from cuadc_onboard.mavlink_io import MavlinkClient
from cuadc_onboard.runtime import OnboardRuntime
from cuadc_ground_station.tlog_reader import scan_tlog


class TlogWriter:
    def __init__(self, stream):
        self.stream = stream
        self.packets = 0

    def write(self, packet: bytes) -> None:
        self.stream.write(struct.pack(">Q", int(time.time() * 1_000_000)))
        self.stream.write(packet)
        self.packets += 1


def run(output: Path, simulation_result: Path | None = None) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    simulation = json.loads(simulation_result.read_text()) if simulation_result else {}
    if simulation_result and not simulation.get("selected_target"):
        raise ValueError("仿真没有最终选靶记录，不能据此声称回传选靶成功")
    selected = simulation.get("selected_target") or {"value": 12, "north_m": 10.0, "east_m": 20.0}
    base = load_config(ROOT / "onboard/config/onboard.yaml")
    config = replace(
        base,
        vision=replace(base.vision, task=int(simulation.get("task", 1))),
        runtime=RuntimeConfig(directory=str(output / "runtime")),
    )
    runtime = OnboardRuntime(config)
    runtime.contract = SimpleNamespace(
        scan_start_seq=3, scan_end_seq=8, fallback_release_seq=12, land_seq=20,
    )
    payload = {
        "status": "READY", "mission_mode": "DYNAMIC_READY",
        "selected_target": selected,
        "vision": {"detections": [{**selected, "confidence": 0.95}]},
    }
    tlog = output / "synthetic.tlog"
    with tlog.open("wb") as stream:
        sink = TlogWriter(stream)
        encoder = mavutil.mavlink.MAVLink(sink, srcSystem=1, srcComponent=191)
        client = MavlinkClient()
        client.connection = SimpleNamespace(mav=encoder)
        runtime.client = client
        runtime._publish_mavlink_report(payload, force=True)
        if runtime.state["mavlink_report"].get("error"):
            raise RuntimeError(runtime.state["mavlink_report"]["error"])
    snapshot = scan_tlog(tlog)
    if snapshot.selected_value != int(selected["value"]):
        raise RuntimeError("机载选靶值与地面站解析结果不一致")
    report = snapshot.grouped_zh()
    # Do not embed the local source log path in a shareable summary.
    report["数据源"]["Mission Planner日志"] = "合成 TLOG"
    result = {
        "passed": True, "packets": sink.packets,
        "scope": "机载软件编码与地面站解码（离线，无硬件连接）",
        "input_source": "仿真最终选靶" if simulation_result else "合成测试靶标",
        "selected_value": snapshot.selected_value, "report": report,
    }
    (output / "report.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "dist/validation/reports")
    parser.add_argument("--simulation-result", type=Path)
    args = parser.parse_args()
    result = run(args.output, args.simulation_result)
    print(f"[回传验收] {result['packets']} 个 MAVLink 数据包，中文解析一致")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
