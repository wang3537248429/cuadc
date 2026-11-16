#!/usr/bin/env python3
"""Run and score the 24-case CUADC physical acceptance matrix.

This file is an offline scorer / QA harness. It is the only non-simulation
consumer of ``scenario.json`` and never feeds truth back to companion, GUI,
WMS, route state or the aircraft mission.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys


APP_ROOT = Path(__file__).resolve().parents[1]
ROOT = APP_ROOT if (APP_ROOT / "src/cuadc_sim").is_dir() else APP_ROOT.parent
SCRIPT_DIR = Path(__file__).resolve().parent
LATEST_RESULT = ROOT / "runs/latest.json"
SCENARIO = ROOT / ".runtime/scenario.json"
RECOGNITION_MATCH_RADIUS_M = 10.0


def true_target(scenario: dict, task: int, group: str) -> dict:
    selected = next(item for item in scenario["groups"] if item["name"] == group)
    occupied = [well for well in selected["wells"] if well["occupied"]]
    if len(occupied) != 3:
        raise ValueError("scorer expected exactly three occupied targets")
    if task == 1:
        return max(occupied, key=lambda item: int(item["value"]))
    return sorted(occupied, key=lambda item: int(item["value"]))[1]


def score_recognition(result: dict, scenario: dict) -> dict:
    """Offline-only nearest-track check for the three printed target names."""
    task = int(result["task"])
    group_name = str(result["group"])
    selected = next(item for item in scenario["groups"] if item["name"] == group_name)
    occupied = [well for well in selected["wells"] if well["occupied"]]
    detections = list(result.get("detections") or [])
    available = set(range(len(detections)))
    correct = incorrect = blank = 0
    for well in occupied:
        candidates = []
        for index in available:
            detection = detections[index]
            if detection.get("north_m") is None or detection.get("east_m") is None:
                continue
            distance = math.hypot(
                float(detection["north_m"]) - float(well["north_m"]),
                float(detection["east_m"]) - float(well["east_m"]),
            )
            if distance <= RECOGNITION_MATCH_RADIUS_M:
                candidates.append((distance, index))
        if not candidates:
            blank += 1
            continue
        _, index = min(candidates)
        available.remove(index)
        if int(detections[index]["value"]) == int(well["value"]):
            correct += 1
        else:
            incorrect += 1
    return {
        "correct": correct,
        "incorrect": incorrect,
        "blank": blank,
        "recognition_points": 200 * correct - 100 * incorrect,
        "all_three_correct": correct == 3 and incorrect == 0 and blank == 0,
    }


def score_case(result: dict, scenario: dict, wind_speed: float) -> dict:
    impact = result.get("impact") or {}
    touchdown = result.get("first_touchdown") or {}
    stop = result.get("final_stop") or {}
    release = result.get("release") or {}
    metrics = result.get("route_metrics") or {}
    target = true_target(scenario, int(result["task"]), str(result["group"]))
    impact_north = float(impact.get("north_m", math.inf))
    impact_east = float(impact.get("east_m", math.inf))
    target_error = math.hypot(
        impact_north - float(target["north_m"]),
        impact_east - float(target["east_m"]),
    )
    group_center = 200.0 if result["group"] == "A" else -200.0
    nominal_drop_points = (
        500
        if target_error <= 3.0
        else 300
        if target_error <= 5.0
        else 50
        if abs(impact_north - group_center) <= 30.0
        and abs(impact_east) <= 30.0
        else 0
    )
    recognition = score_recognition(result, scenario)
    valid_effective_drop = bool(
        recognition["all_three_correct"]
        and nominal_drop_points > 0
    )
    drop_points = nominal_drop_points if valid_effective_drop else 0
    checks = {
        "completed": result.get("status") == "COMPLETED",
        "physical_release": release.get("physical_confirmed_at") is not None,
        "physical_impact": bool(impact),
        "within_300m": float(metrics.get("max_home_distance_m", math.inf)) <= 300.0,
        "touchdown_in_zone": touchdown.get("in_takeoff_landing_zone") is True,
        "stop_in_zone": stop.get("in_takeoff_landing_zone") is True,
        "impact_in_target_zone": (
            abs(impact_north - group_center) <= 30.0
            and abs(impact_east) <= 30.0
        ),
        "roll_within_35deg": float(metrics.get("max_roll_deg", math.inf)) <= 35.0,
        "airspeed_at_least_13mps": (
            float(metrics.get("min_airspeed_mps", -math.inf)) >= 13.0
        ),
        "nominal_time_within_120s": (
            wind_speed > 0.0
            or float(metrics.get("mission_time_s", math.inf)) <= 120.0
        ),
        "simulated_mission_within_480s": (
            float(metrics.get("mission_time_s", math.inf)) <= 480.0
        ),
    }
    return {
        "checks": checks,
        "hard_pass": all(checks.values()),
        "official_drop_points": drop_points,
        "nominal_drop_points_before_recognition_gate": nominal_drop_points,
        "recognition": recognition,
        "valid_effective_drop_proxy": valid_effective_drop,
        "precision_3m": target_error <= 3.0,
        "target_error_m": target_error,
        "truth_target": {
            "id": target["id"],
            "value": target["value"],
            "north_m": target["north_m"],
            "east_m": target["east_m"],
        },
    }


def stream_case(command: list[str], log_path: Path) -> int:
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
        return process.wait()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = args.output_dir or ROOT / "runs/acceptance-matrix" / stamp
    output_dir.mkdir(parents=True, exist_ok=True)

    cases = list(
        itertools.product((1, 2), ("A", "B"), (1, 17, 99), (0.0, 5.0))
    )
    reports: list[dict] = []
    for index, (task, group, seed, wind) in enumerate(cases, start=1):
        case_id = f"t{task}-{group.lower()}-s{seed}-w{int(wind)}"
        print(f"\n[MATRIX] {index:02d}/24 {case_id}", flush=True)
        command = [
            str(ROOT / ".venv/bin/python"),
            str(SCRIPT_DIR / "e2e_smoke.py"),
            "--full",
            "--timeout",
            "300",
            "--task",
            str(task),
            "--group",
            group,
            "--seed",
            str(seed),
            "--wind-speed",
            str(wind),
            "--wind-direction",
            "90",
        ]
        code = stream_case(command, output_dir / f"{case_id}.log")
        if code != 0:
            report = {"case_id": case_id, "process_exit": code, "hard_pass": False}
        else:
            result = json.loads(LATEST_RESULT.read_text(encoding="utf-8"))
            scenario = json.loads(SCENARIO.read_text(encoding="utf-8"))
            score = score_case(result, scenario, wind)
            report = {
                "case_id": case_id,
                "task": task,
                "group": group,
                "seed": seed,
                "wind_speed_mps": wind,
                "result": result,
                "score": score,
                "hard_pass": score["hard_pass"],
            }
        (output_dir / f"{case_id}.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        reports.append(report)
        print(
            f"[MATRIX] {case_id}: "
            f"hard={'PASS' if report['hard_pass'] else 'FAIL'}",
            flush=True,
        )

    precision = sum(
        bool(report.get("score", {}).get("precision_3m")) for report in reports
    )
    hard = sum(bool(report["hard_pass"]) for report in reports)
    summary = {
        "schema": "cuadc-acceptance-matrix.v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "cases": len(reports),
        "hard_passes": hard,
        "precision_3m_passes": precision,
        "acceptance_pass": hard == 24 and precision >= 23,
        "reports": [
            {
                "case_id": report["case_id"],
                "hard_pass": report["hard_pass"],
                "precision_3m": report.get("score", {}).get("precision_3m", False),
                "target_error_m": report.get("score", {}).get("target_error_m"),
            }
            for report in reports
        ],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0 if summary["acceptance_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
