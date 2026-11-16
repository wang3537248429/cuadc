#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time

from pymavlink import mavutil


STATUS_NAMES = {
    0: "STARTING",
    1: "WAITING_FOR_NAVIGATION",
    2: "READY",
    3: "VISION_FAILED",
    4: "MISSION_INVALID",
    5: "PAYLOAD_SERVO_INVALID",
    6: "FATAL",
    99: "UNKNOWN",
}


def decoded(value) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace").rstrip("\x00")
    return str(value).rstrip("\x00")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Receive Raspberry Pi CUADC status from a MAVLink telemetry link"
    )
    parser.add_argument(
        "--endpoint",
        default="udpin:0.0.0.0:14550",
        help="pymavlink endpoint supplied by the ground telemetry relay",
    )
    parser.add_argument(
        "--component",
        type=int,
        default=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
    )
    parser.add_argument("--print-hz", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.print_hz <= 0.0:
        parser.error("--print-hz must be positive")

    connection = mavutil.mavlink_connection(
        args.endpoint,
        source_system=255,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        autoreconnect=True,
    )
    values: dict[str, int | float | str | bool] = {}
    next_print = time.monotonic()
    changed = False
    try:
        while True:
            message = connection.recv_match(
                type=[
                    "HEARTBEAT",
                    "STATUSTEXT",
                    "NAMED_VALUE_INT",
                    "NAMED_VALUE_FLOAT",
                ],
                blocking=True,
                timeout=0.2,
            )
            if message is not None and int(message.get_srcComponent()) == args.component:
                message_type = message.get_type()
                if message_type == "HEARTBEAT":
                    values["component_alive"] = True
                    values["status_code"] = int(message.custom_mode)
                    values["status_name"] = STATUS_NAMES.get(
                        int(message.custom_mode), "UNKNOWN"
                    )
                    values["last_heartbeat_at"] = time.time()
                    changed = True
                elif message_type == "STATUSTEXT":
                    print(
                        json.dumps(
                            {
                                "type": "event",
                                "received_at": time.time(),
                                "severity": int(message.severity),
                                "text": decoded(message.text),
                            },
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
                else:
                    name = decoded(message.name)
                    values[name] = (
                        int(message.value)
                        if message_type == "NAMED_VALUE_INT"
                        else float(message.value)
                    )
                    values["last_report_at"] = time.time()
                    if name == "CU_STATUS":
                        values["status_name"] = STATUS_NAMES.get(
                            int(message.value), "UNKNOWN"
                        )
                    changed = True
            now = time.monotonic()
            if changed and now >= next_print:
                print(
                    json.dumps(
                        {"type": "state", **values},
                        ensure_ascii=False,
                        sort_keys=True,
                    ),
                    flush=True,
                )
                changed = False
                next_print = now + 1.0 / args.print_hz
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
