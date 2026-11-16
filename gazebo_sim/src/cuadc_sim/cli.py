from __future__ import annotations

import argparse


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="cuadc-sim")
    subparsers = parser.add_subparsers(dest="command", required=True)

    scenario_parser = subparsers.add_parser("generate-world", help="generate deterministic Gazebo world")
    scenario_parser.add_argument("--task", type=int, choices=(1, 2), default=1)
    scenario_parser.add_argument("--group", choices=("A", "B"), default="A")
    scenario_parser.add_argument("--seed", type=int, default=1)
    scenario_parser.add_argument("--wind-speed", type=float, default=0.0)
    scenario_parser.add_argument("--wind-direction", type=float, default=0.0)

    companion_parser = subparsers.add_parser("companion", help="run MAVLink mission executive")
    companion_parser.add_argument("--endpoint", default="udpin:0.0.0.0:14551")
    companion_parser.add_argument("--dry-run", action="store_true")

    vision_parser = subparsers.add_parser("vision", help="run Gazebo-camera YOLO and result window")
    vision_parser.add_argument("--confidence", type=float, default=0.20)
    vision_parser.add_argument("--headless", action="store_true")

    subparsers.add_parser("wms", help="serve the local Mission Planner WMS layer")

    args = parser.parse_args(argv)
    if args.command == "generate-world":
        from .scenario import main as scenario_main

        scenario_args = [
            "--task",
            str(args.task),
            "--group",
            args.group,
            "--seed",
            str(args.seed),
            "--wind-speed",
            str(args.wind_speed),
            "--wind-direction",
            str(args.wind_direction),
        ]
        return scenario_main(scenario_args)
    if args.command == "companion":
        from .companion import main as companion_main

        return companion_main(args.endpoint, args.dry_run)
    if args.command == "vision":
        from .vision_service import main as vision_main

        return vision_main(args.confidence, not args.headless)
    if args.command == "wms":
        from .wms import main as wms_main

        return wms_main()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
