#!/usr/bin/env python3
"""将 QGC 航线迁移到公开 SITL 原点，保留局部几何与相对高度。"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

SITL_LAT = -35.363261
SITL_LON = 149.165230
SITL_ALT = 46.0


def anonymize(body: str, origin_lat: float, origin_lon: float, origin_alt: float) -> str:
    lines = body.splitlines()
    if not lines or lines[0].strip() != "QGC WPL 110":
        raise ValueError("需要 QGC WPL 110 航线")
    result = ["QGC WPL 110"]
    for line in lines[1:]:
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 12:
            raise ValueError("航点必须包含 12 个字段")
        lat, lon = float(fields[8]), float(fields[9])
        if int(fields[2]) in (0, 3, 10) and (lat != 0 or lon != 0):
            fields[8] = f"{SITL_LAT + lat - origin_lat:.10f}"
            fields[9] = f"{SITL_LON + (lon - origin_lon) * math.cos(math.radians(origin_lat)) / math.cos(math.radians(SITL_LAT)):.10f}"
            if int(fields[2]) == 0:
                fields[10] = f"{float(fields[10]) - origin_alt + SITL_ALT:.6f}"
        result.append("\t".join(fields))
    return "\n".join(result) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--origin-lat", type=float, required=True)
    parser.add_argument("--origin-lon", type=float, required=True)
    parser.add_argument("--origin-alt", type=float, required=True)
    args = parser.parse_args()
    if args.destination.exists():
        parser.error("输出已存在；请指定新文件，避免覆盖原始航线")
    body = anonymize(args.source.read_text(), args.origin_lat, args.origin_lon, args.origin_alt)
    args.destination.write_text(body, encoding="utf-8")


if __name__ == "__main__":
    main()
