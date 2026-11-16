#!/usr/bin/env python3
"""Safely configure or restore Mission Planner's local CUADC WMS settings."""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET


DEFAULT_CONFIG = Path.home() / ".local/share/Mission Planner/config.xml"
WMS_URL = "http://127.0.0.1:8765/wms?"
WMS_LAYER = "cuadc_field"


def mission_planner_running() -> bool:
    own_pid = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").lower()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if b"missionplanner.exe" in command:
            return True
    return False


def _backup(config_path: Path, label: str = "before-wms") -> Path:
    backup_dir = config_path.parent / "cuadc-wms-backups"
    backup_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    backup = backup_dir / f"config.{label}.{timestamp}.xml"
    shutil.copy2(config_path, backup)
    return backup


def _set_text(root: ET.Element, name: str, value: str) -> None:
    element = root.find(name)
    if element is None:
        element = ET.SubElement(root, name)
    element.text = value


def _write_tree_atomic(tree: ET.ElementTree, config_path: Path) -> None:
    ET.indent(tree, space="  ")
    payload = ET.tostring(tree.getroot(), encoding="utf-8", xml_declaration=True)
    temporary = config_path.with_suffix(".xml.cuadc.tmp")
    temporary.write_bytes(b"\xef\xbb\xbf" + payload)
    ET.parse(temporary)
    os.replace(temporary, config_path)


def configure(config_path: Path) -> Path:
    tree = ET.parse(config_path)
    backup = _backup(config_path)
    root = tree.getroot()
    _set_text(root, "WMSserver", WMS_URL)
    _set_text(root, "WMSLayer", WMS_LAYER)
    _set_text(root, "CustomWMSURL", WMS_URL)
    _set_text(root, "MapType", "WMS Custom")
    _write_tree_atomic(tree, config_path)
    return backup


def restore(config_path: Path, requested: str) -> tuple[Path, Path]:
    backup_dir = config_path.parent / "cuadc-wms-backups"
    if requested == "latest":
        candidates = sorted(backup_dir.glob("config.before-wms.*.xml"))
        if not candidates:
            raise FileNotFoundError(f"no CUADC WMS backup in {backup_dir}")
        source = candidates[-1]
    else:
        source = Path(requested).expanduser().resolve()
        if not source.is_file():
            raise FileNotFoundError(source)
    ET.parse(source)
    safety_backup = _backup(config_path, label="before-restore")
    temporary = config_path.with_suffix(".xml.cuadc.tmp")
    shutil.copy2(source, temporary)
    os.replace(temporary, config_path)
    return source, safety_backup


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--restore",
        nargs="?",
        const="latest",
        help="restore the latest backup, or the supplied backup path",
    )
    arguments = parser.parse_args(argv)
    config_path = arguments.config.expanduser().resolve()
    if mission_planner_running():
        print("Mission Planner is running; close it before changing config.xml", file=sys.stderr)
        return 2
    if not config_path.is_file():
        print(f"Mission Planner config not found: {config_path}", file=sys.stderr)
        return 2
    if arguments.restore:
        source, safety = restore(config_path, arguments.restore)
        print(f"Restored Mission Planner config from: {source}")
        print(f"Pre-restore safety backup: {safety}")
    else:
        backup = configure(config_path)
        print(f"Mission Planner backup: {backup}")
        print(f"Configured WMS URL: {WMS_URL}")
        print(f"Configured WMS layer: {WMS_LAYER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
