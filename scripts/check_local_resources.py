#!/usr/bin/env python3
"""校验本地可选素材；公开源码检查允许资源整组缺席，运行检查不允许。"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def check_manifest(project: Path, manifest: Path, *, required: bool = False) -> list[str]:
    if not manifest.is_file():
        return [f"缺少清单：{manifest.relative_to(project)}"] if required else []
    entries = []
    for row in manifest.read_text().splitlines():
        if not row.strip():
            continue
        fields = row.split(maxsplit=1)
        if len(fields) != 2 or not re.fullmatch(r"[0-9a-fA-F]{64}", fields[0]):
            return ["资源清单格式错误"]
        digest, relative = fields
        relative = Path(relative.strip().removeprefix("*"))
        if relative.is_absolute() or ".." in relative.parts:
            return ["资源清单包含不安全的路径"]
        path = project / relative
        if path.is_symlink() or not path.resolve().is_relative_to(project.resolve()):
            return ["资源清单包含不安全的路径"]
        entries.append((digest, path))
    if not entries:
        return ["资源清单为空"]
    if not required and not any(path.exists() for _, path in entries):
        return []
    errors = []
    for digest, path in entries:
        if not path.is_file():
            errors.append(f"缺少资源：{path.relative_to(project)}")
        elif hashlib.sha256(path.read_bytes()).hexdigest() != digest.lower():
            errors.append(f"资源校验失败：{path.relative_to(project)}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", choices=("gazebo_sim", "onboard", "all"), default="all")
    parser.add_argument("--require", action="store_true", help="运行前要求本地模型和靶标全部存在")
    args = parser.parse_args()
    errors = []
    modules = ("gazebo_sim", "onboard") if args.project == "all" else (args.project,)
    for module in modules:
        project = ROOT / module
        # Runtime .pt files are sufficient; the training-only pretrained
        # archive and compressed copies are not execution dependencies.
        errors.extend(check_manifest(project, project / "models/vision/MODEL_SHA256SUMS", required=args.require))
        if module == "gazebo_sim":
            errors.extend(check_manifest(project, project / "assets/targets/REFERENCE_SHA256SUMS", required=args.require))
    for error in errors:
        print(f"[资源] {error}")
    if errors:
        print("[资源] 请配置自行取得合法使用权的素材和模型；见 docs/资源与许可.md。")
    else:
        print("[资源] 校验通过；未随源码提供的资源不计为识别或飞行验收。")
    return int(bool(errors))


if __name__ == "__main__":
    raise SystemExit(main())
