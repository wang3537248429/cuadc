#!/usr/bin/env python3
"""从明确的文件清单构建无历史、无个人元数据的公开源码包。"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import os
from pathlib import Path
import re
import shutil
import tarfile
import tempfile

try:
    from .privacy_audit import audit
    from .release_policy import excluded, public_files
except ImportError:
    from privacy_audit import audit
    from release_policy import excluded, public_files


ROOT = Path(__file__).resolve().parents[1]
PROJECT_DIRS = {
    "onboard": ROOT / "onboard",
    "simulation": ROOT / "gazebo_sim",
    "ground-station": ROOT / "ground_station",
    "public": ROOT,
}
PACKAGE_KINDS = tuple(PROJECT_DIRS)
OUTPUT_DIRECTORIES = {
    "onboard": "onboard", "simulation": "gazebo_sim",
    "ground-station": "ground_station", "public": "public",
}


def project_version(kind: str) -> str:
    source = PROJECT_DIRS["simulation"] if kind == "public" else PROJECT_DIRS[kind]
    content = (source / "pyproject.toml").read_text(encoding="utf-8")
    match = re.search(r'^version\s*=\s*"([0-9A-Za-z.+-]+)"\s*$', content, re.MULTILINE)
    if match is None:
        raise ValueError(f"{kind}: pyproject.toml 缺少可用版本")
    return match.group(1)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def ignored(_directory: str, names: list[str]) -> set[str]:
    """兼容本地调用；敏感文件由审计器报错，不会静默放过。"""
    return {name for name in names if excluded(Path(name))}


def write_package_checksums(package_root: Path) -> None:
    records = [
        f"{sha256(package_root / relative)}  {relative.as_posix()}"
        for relative in public_files(package_root)
        if relative.name != "PACKAGE_SHA256SUMS"
    ]
    (package_root / "PACKAGE_SHA256SUMS").write_text("\n".join(records) + "\n", encoding="utf-8")


def validate_staging(kind: str, package_root: Path) -> None:
    required = ["README.md", "LICENSE"]
    if kind == "public":
        required += [
            "gazebo_sim/pyproject.toml", "onboard/pyproject.toml",
            "ground_station/pyproject.toml", "scripts/privacy_audit.py",
            ".github/workflows/ci.yml", "SECURITY.md",
        ]
    else:
        required += ["pyproject.toml"]
    missing = [item for item in required if not (package_root / item).is_file()]
    if missing:
        raise RuntimeError(f"{kind}: 源码包缺少 {missing}")
    if not (package_root / "LICENSE").read_text(encoding="utf-8").startswith("MIT License"):
        raise RuntimeError(f"{kind}: 缺少 MIT 许可")
    _, findings = audit(package_root)
    if findings:
        raise RuntimeError("发布被隐私审计阻止：\n" + "\n".join(findings))


def export_directory(kind: str, destination: Path, *, link_dataset_files: bool = False) -> Path:
    """Export a reviewable source tree without Git history or runtime files.

    The optional hardlink mode is only for immutable, versioned dataset files
    on the same filesystem. All editable code/configuration is copied. The
    manifest detects any later change to a shared dataset image.
    """
    source = PROJECT_DIRS[kind]
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(f"输出目录已存在：{destination}")
    if destination.is_relative_to(ROOT) and not excluded(destination.relative_to(ROOT)):
        raise ValueError("工作区内的发布输出必须放在 dist 等被排除的目录")
    # ``audit`` also returns local model archives so their pickle metadata can
    # be inspected before a release.  Those archives are deliberately outside
    # the source-package allowlist: uncleared models must stay local. Rebuild the copy list from
    # ``public_files`` after the audit so a local, ignored weight can never be
    # included accidentally.
    _, findings = audit(source, workspace=kind == "public")
    files = public_files(source, workspace=kind == "public")
    if findings:
        raise RuntimeError("发布被隐私审计阻止：\n" + "\n".join(findings))
    destination.mkdir(parents=True)
    try:
        for relative in files:
            target = destination / relative
            original = source / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            linked = False
            if link_dataset_files and "datasets" in relative.parts:
                try:
                    os.link(original, target)
                    linked = True
                except OSError:
                    pass
            if not linked:
                shutil.copyfile(original, target)
                target.chmod(0o755 if original.stat().st_mode & 0o111 else 0o644)
        validate_staging(kind, destination)
        write_package_checksums(destination)
    except BaseException:
        shutil.rmtree(destination)
        raise
    return destination


def build(kind: str, output_dir: Path, include_wheelhouse: bool = False) -> Path:
    if include_wheelhouse:
        raise ValueError("公开源码包不包含本地 wheelhouse")
    source = PROJECT_DIRS[kind]
    # Keep auditing ignored weight archives for metadata leaks, but export only
    # the public allowlist.  ``audit`` intentionally has a larger inspection
    # set than ``public_files`` for this purpose.
    _, findings = audit(source, workspace=kind == "public")
    files = public_files(source, workspace=kind == "public")
    if findings:
        raise RuntimeError("发布被隐私审计阻止：\n" + "\n".join(findings))
    prefixes = {
        "public": "cuadc-source", "simulation": "cuadc-gazebo-simulation",
        "onboard": "cuadc-onboard-rpi4", "ground-station": "cuadc-ground-station",
    }
    basename = f"{prefixes[kind]}-{project_version(kind)}"
    category_dir = output_dir.resolve() / OUTPUT_DIRECTORIES[kind]
    # Do not let an output directory become part of the exported input tree.
    if category_dir.is_relative_to(ROOT) and not excluded(category_dir.relative_to(ROOT)):
        raise ValueError("工作区内的发布输出必须放在 dist 等被排除的目录")
    category_dir.mkdir(parents=True, exist_ok=True)
    archive = category_dir / f"{basename}.tar.gz"
    temporary_parent=Path(os.environ.get('CUADC_PACKAGE_TMPDIR',category_dir))
    temporary_parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".cuadc-stage-", dir=temporary_parent) as temporary:
        package_root = Path(temporary) / basename
        for relative in files:
            destination = package_root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source / relative, destination)
            destination.chmod(0o755 if (source / relative).stat().st_mode & 0o111 else 0o644)
        validate_staging(kind, package_root)
        write_package_checksums(package_root)
        pending = Path(temporary) / "source.tar.gz"
        with pending.open("wb") as output:
            with gzip.GzipFile(filename="", mode="wb", fileobj=output, mtime=0, compresslevel=1) as compressed:
                with tarfile.open(fileobj=compressed, mode="w") as stream:
                    for relative in public_files(package_root):
                        path = package_root / relative
                        info = stream.gettarinfo(str(path), arcname=f"{basename}/{relative.as_posix()}")
                        info.uid = info.gid = 0
                        info.uname = info.gname = ""
                        info.mtime = 0
                        info.pax_headers = {}
                        with path.open("rb") as content:
                            stream.addfile(info, content)
        destination=archive.with_suffix(archive.suffix+'.tmp')
        shutil.copyfile(pending,destination)
        destination.replace(archive)
    return archive


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=PACKAGE_KINDS, nargs="?", default="public")
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    parser.add_argument("--directory", type=Path, help="直接导出干净目录；不生成压缩包")
    parser.add_argument("--link-dataset-files", action="store_true", help="目录导出时硬链接不可变数据集以节约空间；编辑前须复制")
    args = parser.parse_args(argv)
    if args.directory:
        print(export_directory(args.kind, args.directory, link_dataset_files=args.link_dataset_files))
        return 0
    if args.link_dataset_files:
        parser.error("--link-dataset-files 只用于 --directory")
    archive = build(args.kind, args.output)
    checksum_file = archive.parent / "SHA256SUMS"
    checksum_file.write_text(f"{sha256(archive)}  {archive.name}\n", encoding="utf-8")
    print(f"{archive}\n{checksum_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
