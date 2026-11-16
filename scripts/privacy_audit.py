#!/usr/bin/env python3
"""扫描公开内容，只报告文件、位置和类别，绝不回显敏感值。"""
from __future__ import annotations

import argparse
import gzip
import io
import json
import os
from pathlib import Path
import pickletools
import re
import struct
import sys
import zipfile

try:
    from .release_policy import PUBLIC_ROOTS, excluded, public_files
except ImportError:
    from release_policy import PUBLIC_ROOTS, excluded, public_files


ROOT = Path(__file__).resolve().parents[1]
PATTERNS = {
    "个人目录": re.compile(r"/(?:home|media)/[A-Za-z0-9._-]+|(?<![A-Za-z]:)/Users/(?!pilot\b)[A-Za-z0-9._-]+|[A-Z]:[\\/]Users[\\/](?!pilot\b)[^\s\"']+"),
    "访问凭据": re.compile(
        r"(?:github" + r"_pat_|gh[pousr]_)[A-Za-z0-9_]{20,}|"
        r"AKIA[0-9A-Z]{16}|sk-" + r"(?:proj-)?[A-Za-z0-9_-]{32,}|"
        r"BEGIN (?:RSA |OPENSSH |EC )?PRIVATE" + r" KEY"
    ),
    "硬编码密码": re.compile(
        r"(?i)(?:password|passwd|api_key|access_token)\s*[:=]\s*[\"']"
        r"(?!example[\"']|placeholder[\"']|[\"'])[^\"'\s]{4,}[\"']"
    ),
    "设备序列号": re.compile(r"usb-[^/\s\"'<>]+_[A-Za-z0-9]{8,}(?:-if\d+|-video-index\d+)"),
    "私人社交标识": re.compile(r"wxid_" + r"[A-Za-z0-9]{6,}|xwe" + "chat", re.I),
    "手机号或身份证": re.compile(r"(?<![\w.])(?:1[3-9]\d{9}|[1-9]\d{16}[0-9Xx])(?![\w.])"),
}
# 航线与参数文件名会随 onboard/config 发布，不能带场次、验收状态或战术语义。
OPERATIONAL_FILENAME = re.compile(
    r"侦察|影子|验收|正式版|全流程|场次|预赛|决赛|彩排|对抗|实机|实飞|[AB]区"
)

def scan_text(body: str, label: str) -> list[str]:
    findings = [
        f"{label}:{line_number}: {category}"
        for line_number, line in enumerate(body.splitlines(), 1)
        for category, pattern in PATTERNS.items()
        if pattern.search(line)
    ]
    for line_number, line in enumerate(body.splitlines(), 1):
        for match in re.finditer(r"[\w.+-]+@([\w.-]+\.[A-Za-z]{2,})", line):
            # Documentation may contain SSH examples and reserved email domains.
            if match[0].startswith("root@") or match[1].endswith((".invalid", ".example")):
                continue
            findings.append(f"{label}:{line_number}: 邮箱地址需人工脱敏")
    return findings


def scan_waypoints(body: str, label: str) -> list[str]:
    findings = []
    for number, line in enumerate(body.splitlines()[1:], 2):
        if not line.strip():
            continue
        try:
            row = line.split()
            lat, lon = float(row[8]), float(row[9])
            # Synthetic examples and ArduPilot's public Canberra SITL origin.
            allowed = (abs(lat) < 5 and abs(lon) < 5) or (
                abs(lat + 35.363261) < 0.02 and abs(lon - 149.165230) < 0.02
            )
            if len(row) != 12 or not allowed:
                findings.append(f"{label}:{number}: 未脱敏或无效的航线坐标")
        except (ValueError, IndexError):
            findings.append(f"{label}:{number}: 无法审计的航线格式")
    return findings


def scan_json_coordinates(value: object, label: str) -> list[str]:
    """Check nested geographic fields, including arena and saved telemetry."""
    findings: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            field = str(key).lower()
            latitude = field in {"lat", "latitude"} or field.endswith(("lat_deg", "latitude_deg"))
            longitude = field in {"lon", "lng", "longitude"} or field.endswith(("lon_deg", "longitude_deg"))
            if (latitude or longitude) and isinstance(child, (int, float, str)) and not isinstance(child, bool):
                try:
                    coordinate = float(child)
                except ValueError:
                    findings.append(f"{label}:json:{key}: 无法审计的坐标字段")
                    continue
                center = -35.363261 if latitude else 149.165230
                if not (abs(coordinate) < 5 or abs(coordinate - center) < 0.02):
                    findings.append(f"{label}:json:{key}: 非公开示例坐标")
            findings.extend(scan_json_coordinates(child, label))
    elif isinstance(value, list):
        for child in value:
            findings.extend(scan_json_coordinates(child, label))
    return findings


def scan_checkpoint(path: Path, label: str) -> list[str]:
    """Inspect pickle opcodes as data; never torch.load / pickle.loads."""
    findings = []
    try:
        with gzip.open(path, "rb") as compressed:
            data = compressed.read(150 * 1024 * 1024 + 1)
        if len(data) > 150 * 1024 * 1024:
            return [f"{label}: 权重超过元数据审计大小限制"]
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            names = [name for name in archive.namelist() if name.endswith("data.pkl")]
            if len(names) != 1:
                return [f"{label}: 检查点元数据格式未知"]
            for op, value, position in pickletools.genops(archive.read(names[0])):
                if isinstance(value, str):
                    findings.extend(scan_text(value, f"{label}:metadata@{position}"))
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        findings.append(f"{label}: 无法审计检查点（{type(exc).__name__}）")
    return findings


def scan_jpeg_comments(data: bytes, label: str) -> list[str]:
    """Inspect JPEG COM markers without decoding the image.

    JPEG comments are application metadata, so they must be checked with the
    same redaction-aware text scanner as source files.  The parser stops at
    the start-of-scan marker; compressed image bytes are never interpreted as
    text.
    """
    findings: list[str] = []
    if len(data) < 2 or data[:2] != b"\xff\xd8":
        return findings
    offset = 2
    while offset + 1 < len(data):
        if data[offset] != 0xFF:
            offset += 1
            continue
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in {0xD8, 0xD9}:  # SOI / EOI have no payload.
            if marker == 0xD9:
                break
            continue
        if marker == 0xDA:  # Start of scan; the rest is compressed pixels.
            break
        if offset + 2 > len(data):
            break
        length = int.from_bytes(data[offset:offset + 2], "big")
        if length < 2 or offset + length > len(data):
            break
        payload = data[offset + 2:offset + length]
        if marker == 0xFE:
            try:
                comment = payload.decode("utf-8")
            except UnicodeDecodeError:
                comment = payload.decode("utf-8", errors="replace")
            findings.extend(scan_text(comment, f"{label}:jpeg-comment"))
        offset += length
    return findings


def audit(source: Path, *, workspace: bool = False) -> tuple[list[Path], list[str]]:
    files = public_files(source, workspace=workspace)
    # ``release_policy`` deliberately excludes compressed model archives from
    # packages, but those files still need the same metadata review when they
    # are present in a working tree.  Likewise, dotenv files must be surfaced
    # as findings instead of being silently skipped as hidden files.  Walk the
    # same published roots while pruning excluded directories so this extra
    # audit set cannot inspect unrelated local caches.
    extra: set[Path] = set()
    release_roots = PUBLIC_ROOTS if workspace else (".",)
    starts = [source / name for name in release_roots]
    if workspace:
        for path in source.glob(".env*"):
            if path.is_symlink():
                raise ValueError(f"发布范围含符号链接：{path.relative_to(source)}")
            if path.is_file():
                extra.add(path.relative_to(source))
    for start in starts:
        if not start.exists() or not start.is_dir():
            continue
        for directory, subdirs, names in os.walk(start, followlinks=False):
            relative = Path(directory).resolve().relative_to(source.resolve())
            subdirs[:] = [name for name in sorted(subdirs)
                          if not excluded(relative / name)]
            for name in sorted(names):
                path = relative / name
                if name.startswith(".env") or name.endswith(".pt.gz") or name.endswith(".onnx.gz"):
                    if (source / path).is_symlink():
                        raise ValueError(f"发布范围含符号链接：{path}")
                    extra.add(path)
    files = sorted(set(files) | extra)
    findings = []
    for relative in files:
        path = source / relative
        label = relative.as_posix()
        if (
            (path.name.startswith(".env") and path.name != ".env.example")
            or path.suffix.lower() in {".pem", ".key", ".p12", ".pfx"}
            or path.name.startswith(("id_rsa", "id_ed25519"))
        ):
            findings.append(f"{label}: 凭据文件禁止发布")
        if path.suffix.lower() in {".waypoints", ".params"} and OPERATIONAL_FILENAME.search(path.stem):
            findings.append(f"{label}: 航线/参数文件名含作业信息，请改为中性命名")
        if path.name.endswith(".pt.gz"):
            findings.extend(scan_checkpoint(path, label))
            continue
        raw = path.read_bytes()
        if path.suffix.lower() in {".jpg", ".jpeg", ".png"}:
            if b"Exif\x00\x00" in raw or b"http://ns.adobe.com/xap/" in raw or b"eXIf" in raw:
                findings.append(f"{label}: 图片含未经审查的 EXIF/XMP 元数据")
            if path.suffix.lower() in {".jpg", ".jpeg"}:
                findings.extend(scan_jpeg_comments(raw, label))
            # Textual PNG metadata can also carry identity.
            if path.suffix.lower() == ".png":
                offset = 8
                while offset + 12 <= len(raw):
                    size = struct.unpack(">I", raw[offset:offset + 4])[0]
                    kind = raw[offset + 4:offset + 8]
                    value = raw[offset + 8:offset + 8 + size]
                    # The pinned upstream logo has only this software label.
                    if kind in {b"tEXt", b"zTXt", b"iTXt"} and value != b"Software\x00www.inkscape.org":
                        findings.append(f"{label}: PNG 含未经审查的文本元数据")
                    offset += size + 12
            continue
        try:
            body = raw.decode("utf-8")
        except UnicodeDecodeError:
            findings.append(f"{label}: 未知二进制文件，需明确发布格式")
            continue
        findings.extend(scan_text(body, label))
        if path.suffix.lower() == ".waypoints":
            findings.extend(scan_waypoints(body, label))
        if path.suffix.lower() == ".json":
            try:
                findings.extend(scan_json_coordinates(json.loads(body), label))
            except json.JSONDecodeError:
                findings.append(f"{label}: 无效 JSON，无法审计坐标")
    return files, findings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--project", action="store_true", help="审计一个独立模块的全部公开文件")
    parser.add_argument("--json", type=Path, help="将脱敏审计结果写入本地文件")
    args = parser.parse_args(argv)
    try:
        files, findings = audit(args.root.resolve(), workspace=not args.project)
    except ValueError as exc:
        print(f"[隐私] {exc}", file=sys.stderr)
        return 1
    result = {"files_scanned": len(files), "findings": findings, "passed": not findings}
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for finding in findings:
        print(f"[隐私] {finding}", file=sys.stderr)
    print(f"[隐私] 扫描 {len(files)} 个文件，发现 {len(findings)} 项")
    return int(bool(findings))


if __name__ == "__main__":
    raise SystemExit(main())
