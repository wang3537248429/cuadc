"""公开源码范围；运行资料和许可未明确的资源不得随源码分发。"""
from __future__ import annotations

import os
from pathlib import Path


PUBLIC_ROOTS = (
    "README.md", "LICENSE", "LICENSES", "SECURITY.md", "CONTRIBUTING.md",
    "THIRD_PARTY_NOTICES.md", "pytest.ini", ".gitignore", ".gitattributes", ".github",
    "gazebo_sim", "onboard", "ground_station", "scripts", "docs",
)
EXCLUDED_NAMES = {
    ".git", ".private",
    ".tmp_docs", ".venv", ".venv-onboard", ".venv-windows", "venv", "env",
    ".deps", ".runtime", "runs", "dist", "build", "wheelhouse", "package",
    "__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache", ".cache",
    ".ultralytics", ".ultralytics_cfg", ".torch_cache", ".matplotlib_cfg",
    ".ipynb_checkpoints", ".coverage", "htmlcov", "core",
}
EXCLUDED_SUFFIXES = {
    ".pyc", ".pyo", ".pt", ".onnx", ".engine", ".pdf", ".docx", ".ipynb",
    ".log", ".tlog", ".raw", ".ulg", ".bag", ".mp4", ".mov", ".avi",
    ".bin", ".tmp", ".bak", ".swp", ".swo", ".cache",
}
# These are the two runtime checkpoints explicitly approved for publication.
# Keep the training-only pretraining archive and any newly added weights out of
# a release until their provenance has been reviewed.
PUBLIC_MODEL_ARCHIVES = {
    "task1_v4m_best.pt.gz",
    "task2_sim_digits_v1.pt.gz",
}
# 数据和样例的再分发许可尚未确认，只发布目录说明。
DATASET_ROOT = "datasets"
DATASET_KEEP = {"README.md"}
# 白名单之外的隐藏目录一律不发布，避免工具缓存或本地工作目录混进源码包。
HIDDEN_ALLOWED = {".github", ".gitignore", ".gitattributes"}


def restricted_resource(path: Path) -> bool:
    """Uncleared artwork, derived data and training records stay local."""
    parts = path.parts
    if "targets" in parts and "assets" in parts:
        tail = parts[parts.index("targets") + 1:]
        return bool(tail) and tail not in {("catalog.json",), ("README.md",), ("REFERENCE_SHA256SUMS",)}
    if "datasets" in parts:
        tail = parts[parts.index("datasets") + 1:]
        return bool(tail) and tail != ("README.md",)
    if "vision" in parts and "models" in parts:
        tail = parts[parts.index("vision") + 1:]
        return bool(tail) and tail not in {
            ("MODEL_SHA256SUMS",), ("ARCHIVE_SHA256SUMS",), ("README.md",),
        } and not (len(tail) == 1 and tail[0] in PUBLIC_MODEL_ARCHIVES)
    return False


def excluded(path: Path) -> bool:
    """Only generated/private material is silently excluded; secrets fail audit."""
    # Keep dotenv files visible to the privacy audit.  A checked-in
    # ``.env.example`` is useful documentation, while ``.env``/``.env.local``
    # must be reported as a publishing error instead of disappearing from the
    # file list before the audit can inspect them.
    if any(
        part.startswith(".")
        and part not in HIDDEN_ALLOWED
        and not (part.startswith(".env") and part == path.name)
        for part in path.parts
    ):
        return True
    if restricted_resource(path):
        return True
    if DATASET_ROOT in path.parts:
        if path.parts[-1] == DATASET_ROOT:
            return False          # 允许继续遍历，由子项决定保留与否
        return not DATASET_KEEP.intersection(path.parts)
    if any(part in EXCLUDED_NAMES or part.endswith(".egg-info") for part in path.parts):
        return True
    if path.suffix.lower() in EXCLUDED_SUFFIXES:
        return True
    # Only the explicitly reviewed runtime model archives enter a release.
    if (
        path.name.endswith((".pt.gz", ".onnx.gz"))
        and path.name not in PUBLIC_MODEL_ARCHIVES
    ):
        return True
    if path.name.endswith((".local.yaml", ".local.yml", ".local.json")) or path.name.startswith("core."):
        return True
    if "vision_training" in path.parts:
        return bool({"dataset", "calibration", "demo"}.intersection(path.parts))
    return False


def public_files(source: Path, *, workspace: bool = False) -> list[Path]:
    """Return relative regular paths, refusing symlinks before reading them."""
    paths: list[Path] = []
    starts = [source / name for name in PUBLIC_ROOTS] if workspace else [source]
    for start in starts:
        if not start.exists() and not start.is_symlink():
            continue
        if start.is_symlink():
            raise ValueError(f"发布范围含符号链接：{start.relative_to(source)}")
        if start.is_file():
            paths.append(start.relative_to(source))
            continue
        for directory, subdirs, files in os.walk(start, followlinks=False):
            relative = Path(directory).relative_to(source)
            kept = []
            for name in sorted(subdirs):
                child = relative / name
                if excluded(child):
                    continue
                if (source / child).is_symlink():
                    raise ValueError(f"发布范围含符号链接：{child}")
                kept.append(name)
            subdirs[:] = kept
            for name in sorted(files):
                child = relative / name
                if excluded(child):
                    continue
                if (source / child).is_symlink() or not (source / child).is_file():
                    raise ValueError(f"发布范围含非常规文件：{child}")
                if any(ord(character) < 32 for character in str(child)):
                    raise ValueError("发布文件名包含控制字符")
                paths.append(child)
    return sorted(paths)
