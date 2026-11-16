from __future__ import annotations

from pathlib import Path


AUTO_CAMERA_DEVICE = "auto"


def is_auto_camera_device(value: str) -> bool:
    return value.strip().lower() == AUTO_CAMERA_DEVICE


def _video_node_sort_key(path: Path) -> tuple[int, str]:
    suffix = path.name.removeprefix("video")
    return (int(suffix) if suffix.isdigit() else 1_000_000, path.name)


def discover_camera_devices(
    *,
    by_id_dir: Path = Path("/dev/v4l/by-id"),
    dev_dir: Path = Path("/dev"),
    sys_class_dir: Path = Path("/sys/class/video4linux"),
) -> tuple[str, ...]:
    """Return stable USB UVC capture nodes, excluding metadata/codec nodes."""

    devices: list[str] = []
    resolved_nodes: set[str] = set()

    if by_id_dir.is_dir():
        for link in sorted(by_id_dir.glob("*-video-index0")):
            if not link.exists():
                continue
            try:
                resolved = str(link.resolve(strict=True))
            except OSError:
                continue
            if resolved in resolved_nodes:
                continue
            devices.append(str(link))
            resolved_nodes.add(resolved)

    if sys_class_dir.is_dir() and dev_dir.is_dir():
        for node in sorted(dev_dir.glob("video*"), key=_video_node_sort_key):
            sys_entry = sys_class_dir / node.name
            driver_link = sys_entry / "device" / "driver"
            try:
                interface_index = (sys_entry / "index").read_text(
                    encoding="utf-8"
                ).strip()
                driver = driver_link.resolve(strict=True).name
                resolved = str(node.resolve(strict=True))
            except OSError:
                continue
            if (
                interface_index != "0"
                or driver != "uvcvideo"
                or resolved in resolved_nodes
            ):
                continue
            devices.append(str(node))
            resolved_nodes.add(resolved)

    return tuple(devices)


def camera_candidates(configured: str) -> tuple[str, ...]:
    if is_auto_camera_device(configured):
        return discover_camera_devices()
    return (configured,)


def camera_identity(device: str) -> str:
    path = Path(device)
    if path.name:
        return path.name
    return device
