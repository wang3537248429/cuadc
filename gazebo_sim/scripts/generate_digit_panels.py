#!/usr/bin/env python3
"""按规则图 4 生成白底黑字双数字板，不读取旧蓝色合成靶图。"""
from pathlib import Path
import argparse
import os
import shutil


FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
)


def find_font(requested: Path | None) -> Path:
    if requested is not None:
        if requested.is_file():
            return requested
        raise FileNotFoundError(f"字体文件不存在：{requested}")
    configured = os.environ.get("CUADC_FONT")
    if configured:
        return find_font(Path(configured).expanduser())
    for candidate in FONT_CANDIDATES:
        path = Path(candidate)
        if path.is_file():
            return path
    fc_match = shutil.which("fc-match")
    if fc_match:
        import subprocess

        result = subprocess.run(
            [fc_match, "--format=%{file}", "sans:style=Bold"],
            check=False,
            capture_output=True,
            text=True,
        )
        path = Path(result.stdout.strip())
        if path.is_file():
            return path
    raise RuntimeError(
        "未找到可用字体；请安装 DejaVu Sans/Noto Sans，或设置 CUADC_FONT"
    )


def main() -> int:
    from PIL import Image, ImageDraw, ImageFont

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--font", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "assets/targets/reference/task2")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    font_path = find_font(args.font)
    # 600 px maps to 600 mm. Each 300 x 600 panel contains one ~400 mm glyph.
    size = 500
    while True:
        font = ImageFont.truetype(str(font_path), size)
        bounds = font.getbbox("8")
        height = bounds[3] - bounds[1]
        if abs(height - 400) <= 1:
            break
        size += 1 if height < 400 else -1
    for value in range(100):
        image = Image.new("RGB", (600, 600), "white")
        draw = ImageDraw.Draw(image)
        for index, digit in enumerate(f"{value:02d}"):
            left, top, right, bottom = draw.textbbox((0, 0), digit, font=font)
            x = index * 300 + (300 - (right - left)) / 2 - left
            y = (600 - (bottom - top)) / 2 - top
            draw.text((x, y), digit, fill="black", font=font)
        draw.line((300, 0, 300, 599), fill=(100, 100, 100), width=1)
        image.save(args.output / f"number-{value:02d}.png")
    print("[数字板] 00–99 已生成：600 mm 方板，两个 300 × 600 mm 数字面板")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
