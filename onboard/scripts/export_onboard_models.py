#!/usr/bin/env python3
"""Export repository YOLO weights to an edge inference format.

Run this on the development workstation. NCNN is the first format to benchmark
on Raspberry Pi ARM; ONNX remains available when the installed runtime performs
better for a particular OpenCV / ONNX Runtime build.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil

from ultralytics import YOLO


APP_ROOT = Path(__file__).resolve().parents[1]
ROOT = APP_ROOT if (APP_ROOT / "src/cuadc_onboard").is_dir() else APP_ROOT.parent
MODELS = {
    1: (ROOT / "models/vision/task1_v4m_best.pt", 640),
    2: (ROOT / "models/vision/task2_sim_digits_v1.pt", 960),
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", choices=("1", "2", "all"), default="all")
    parser.add_argument("--format", choices=("ncnn", "onnx"), default="ncnn")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "models/onboard")
    args = parser.parse_args()
    tasks = (1, 2) if args.task == "all" else (int(args.task),)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for task in tasks:
        weights, image_size = MODELS[task]
        if not weights.is_file():
            raise FileNotFoundError(
                f"missing {weights}; decompress and verify the model archive first"
            )
        destination = args.output_dir / f"task{task}_{args.format}"
        generated_candidate = (
            weights.with_suffix(".onnx")
            if args.format == "onnx"
            else weights.with_name(f"{weights.stem}_ncnn_model")
        )
        if destination.exists() or generated_candidate.exists():
            existing = destination if destination.exists() else generated_candidate
            raise FileExistsError(
                f"refusing to overwrite {existing}; remove or rename it explicitly"
            )
        model = YOLO(str(weights))
        exported = Path(
            model.export(
                format=args.format,
                imgsz=image_size,
                device="cpu",
                half=False,
                dynamic=False,
                simplify=False,
            )
        )
        shutil.move(str(exported), destination)
        print(f"Task {task}: {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
