#!/usr/bin/env python3
"""在原生像素上增强后提取训练候选，保持父布局划分并混入旧课程。"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from cuadc_sim.pixel_rois import pixel_views


def main():
    cv2.setNumThreads(1)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sources', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=261032)
    args = parser.parse_args()
    output = args.output / 'task1'
    if output.exists():
        raise FileExistsError('训练输出必须是新目录')
    rng = np.random.default_rng(args.seed)
    records = []
    def write(image, labels, split, parent, seed, suffix, value=None):
        name = f'{len(records):05d}-{suffix}'
        for directory in ('images', 'labels'):
            (output / directory / split).mkdir(parents=True, exist_ok=True)
        target = output / 'images' / split / (name+'.jpg')
        cv2.imwrite(str(target), image, [cv2.IMWRITE_JPEG_QUALITY, 96])
        (output / 'labels' / split / (name+'.txt')).write_text('\n'.join(labels)+'\n')
        records.append({'image': target.relative_to(output).as_posix(), 'split': split,
                        'parent': parent, 'seed': hex(seed), 'value': value,
                        'sha256': hashlib.sha256(target.read_bytes()).hexdigest()})
    for source_root in args.sources:
        source = source_root / 'task1'
        manifest = json.loads((source / 'manifest.json').read_text())
        for sample in manifest['samples']:
            path = source / sample['image']
            original = cv2.imread(str(path))
            if original is None:
                raise ValueError('图片无法解码')
            height, width = original.shape[:2]
            rows = (source / 'labels' / sample['split'] / path.with_suffix('.txt').name).read_text().splitlines()
            corners = []
            for line in rows:
                cls, x, y, w, h = map(float, line.split())
                corners.append((int(cls), np.array([(x-w/2)*width, (y-h/2)*height,
                                                     (x+w/2)*width, (y+h/2)*height])))
            for copy in range(2 if sample['split'] == 'train' else 1):
                seed = int(rng.integers(0, 2**63))
                local = np.random.default_rng(seed)
                frame = original.copy()
                if copy:
                    frame = np.clip(frame.astype(np.float32)*local.uniform(.75, 1.15)
                                    + local.normal(0, local.uniform(.1, 1.2), frame.shape), 0, 255).astype(np.uint8)
                    if local.random() < .5:
                        frame = cv2.GaussianBlur(frame, (3, 3), float(local.uniform(.3, .65)))
                    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, int(local.integers(72, 96))])
                    if not ok:
                        raise ValueError('JPEG 增强失败')
                    frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                view = pixel_views(frame, sample['color'])[0]
                rh, rw = view.image.shape[:2]
                labels = []
                for cls, box in corners:
                    box = (box-np.array([view.origin_x, view.origin_y]*2))*view.scale
                    box += np.array([view.pad_x, view.pad_y]*2)
                    low = np.maximum(box[:2], 0)
                    high = np.minimum(box[2:], [rw-1, rh-1])
                    if min(high-low) < 3:
                        raise ValueError('像素候选遗漏已标注靶板')
                    normalized = np.r_[(low+high)/2/[rw, rh], (high-low)/[rw, rh]]
                    labels.append(str(cls)+' '+' '.join(f'{v:.7f}' for v in normalized))
                write(view.image, labels, sample['split'],
                      f'{source_root.name}/{sample["image"]}', seed, f'oblique-{copy}', sample['value'])
    # 只复用历史训练划分，防止新课程遗忘其他目标；历史验证/测试不进入训练。
    for old_name in ('flight_v3', 'flight_v4'):
        old = ROOT / 'datasets' / old_name / 'task1'
        groups = {}
        for path in sorted((old / 'images/train').glob('*.jpg')):
            label = old / 'labels/train' / path.with_suffix('.txt').name
            rows = label.read_text().splitlines()
            if not rows or not rows[0].strip():
                continue
            cls = int(rows[0].split()[0])
            groups.setdefault(cls, []).append((path, rows))
        for cls, items in groups.items():
            for index in rng.choice(len(items), min(20, len(items)), replace=False):
                path, labels = items[index]
                seed = int(rng.integers(0, 2**63))
                write(cv2.imread(str(path)), labels, 'train', f'{old_name}/images/train/{path.name}', seed, 'replay', cls)
    (output / 'data.yaml').write_text((args.sources[0] / 'task1/data.yaml').read_text())
    (output / 'manifest.json').write_text(json.dumps({
        'seed': args.seed, 'sources': [source.name for source in args.sources],
        'policy': '原生帧在像素候选前增强；父布局划分不变；只复用历史训练划分；私人视频与历史失败飞行不用于训练',
        'samples': records}, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'samples': len(records), 'splits': {split: sum(row['split'] == split for row in records)
                                                       for split in ('train', 'val', 'test')}}, ensure_ascii=False))


if __name__ == '__main__':
    main()
