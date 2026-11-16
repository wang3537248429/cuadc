#!/usr/bin/env python3
"""生成飞行像素尺度课程；先降采样再提候选，避免只在放大后的清晰图上训练。"""
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


def prepare(image, labels, rng, color, task, *, apply_roi=True):
    height, width = image.shape[:2]
    boxes = []
    for row in labels:
        cls, x, y, w, h = map(float, row.split())
        corners = np.float32([[(x-w/2)*width, (y-h/2)*height],
                              [(x+w/2)*width, (y-h/2)*height],
                              [(x+w/2)*width, (y+h/2)*height],
                              [(x-w/2)*width, (y+h/2)*height]])
        boxes.append((int(cls), corners))
    # About 18 source pixels per 0.6 m panel at 18 m AGL / 1280 px / 100 deg.
    span = max((np.ptp(points, axis=0).max() for _, points in boxes), default=55)
    desired = float(rng.uniform(12, 34)) if rng.random() < .8 else float(rng.uniform(35, 65))
    scale = min(1., max(.12, desired / span))
    reduced = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    image = cv2.resize(reduced, (width, height), interpolation=cv2.INTER_LINEAR)
    source = np.float32([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]])
    destination = source + rng.uniform(-.045, .045, (4, 2)).astype(np.float32) * [width, height]
    perspective = cv2.getPerspectiveTransform(source, destination.astype(np.float32))
    image = cv2.warpPerspective(image, perspective, (width, height), borderMode=cv2.BORDER_REFLECT_101)
    boxes = [(cls, cv2.perspectiveTransform(points[None], perspective)[0]) for cls, points in boxes]
    image = np.clip(image.astype(np.float32)*rng.uniform(.65, 1.2)+rng.uniform(-12, 12), 0, 255).astype(np.uint8)
    if rng.random() < .5:
        size = int(rng.choice([3, 5]))
        kernel = np.zeros((size, size), np.float32)
        if rng.random() < .5:kernel[size//2, :] = 1/size
        else:kernel[:, size//2] = 1/size
        image = cv2.filter2D(image, -1, kernel)
    image = np.clip(image.astype(np.float32)+rng.normal(0, rng.uniform(0, 2), image.shape), 0, 255).astype(np.uint8)
    # The same pixel candidate code is used by live inference; labels are offline only.
    view = pixel_views(image, color, canonical=task == 2)[0] if apply_roi else None
    if view is not None and view.roi:
        boxes = [(cls, np.float32((points-[view.origin_x, view.origin_y])*view.scale+[view.pad_x, view.pad_y]))
                 for cls, points in boxes]
        if view.inverse_affine is not None:
            affine = cv2.invertAffineTransform(view.inverse_affine)
            boxes = [(cls, cv2.transform(points[None], affine)[0]) for cls, points in boxes]
        image = view.image
    height, width = image.shape[:2]
    output = []
    for cls, points in boxes:
        low = np.maximum(points.min(axis=0), [0, 0])
        high = np.minimum(points.max(axis=0), [width-1, height-1])
        if min(high-low) < 5:continue
        normalized = np.r_[(low+high)/2/[width, height], (high-low)/[width, height]]
        output.append(str(cls)+' '+' '.join(f'{x:.7f}' for x in normalized))
    return image, output, scale


def main():
    cv2.setNumThreads(1)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', type=int, choices=(1, 2), required=True)
    parser.add_argument('--source', type=Path, default=ROOT/'datasets/gazebo_v1')
    parser.add_argument('--output', type=Path, default=ROOT/'datasets/flight_v3')
    parser.add_argument('--seed', type=int, default=261026)
    args = parser.parse_args()
    output = args.output/f'task{args.task}'
    if output.exists():raise FileExistsError('为避免旧样本混入，请使用新的空输出目录')
    rng = np.random.default_rng([args.seed, args.task])
    records = []
    source = args.source/f'task{args.task}'
    samples = json.loads((source/'manifest.json').read_text())['samples']
    def write(sample, parent_root, copy, negative=False):
        split = sample['split'];path = parent_root/sample['image']
        labels = [] if negative else (parent_root/'labels'/split/path.with_suffix('.txt').name).read_text().splitlines()
        original_labels=list(labels)
        seed = int(rng.integers(0, 2**63));local = np.random.default_rng(seed)
        image, labels, factor = prepare(cv2.imread(str(path)), labels, local, sample['color'], args.task)
        name = ('negative-' if negative else '')+path.stem+f'-flight{copy}'
        for folder in ('images', 'labels'):(output/folder/split).mkdir(parents=True, exist_ok=True)
        target = output/'images'/split/(name+'.jpg')
        cv2.imwrite(str(target), image, [cv2.IMWRITE_JPEG_QUALITY, int(local.integers(78, 97))])
        (output/'labels'/split/(name+'.txt')).write_text('\n'.join(labels)+'\n')
        records.append({'image':f'images/{split}/{name}.jpg', 'split':split,
                        'parent':f'{parent_root.name}/{sample["image"]}', 'seed':hex(seed),
                        'value':None if negative else sample['value'], 'negative':negative,
                        'source_downsample_factor':factor, 'sha256':hashlib.sha256(target.read_bytes()).hexdigest()})
        if split!='train':
            raw, _, _ = prepare(cv2.imread(str(path)), original_labels,
                                 np.random.default_rng(seed), sample['color'], args.task, apply_roi=False)
            # The separate raw export is scored through the live pixel pipeline;
            # it is not used by YOLO's training/validation loader.
            raw_path=output/'raw'/split/(name+'.jpg')
            raw_path.parent.mkdir(parents=True,exist_ok=True)
            cv2.imwrite(str(raw_path),raw,[cv2.IMWRITE_JPEG_QUALITY,92])
            records[-1].update(raw_image=raw_path.relative_to(output).as_posix(),
                               raw_sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest())
    for sample in samples:
        split = sample['split']
        copies = ({'train':3, 'val':2, 'test':3} if args.task == 1 else {'train':2, 'val':1, 'test':2})[split]
        if args.task == 1 and split == 'train' and sample['value'] in (3, 4, 5, 10):copies += 2
        for copy in range(copies):write(sample, source, copy)
    other = args.source/f'task{3-args.task}'
    others = json.loads((other/'manifest.json').read_text())['samples']
    for split, count in [('train', 80), ('val', 20), ('test', 20)]:
        candidates = [row for row in others if row['split'] == split]
        for index in np.linspace(0, len(candidates)-1, min(count, len(candidates)), dtype=int):
            write(candidates[index], other, 0, negative=True)
    (output/'data.yaml').write_text((source/'data.yaml').read_text())
    (output/'manifest.json').write_text(json.dumps({'seed':args.seed, 'source':'gazebo_v1',
        'policy':'父图划分不变；低分辨率退化在候选放大之前产生；不使用私人视频或既有失败飞行作为训练帧',
        'samples':records}, ensure_ascii=False, indent=2)+'\n')
    print(f'任务 {args.task}：新增 {len(records)} 张飞行尺度图片', flush=True)


if __name__ == '__main__':main()
