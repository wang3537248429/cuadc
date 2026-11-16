#!/usr/bin/env python3
"""确定性五边井数据集：共享真实模型尺寸，三维投影与种子分组。"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from cuadc_sim.well_geometry import footprint, WALL_HEIGHT_M, WALL_THICKNESS_M

TASK1_NAMES = ['machine_gunner', 'rocketeer', 'multirotor', 'fixed_wing', 'truck', 'air_defense',
               'tank', 'helicopter', 'fighter', 'transport', 'recon', 'bomber']
TASK2_NAMES = [f'tens_{n}' for n in range(10)] + [f'ones_{n}' for n in range(10)]


def rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]], dtype=float)


def sample(task: int, seed: int, index: int, size: int = 640):
    rng = np.random.default_rng(seed)
    background = np.empty((size, size, 3), np.float32)
    background[:] = rng.uniform((35, 65, 30), (100, 140, 105))
    noise = rng.normal(0, 12, (32, 32, 3)).astype(np.float32)
    background += cv2.resize(noise, (size, size), interpolation=cv2.INTER_CUBIC)
    background += rng.normal(0, 3.5, background.shape)
    image = np.clip(background, 0, 255).astype(np.uint8)
    # Panels span roughly 8–110 pixels, including the difficult flight scale.
    altitude = float(np.exp(rng.uniform(np.log(1.5), np.log(21))))
    focal = size / (2 * math.tan(math.radians(100) / 2))
    tilt_x, tilt_y = rng.uniform(-0.22, 0.22, 2)
    yaw = float(rng.uniform(-math.pi, math.pi))
    cr, sr, cp, sp, cy, sy = math.cos(tilt_x), math.sin(tilt_x), math.cos(tilt_y), math.sin(tilt_y), math.cos(yaw), math.sin(yaw)
    r = np.array([[cp*cy, sr*sp*cy-cr*sy, cr*sp*cy+sr*sy],
                  [cp*sy, sr*sp*sy+cr*cy, cr*sp*sy-sr*cy], [-sp, sr*cp, cr*cp]])
    # Camera is expressed in a convenient downward-looking coordinate frame.
    def project(points, z):
        points = np.column_stack([points, np.full(len(points), altitude-z)]) @ r
        return (points[:, :2] / points[:, 2:] * focal + size / 2).astype(np.float32)
    labels, metadata = [], []
    semantic = np.zeros((size, size), np.uint16)
    instances = []
    objects = int(rng.integers(1, 5)) if altitude > 3 else 1
    centers = [(0.25, 0.25), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)]
    rng.shuffle(centers)
    for j in range(objects):
        center = (np.array(centers[j]) + rng.uniform(-0.09, 0.09, 2) - 0.5) * (size * altitude / focal)
        heading = float(rng.uniform(-math.pi, math.pi))
        board_heading = heading if task == 2 else float(rng.uniform(-math.pi, math.pi))
        outline = np.array(footprint()) @ rotation(heading).T + center
        bottom, top = project(outline, 0.03), project(outline, WALL_HEIGHT_M)
        shadow = project(outline + np.array([0.22, 0.35]), 0)
        shadow_mask = np.zeros((size, size), np.uint8)
        cv2.fillConvexPoly(shadow_mask, np.round(shadow).astype(np.int32), 255)
        image[shadow_mask > 0] = (image[shadow_mask > 0] * float(rng.uniform(0.55, 0.8))).astype(np.uint8)
        team_color = (180, 80, 12) if rng.random() < 0.5 else (15, 20, 200)
        cv2.fillConvexPoly(image, np.round(bottom).astype(np.int32), team_color)
        cv2.fillConvexPoly(semantic, np.round(bottom).astype(np.int32), 0)
        occupied = bool(rng.random() > 0.15)
        value = (index * 5 + j * 7) % 12 if task == 1 else (index * 37 + j * 17) % 100
        corners = np.array([[-.3, -.3], [.3, -.3], [.3, .3], [-.3, .3]])
        world_panel = corners @ rotation(board_heading).T + center
        panel_points = project(world_panel, 0.034)
        if occupied:
            name = TASK1_NAMES[value] if task == 1 else f'number-{value:02d}'
            source = ROOT / f'assets/targets/reference/task{task}/{name}.png'
            texture = cv2.imread(str(source), cv2.IMREAD_COLOR)
            h, w = texture.shape[:2]
            homography = cv2.getPerspectiveTransform(np.float32([[0, h-1], [w-1, h-1], [w-1, 0], [0, 0]]), panel_points)
            warped = cv2.warpPerspective(texture, homography, (size, size))
            mask = np.zeros((size, size), np.uint8)
            cv2.fillConvexPoly(mask, np.round(panel_points).astype(np.int32), 255)
            image[mask > 0] = warped[mask > 0]
            parts = [(value, corners)] if task == 1 else [
                (value//10, np.array([[-.3,-.3],[0,-.3],[0,.3],[-.3,.3]])),
                (10+value%10, np.array([[0,-.3],[.3,-.3],[.3,.3],[0,.3]])),
            ]
            for class_id, panel in parts:
                box = project(panel @ rotation(board_heading).T + center, 0.034)
                instance_id = len(instances) + 1
                cv2.fillConvexPoly(semantic, np.round(box).astype(np.int32), instance_id)
                instances.append((class_id, abs(cv2.contourArea(box))))
        # Render five physical wall faces after the panel, giving real occlusion.
        for k in sorted(range(5), key=lambda k: float(np.mean(top[[k,(k+1)%5], 1]))):
            nxt = (k+1) % 5
            face = np.array([bottom[k], bottom[nxt], top[nxt], top[k]])
            shade = int(rng.uniform(150, 225))
            cv2.fillConvexPoly(image, np.round(face).astype(np.int32), (shade, shade, shade))
            cv2.fillConvexPoly(semantic, np.round(face).astype(np.int32), 0)
            cv2.line(image, tuple(np.round(top[k]).astype(int)), tuple(np.round(top[nxt]).astype(int)),
                     (240, 240, 240), max(1, round(focal*WALL_THICKNESS_M/altitude)), cv2.LINE_AA)
        metadata.append({'value': value if occupied else None, 'heading': heading, 'board_heading': board_heading})
    # Only label actually visible printed panels; reject hidden or badly clipped ones.
    for instance_id, (class_id, projected_area) in enumerate(instances, 1):
        ys, xs = np.nonzero(semantic == instance_id)
        if len(xs) < max(25, projected_area * .6):
            continue
        if np.count_nonzero(np.mean(image[ys, xs], axis=1) < 110) < 3:
            continue
        x1, y1, x2, y2 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
        if x2-x1 < 6 or y2-y1 < 6:
            continue
        labels.append(f'{class_id} {(x1+x2)/(2*size):.7f} {(y1+y2)/(2*size):.7f} {(x2-x1)/size:.7f} {(y2-y1)/size:.7f}')
    if rng.random() < .55:
        image = cv2.GaussianBlur(image, (3,3), float(rng.uniform(.15,.65)))
    image = np.clip(image.astype(np.float32)*rng.uniform(.72,1.16)+rng.uniform(-5,5), 0,255).astype(np.uint8)
    return image, labels, {'seed': hex(seed), 'altitude_m': altitude, 'objects': metadata}


def generate(output: Path, train: int, val: int, test: int, seed: int, size: int = 640):
    summary = {'schema': 'cuadc-pentagon-dataset.v1', 'master_seed': seed,
               'geometry': {'vertices': footprint(), 'wall_height_m': WALL_HEIGHT_M},
               'split_policy': '训练/验证/测试使用独立 SeedSequence 子流，无帧复用', 'tasks': {}}
    for task in (1, 2):
        task_dir = output / f'task{task}'
        records = []
        class_counts = {split: [0]*(12 if task == 1 else 20) for split in ('train','val','test')}
        streams = np.random.SeedSequence([seed, task]).spawn(3)
        hashes = set()
        for split_index, (split, count) in enumerate((('train',train),('val',val),('test',test))):
            seeds = streams[split_index].generate_state(count, dtype=np.uint64)
            for index, sample_seed in enumerate(seeds):
                image, labels, meta = sample(task, int(sample_seed), index, size)
                if split != 'test':
                    # Train on unscaled 320 px crops, preserving flight-scale pixels.
                    crop_rng = np.random.default_rng(int(sample_seed) ^ 0xC0FFEE)
                    if labels:
                        chosen = list(map(float, labels[int(crop_rng.integers(len(labels)))].split()[1:]))
                        cx, cy = chosen[0]*size, chosen[1]*size
                    else:
                        cx, cy = size/2, size/2
                    x0 = int(np.clip(cx-160+crop_rng.uniform(-45,45),0,size-320))
                    y0 = int(np.clip(cy-160+crop_rng.uniform(-45,45),0,size-320))
                    new_labels=[]
                    for line in labels:
                        cls,cx,cy,w,h=map(float,line.split())
                        x1,y1,x2,y2=(cx-w/2)*size-x0,(cy-h/2)*size-y0,(cx+w/2)*size-x0,(cy+h/2)*size-y0
                        original=max(1,(x2-x1)*(y2-y1))
                        x1,y1,x2,y2=max(0,x1),max(0,y1),min(319,x2),min(319,y2)
                        if (x2-x1)*(y2-y1) < original*.75 or min(x2-x1,y2-y1)<6:
                            continue
                        new_labels.append(f'{int(cls)} {(x1+x2)/640:.7f} {(y1+y2)/640:.7f} {(x2-x1)/320:.7f} {(y2-y1)/320:.7f}')
                    image=image[y0:y0+320,x0:x0+320].copy()
                    labels=new_labels
                    meta['crop_xywh']=[x0,y0,320,320]
                digest = hashlib.sha256(image.tobytes()).hexdigest()
                if digest in hashes:
                    raise RuntimeError('重复图像跨越数据划分')
                hashes.add(digest)
                name = f'frame_{index:06d}'
                image_dir, label_dir = task_dir/'images'/split, task_dir/'labels'/split
                image_dir.mkdir(parents=True,exist_ok=True); label_dir.mkdir(parents=True,exist_ok=True)
                cv2.imwrite(str(image_dir/f'{name}.jpg'),image,[cv2.IMWRITE_JPEG_QUALITY,92])
                (label_dir/f'{name}.txt').write_text('\n'.join(labels)+'\n')
                for line in labels:
                    class_counts[split][int(line.split()[0])] += 1
                records.append({'split':split,'frame':name, 'pixel_sha256':digest, 'file_sha256':hashlib.sha256((image_dir/f'{name}.jpg').read_bytes()).hexdigest(), **meta})
            print(f'[数据集] task{task} {split}: {count}',flush=True)
        data={'train':'images/train','val':'images/val','test':'images/test','names':TASK1_NAMES if task==1 else TASK2_NAMES}
        (task_dir/'data.yaml').write_text(yaml.safe_dump(data,allow_unicode=True,sort_keys=False))
        (task_dir/'manifest.json').write_text(json.dumps({'records':records,'class_counts':class_counts},ensure_ascii=False,indent=2))
        summary['tasks'][str(task)]={'train':train,'val':val,'test':test,'class_counts':class_counts}
    (output/'manifest.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'.runtime/pentagon-dataset')
    parser.add_argument('--train',type=int,default=720)
    parser.add_argument('--val',type=int,default=144)
    parser.add_argument('--test',type=int,default=144)
    parser.add_argument('--seed',type=int,default=20261016)
    args=parser.parse_args()
    generate(args.output,args.train,args.val,args.test,args.seed)


if __name__=='__main__': main()
