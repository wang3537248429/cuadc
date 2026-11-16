#!/usr/bin/env python3
"""已人工确认类别的参考靶板补训；去除外部背景，输出始终保持私有。"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for split in ('train','val','test'):
        parser.add_argument(f'--{split}-image',type=Path,required=True)
    parser.add_argument('--value',type=int,required=True,help='人工核对的任务一类别值 1..12')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if '.private' not in args.output.resolve().parts or not 1<=args.value<=12:
        raise ValueError('原视频派生样本只允许写入 .private；类别须已人工确认')
    out=args.output/'task1'
    if out.exists():raise FileExistsError('输出必须是新目录')
    cv2.setNumThreads(1);rng=np.random.default_rng(261101);records=[]
    for split in ('train','val','test'):
        frame=cv2.imread(str(getattr(args,f'{split}_image')))
        if frame is None:raise ValueError('私有参考图片无法读取')
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        blue=cv2.inRange(hsv,np.array([90,80,40]),np.array([140,255,255]))
        contours,hierarchy=cv2.findContours(blue,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        candidates=[i for i,c in enumerate(contours) if hierarchy[0][i][3]==-1
                    and hierarchy[0][i][2]>=0 and 200<cv2.contourArea(c)<5000
                    and cv2.boundingRect(c)[0]>frame.shape[1]*.75]
        if len(candidates)!=1:raise ValueError(f'{split} 需人工确认唯一蓝色靶板，候选数 {len(candidates)}')
        index=candidates[0];child=hierarchy[0][index][2];holes=[]
        while child>=0:
            holes.append(contours[child]);child=hierarchy[0][child][0]
        board=max(holes,key=cv2.contourArea);hull=cv2.convexHull(contours[index])
        x,y,w,h=cv2.boundingRect(hull);x0,y0=max(0,x-6),max(0,y-6)
        x2,y2=min(frame.shape[1],x+w+6),min(frame.shape[0],y+h+6)
        mask=np.zeros(frame.shape[:2],np.uint8);cv2.fillConvexPoly(mask,hull,255)
        clean=np.full_like(frame,114);clean[mask>0]=frame[mask>0]
        patch=clean[y0:y2,x0:x2];points=board.reshape(-1,2).astype(np.float32)-[x0,y0]
        size=max(patch.shape[:2])*2;offset=np.array([(size-patch.shape[1])//2,(size-patch.shape[0])//2])
        base=np.full((size,size,3),114,np.uint8)
        base[offset[1]:offset[1]+patch.shape[0],offset[0]:offset[0]+patch.shape[1]]=patch
        points+=offset
        for j in range(32 if split=='train' else 1):
            seed=int(rng.integers(0,2**63));local=np.random.default_rng(seed)
            angle=float(local.uniform(-180,180)) if split=='train' else 0
            affine=cv2.getRotationMatrix2D((size/2,size/2),angle,1)
            image=cv2.warpAffine(base,affine,(size,size),borderValue=(114,114,114))
            pts=cv2.transform(np.float32(points[None]),affine)[0]
            if split=='train':
                image=np.clip(image.astype(float)*local.uniform(.8,1.15)+local.normal(0,.4,image.shape),0,255).astype(np.uint8)
            pad=np.array([(640-size)//2,(480-size)//2]);raw=np.full((480,640,3),114,np.uint8)
            raw[pad[1]:pad[1]+size,pad[0]:pad[0]+size]=image;pts+=pad
            low,high=pts.min(axis=0),pts.max(axis=0)
            box=np.r_[(low+high)/2/[640,480],(high-low)/[640,480]]
            name=f'private-panel-{split}-{j:03d}.jpg'
            for directory in ('images','labels'):(out/directory/split).mkdir(parents=True,exist_ok=True)
            path=out/'images'/split/name;cv2.imwrite(str(path),raw,[cv2.IMWRITE_JPEG_QUALITY,96])
            (out/'labels'/split/Path(name).with_suffix('.txt')).write_text(str(args.value-1)+' '+' '.join(f'{v:.7f}' for v in box)+'\n')
            records.append({'image':f'images/{split}/{name}','split':split,'seed':hex(seed),
                'value':args.value-1,'color':'A','parent':f'masked-video-view/{split}',
                'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    # Keep all generic parent splits; no generic held-out image enters training.
    source=ROOT/'datasets/gazebo_flight_v10/task1'
    for sample in json.loads((source/'manifest.json').read_text())['samples']:
        target=out/sample['image'];target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source/sample['image'],target)
        label=out/'labels'/sample['split']/target.with_suffix('.txt').name
        label.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source/'labels'/sample['split']/label.name,label);records.append(sample)
    (out/'data.yaml').write_text((source/'data.yaml').read_text())
    (out/'manifest.json').write_text(json.dumps({'scope':'同源参考视频开发回归；不是独立实机航次或全类别验收；原片和派生样本均不公开',
        'samples':records},ensure_ascii=False,indent=2)+'\n')
    print(f'私有匿名靶板课程：{len(records)} 张原生/开发图片')


if __name__=='__main__':main()
