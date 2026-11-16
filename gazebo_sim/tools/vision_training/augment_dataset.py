#!/usr/bin/env python3
"""按原生采集划分增强镜头、曝光和模糊；跨任务负样本减少误报。"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
from cuadc_sim.pixel_rois import pixel_views


def transform(image, labels, rng, group, augment, canonical=False, force_roi=False):
    height,width=image.shape[:2]
    boxes=[]
    for label in labels:
        cls,x,y,w,h=map(float,label.split())
        points=np.float32([[(x-w/2)*width,(y-h/2)*height],[(x+w/2)*width,(y-h/2)*height],
                           [(x+w/2)*width,(y+h/2)*height],[(x-w/2)*width,(y+h/2)*height]])
        boxes.append((int(cls),points))
    if augment:
        k=np.array([[width*.55,0,width/2],[0,width*.55,height/2],[0,0,1]],np.float64)
        d=np.array([rng.uniform(-.13,.13),rng.uniform(-.025,.025),0,0,0])
        image=cv2.undistort(image,k,d)
        boxes=[(cls,cv2.undistortPoints(points[:,None,:],k,d,P=k).reshape(-1,2)) for cls,points in boxes]
    if force_roi or canonical or rng.random()<.55:
        view=pixel_views(image,group,canonical=canonical)[0]
        if view.roi:
            image=view.image
            boxes=[(cls,(points-[view.origin_x,view.origin_y])*view.scale+[view.pad_x,view.pad_y]) for cls,points in boxes]
            if view.inverse_affine is not None:
                affine=cv2.invertAffineTransform(view.inverse_affine)
                boxes=[(cls,cv2.transform(np.float32(points[None,:,:]),affine)[0]) for cls,points in boxes]
    if augment:
        image=np.clip(image.astype(np.float32)*rng.uniform(.65,1.25)+rng.uniform(-15,15),0,255).astype(np.uint8)
        if rng.random()<.6:
            kernel=np.zeros((3,3),np.float32);kernel[1,:]=1/3
            image=cv2.filter2D(image,-1,kernel)
        if rng.random()<.4:image=cv2.GaussianBlur(image,(3,3),float(rng.uniform(.2,.8)))
        noise=rng.normal(0,rng.uniform(0,3),image.shape)
        image=np.clip(image.astype(np.float32)+noise,0,255).astype(np.uint8)
    height,width=image.shape[:2]
    output=[]
    for cls,points in boxes:
        lo=np.maximum(points.min(axis=0),[0,0]);hi=np.minimum(points.max(axis=0),[width-1,height-1])
        if np.min(hi-lo)<5:continue
        box=np.r_[(lo+hi)/2/[width,height],(hi-lo)/[width,height]]
        output.append(f'{cls} '+' '.join(f'{x:.7f}' for x in box))
    return image,output


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',type=int,choices=(1,2),required=True)
    parser.add_argument('--source',type=Path,default=ROOT/'datasets/gazebo_v1')
    parser.add_argument('--output',type=Path,default=ROOT/'datasets/domain_v2')
    parser.add_argument('--copies',type=int,default=3)
    parser.add_argument('--roi-only',action='store_true')
    args=parser.parse_args()
    source=args.source/f'task{args.task}';output=args.output/f'task{args.task}'
    manifest=json.loads((source/'manifest.json').read_text())
    rng=np.random.default_rng(20261019+args.task);records=[]
    for sample in manifest['samples']:
        path=source/sample['image'];split=sample['split']
        lines=(source/'labels'/split/path.with_suffix('.txt').name).read_text().splitlines()
        copies=args.copies if split=='train' else 1
        for copy in range(copies):
            seed=int(rng.integers(0,2**63));local=np.random.default_rng(seed)
            image,labels=transform(cv2.imread(str(path)),lines,local,sample['color'],split=='train',canonical=args.task==2,force_roi=args.roi_only)
            name=f'{path.stem}-aug{copy}'
            for folder in ('images','labels'):(output/folder/split).mkdir(parents=True,exist_ok=True)
            target=output/'images'/split/(name+'.jpg')
            cv2.imwrite(str(target),image,[cv2.IMWRITE_JPEG_QUALITY,int(local.integers(75,96))])
            (output/'labels'/split/(name+'.txt')).write_text('\n'.join(labels)+'\n')
            records.append({'image':f'images/{split}/{name}.jpg','split':split,'parent':sample['image'],'seed':hex(seed),'sha256':hashlib.sha256(target.read_bytes()).hexdigest()})
    # A different task's board is background, preventing number/aircraft cross-task hallucinations.
    other=args.source/f'task{3-args.task}'
    for split,count in [('train',100),('val',20)]:
        candidates=sorted((other/'images'/split).glob('*.jpg'))
        indices=np.linspace(0,len(candidates)-1,min(count,len(candidates)),dtype=int)
        for index in indices:
            path=candidates[index];name=f'negative-{path.stem}'
            target=output/'images'/split/(name+'.jpg')
            target.write_bytes(path.read_bytes());(output/'labels'/split/(name+'.txt')).write_text('')
            records.append({'image':f'images/{split}/{name}.jpg','split':split,'parent':f'task{3-args.task}/images/{split}/{path.name}','negative':True,'sha256':hashlib.sha256(target.read_bytes()).hexdigest()})
    (output/'data.yaml').write_text((source/'data.yaml').read_text())
    (output/'manifest.json').write_text(json.dumps({'source':args.source.name,'policy':'父图划分不变，增强只在原训练划分内产生，实拍视频未参与训练','samples':records},ensure_ascii=False,indent=2)+'\n')
    print(f'任务 {args.task}：{len(records)} 张增强/对照图片')


if __name__=='__main__':main()
