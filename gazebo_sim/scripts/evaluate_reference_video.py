#!/usr/bin/env python3
"""对私有视频的候选区域做离线 YOLO 检查；不将原片或抽帧写入公开目录。"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from cuadc_sim.pixel_rois import pixel_views, panel_pixels_support


def main():
    cv2.setNumThreads(1)
    os.nice(15)
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video',type=Path)
    parser.add_argument('--weights',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=ROOT.parent/'.private/video-eval')
    parser.add_argument('--samples',type=int,default=96)
    args=parser.parse_args()
    if not any(part in ('.private','.runtime') for part in args.output.resolve().parts):
        raise ValueError('原视频抽帧只能输出到 .private 或 .runtime')
    args.output.mkdir(parents=True,exist_ok=True)
    from ultralytics import YOLO
    import torch
    torch.set_num_threads(2)
    model=YOLO(str(args.weights))
    capture=cv2.VideoCapture(str(args.video),cv2.CAP_FFMPEG,[cv2.CAP_PROP_N_THREADS,1])
    fps=capture.get(cv2.CAP_PROP_FPS)
    if not capture.isOpened() or fps<=0:
        raise ValueError('参考视频无法读取')
    duration=capture.get(cv2.CAP_PROP_FRAME_COUNT)/fps
    records=[]
    try:
        for index,seconds in enumerate(np.linspace(duration*.25,duration*.85,args.samples)):
            capture.set(cv2.CAP_PROP_POS_FRAMES,round(float(seconds)*fps));ok,frame=capture.read()
            if not ok:continue
            for n,view in enumerate(pixel_views(frame,'A')):
                if not view.roi:continue
                # Exclude screen menus and telemetry overlays before retaining a private candidate.
                crop_width=(640-2*view.pad_x)/view.scale;crop_height=(640-2*view.pad_y)/view.scale
                x1,y1=view.origin_x,view.origin_y;x2,y2=x1+crop_width,y1+crop_height
                height,width=frame.shape[:2]
                if y1<height*.3 or y2>height*.92:continue
                predictions=model.predict(view.image,imgsz=320,conf=.25,verbose=False)[0]
                detections=[]
                for box in predictions.boxes:
                    bbox=box.xyxy[0].tolist();original=view.to_original(bbox)
                    if panel_pixels_support(frame,original):
                        detections.append({'class':predictions.names[int(box.cls[0])],'confidence':float(box.conf[0])})
                filename=f'candidate-{index:02d}-{n}.jpg'
                cv2.imwrite(str(args.output/filename),view.image)
                records.append({'candidate':filename,'video_time_s':float(seconds),'detections':detections})
    finally:capture.release()
    (args.output/'candidates.json').write_text(json.dumps({'scope':'私有候选片段，需人工核对靶标和遮挡；不得据此声称全类别实机验收通过','records':records},ensure_ascii=False,indent=2)+'\n')
    print(f'已生成 {len(records)} 个私有候选区域，原视频未复制')


if __name__=='__main__':main()
