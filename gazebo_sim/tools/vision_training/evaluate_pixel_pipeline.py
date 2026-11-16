#!/usr/bin/env python3
"""用保留集图片验收实际像素处理链；标签仅在全部推理完成后用于评分。"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',type=int,choices=(1,2),required=True)
    parser.add_argument('--weights',type=Path,required=True)
    parser.add_argument('--dataset',type=Path,default=ROOT/'datasets/gazebo_v1')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--split',choices=('val','test'),default='test')
    args=parser.parse_args()
    import cv2
    cv2.setNumThreads(1)
    os.nice(15)
    import numpy as np
    from cuadc_sim.vision import VisionEngine
    source=args.dataset/f'task{args.task}'
    engine=VisionEngine({'task':args.task,'selected_group':'REAL'},weights=args.weights,
                       confidence=.25,inference_hz=100,inference_size=320 if args.task==1 else 960)
    if not engine.state.model_loaded:raise RuntimeError('模型加载失败')
    guarded=False
    forbidden=[]
    def audit(event,arguments):
        if guarded and event=='open' and arguments and isinstance(arguments[0],(str,bytes)):
            name=Path(str(arguments[0])).name
            if name in {'manifest.json','scenario.json','field_layout.json','mission_replay.json','latest.json'}:
                forbidden.append(name)
                raise PermissionError('推理不得读取标签或场景答案')
    sys.addaudithook(audit)
    predictions={}
    raw_directory=source/f'raw/{args.split}'
    using_raw=raw_directory.is_dir()
    image_directory=raw_directory if using_raw else source/f'images/{args.split}'
    telemetry={'relative_alt_m':10,'roll_rad':0,'pitch_rad':0,'yaw_rad':0,'north_m':0,'east_m':0}
    for image in sorted(image_directory.glob('*.jpg')):
        frame=cv2.imread(str(image))
        if frame is None:raise ValueError('图片损坏')
        engine.tracks.clear()
        engine.last_inference=0
        guarded=True
        engine.process(frame,telemetry)
        guarded=False
        predictions[image.relative_to(source).as_posix()]={
            'values':sorted({int(row['value']) for row in engine.state.visible_detections}),
            'sha256':hashlib.sha256(image.read_bytes()).hexdigest()}
    engine.tracks.clear();engine.last_inference=0
    guarded=True
    engine.process(np.zeros((480,640,3),np.uint8),telemetry)
    guarded=False
    blank_values=[int(row['value']) for row in engine.state.visible_detections]
    # No expected value, pose, colour or filename is passed to the detector.
    samples=json.loads((source/'manifest.json').read_text())['samples']
    scoring=[];per_value={};counts=Counter()
    for sample in samples:
        if sample['split']!=args.split:continue
        key=sample['raw_image'] if using_raw else sample['image']
        prediction=predictions[key]
        if prediction['sha256']!=sample['raw_sha256' if using_raw else 'sha256']:raise ValueError('保留集图片哈希不符')
        expected=None if sample.get('negative') else int(sample['value'])+(1 if args.task==1 else 0)
        observed=prediction['values']
        correct=expected in observed
        exact=observed==([] if expected is None else [expected])
        row=per_value.setdefault(str(expected),{'images':0,'correct':0,'exact':0})
        row['images']+=1;row['correct']+=int(correct);row['exact']+=int(exact)
        counts.update(images=1,positive_images=int(expected is not None),correct=int(correct),exact=int(exact),predictions=len(observed),
                      unexpected_values=len(set(observed)-{expected}))
        scoring.append({'image':sample['image'],'expected':expected,'observed':observed,'exact':exact})
    if len(predictions)!=counts['images']:raise ValueError('清单与保留集不一致')
    report={'task':args.task,'model_sha256':engine.state.model_sha256,
            'source':'YOLO','split':args.split,'counts':dict(counts),
            'target_content_recall':counts['correct']/max(1,counts['positive_images']),
            'exact_image_fraction':counts['exact']/counts['images'],
            'per_value':per_value,'blank_values':blank_values,
            'forbidden_reads':forbidden,'scored_after_inference':True,
            'input_frames_before_pixel_preprocessing':using_raw,
            'scope':'单靶 Gazebo 保留图的实际像素候选与 YOLO 内容识别，不是框 mAP、实物泛化或定位精度',
            'physical_validated':False,'samples':scoring}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key not in {'samples','per_value'}},ensure_ascii=False))
    return int(bool(blank_values or forbidden))


if __name__=='__main__':raise SystemExit(main())
