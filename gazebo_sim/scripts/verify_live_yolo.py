#!/usr/bin/env python3
"""真模型像素隔离验收：拒绝读取场景答案，并对原图/空白图做对照。"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',type=int,choices=(1,2),required=True)
    parser.add_argument('--weights',type=Path,required=True)
    parser.add_argument('--image',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--expect',type=int,nargs='+',required=True,
                        help='独立验收人员指定的预期内容；只在推理结束后比对')
    parser.add_argument('--group',choices=('A','B'),default='A')
    args=parser.parse_args()
    import cv2
    import numpy as np
    import torch
    from cuadc_sim.vision import VisionEngine
    torch.set_num_threads(2)
    frame=cv2.imread(str(args.image))
    if frame is None:
        raise FileNotFoundError('缺少 Gazebo 原生验证画面')
    blocked={'scenario.json','mission_replay.json','field_layout.json','latest.json','result.json','manifest.json'}
    attempts=[]
    def guard(event, arguments):
        if event=='open' and arguments and isinstance(arguments[0],(str,bytes)):
            name=Path(str(arguments[0])).name
            if name in blocked:
                attempts.append(name)
                raise PermissionError('视觉进程禁止读取场景/评分答案')
    sys.addaudithook(guard)
    engine=VisionEngine({'task':args.task,'selected_group':args.group},weights=args.weights,
                        confidence=.25,inference_hz=100,inference_size=960)
    telemetry={'relative_alt_m':10,'north_m':0,'east_m':0,'roll_rad':0,'pitch_rad':0,'yaw_rad':0}
    annotated=engine.process(frame,telemetry)
    detections=[dict(item) for item in engine.state.visible_detections]
    # New engine avoids history or retained boxes influencing the blank control.
    blank_engine=VisionEngine({'task':args.task,'selected_group':args.group},weights=args.weights,
                              confidence=.25,inference_hz=100,inference_size=960)
    blank_engine.process(np.zeros_like(frame),telemetry)
    blank=blank_engine.state.visible_detections
    observed={int(item['value']) for item in detections}
    expected=set(args.expect)
    result={'task':args.task,'source':engine.state.source,'model_loaded':engine.state.model_loaded,
            'model_sha256':engine.state.model_sha256,'frame_sha256':engine.state.frame_sha256,
            'inference_count':engine.state.inference_count,'detections':detections,
            'blank_detections':blank,'forbidden_read_attempts':attempts,
            'expected_values':sorted(expected),'observed_values':sorted(observed),
            'correct_content':observed==expected,
            'scope':'独立相机输入、真实 YOLO 分类与文件隔离；合成姿态只供画框投影，不评价定位精度',
            'passed':engine.state.source=='YOLO' and observed==expected and not blank and not attempts}
    args.output.mkdir(parents=True,exist_ok=True)
    cv2.imwrite(str(args.output/f'task{args.task}-yolo.png'),annotated)
    (args.output/f'task{args.task}-isolation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
