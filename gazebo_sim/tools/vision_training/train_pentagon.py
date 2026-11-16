#!/usr/bin/env python3
"""训练与盲测五边井 YOLOv8n；测试集不参与早停或模型选择。"""
from __future__ import annotations

import argparse
import json
import gzip
import os
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
os.environ.setdefault('YOLO_CONFIG_DIR', str(ROOT/'.runtime/yolo-config'))
os.environ.setdefault('MPLCONFIGDIR', str(ROOT/'.runtime/matplotlib'))
os.environ.setdefault('OMP_NUM_THREADS','8')
os.environ.setdefault('MKL_NUM_THREADS','8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',type=int,choices=(1,2),required=True)
    parser.add_argument('--dataset',type=Path,default=ROOT/'datasets/pentagon_v1')
    parser.add_argument('--output',type=Path,default=ROOT/'.runtime/pentagon-training')
    parser.add_argument('--epochs',type=int,default=30)
    parser.add_argument('--size',type=int,default=320)
    parser.add_argument('--test-size',type=int,default=640)
    parser.add_argument('--batch',type=int,default=12)
    parser.add_argument('--initial',type=Path,help='从已有同类别模型继续训练；不传则从基础骨干开始')
    parser.add_argument('--freeze',type=int,default=10)
    parser.add_argument('--name',help='独立训练阶段名称')
    parser.add_argument('--learning-rate',type=float,default=.002)
    parser.add_argument('--threads',type=int,default=3)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--scale',type=float,default=0)
    parser.add_argument('--translate',type=float,default=0)
    parser.add_argument('--degrees',type=float,default=0,
                        help='训练旋转增强角度范围；图形靶可用 180，规范化数字板通常为 0')
    parser.add_argument('--save-period',type=int,default=-1,help='候选检查点间隔；-1 仅保留 best/last')
    parser.add_argument('--sensor-compression',action='store_true',help='训练批次加入粗像素/JPEG退化；验证与测试保持原样')
    parser.add_argument('--cpu-priority',type=int,default=10)
    args=parser.parse_args()
    # Ultralytics treats a relative project as a child of runs/detect.
    # Resolve once so checkpoint discovery and the final test use that same directory.
    args.output=args.output.resolve()
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(args.threads)
    if not 0<=args.cpu_priority<=19:raise ValueError('cpu-priority must be within 0..19')
    os.nice(args.cpu_priority)
    trainer_options={}
    if args.sensor_compression:
        import cv2
        import numpy as np
        from ultralytics.models.yolo.detect import DetectionTrainer
        class SensorTrainer(DetectionTrainer):
            def preprocess_batch(self,batch):
                # Labels keep the same geometry. Compression occurs on coarse
                # source pixels before enlarging back to the model input size.
                for index,tensor in enumerate(batch['img']):
                    if np.random.random()>.7:continue
                    image=np.ascontiguousarray(tensor.permute(1,2,0).numpy()[:,:,::-1])
                    height,width=image.shape[:2]
                    ratio=float(np.random.uniform(.45,1.))
                    small=cv2.resize(image,None,fx=ratio,fy=ratio,interpolation=cv2.INTER_AREA)
                    ok,encoded=cv2.imencode('.jpg',small,[cv2.IMWRITE_JPEG_QUALITY,int(np.random.randint(60,96))])
                    if ok:small=cv2.imdecode(encoded,cv2.IMREAD_COLOR)
                    restored=cv2.resize(small,(width,height),interpolation=cv2.INTER_LINEAR)
                    batch['img'][index].copy_(torch.from_numpy(np.ascontiguousarray(restored[:,:,::-1].transpose(2,0,1))))
                return super().preprocess_batch(batch)
        trainer_options['trainer']=SensorTrainer
    base=ROOT/'models/vision/task2_sim_digits_v1.pt'
    archive=ROOT/'models/vision/yolov8n_pretraining.pt.gz'
    if archive.is_file() and not args.initial:
        base=ROOT/'.runtime/yolov8n-pretraining.pt'
        base.parent.mkdir(parents=True,exist_ok=True)
        with gzip.open(archive,'rb') as source,base.open('wb') as target:shutil.copyfileobj(source,target)
    model=YOLO(str(args.initial)) if args.initial else YOLO('yolov8n.yaml').load(str(base))
    name=args.name or f'task{args.task}'
    model.add_callback('on_train_start',lambda trainer:torch.set_num_threads(args.threads))
    data=(args.dataset/f'task{args.task}/data.yaml').resolve()
    model.train(data=str(data),epochs=args.epochs,imgsz=args.size,batch=args.batch,
                device='cpu',workers=0,project=str(args.output),name=name,
                exist_ok=True,pretrained=True,optimizer='AdamW',lr0=args.learning_rate,
                patience=10,seed=20261016+args.task,deterministic=True,
                mosaic=0.0,fliplr=0.0,flipud=0.0,degrees=args.degrees,
                scale=args.scale,translate=args.translate,erasing=0.0,close_mosaic=0,
                freeze=args.freeze,cache=False,plots=False,save_period=args.save_period,verbose=False,
                resume=args.resume,**trainer_options)
    best=YOLO(str(args.output/name/'weights/best.pt'))
    result=best.val(data=str(data),split='test',imgsz=args.test_size,batch=args.batch,
                    device='cpu',workers=0,plots=False,save_json=False,
                    project=str(args.output),name=f'{name}-blind-test')
    report={'task':args.task,'architecture':'YOLOv8n','dataset_split':'test','inference_size':args.test_size,
            'training_augmentation':{'scale':args.scale,'translate':args.translate,'degrees':args.degrees,'sensor_compression':args.sensor_compression},
            'metrics':{key:float(value) for key,value in result.results_dict.items()},
            'per_class_map50':list(map(float,result.box.ap50)),
            'class_names':best.names,'test_used_for_model_selection':False}
    (args.output/f'{name}-test.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print('[YOLO 盲测]',json.dumps(report['metrics']),flush=True)


if __name__=='__main__': main()
