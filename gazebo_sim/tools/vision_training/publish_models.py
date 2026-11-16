#!/usr/bin/env python3
"""清除自训练检查点的机器元数据，再同步仿真/机载模型和校验和。"""
from __future__ import annotations

import argparse
import copy
import gzip
import hashlib
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[2]
FILENAMES={1:'task1_v4m_best.pt',2:'task2_sim_digits_v1.pt'}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',type=int,choices=(1,2),required=True)
    parser.add_argument('--checkpoint',type=Path,required=True,help='仅接受本项目自行训练且可信的检查点')
    parser.add_argument('--dataset-label',default='datasets/domain_v2')
    args=parser.parse_args()
    import torch
    from ultralytics import YOLO, __version__
    torch.set_num_threads(2)
    model=copy.deepcopy(YOLO(str(args.checkpoint)).model).cpu().half()
    label=Path(args.dataset_label)
    if label.is_absolute() or '..' in label.parts:raise ValueError('训练来源必须是公开仓库相对路径')
    safe_args={'task':'detect','data':f'{label.as_posix()}/task{args.task}/data.yaml','imgsz':320,'single_cls':False}
    model.args=safe_args
    for module in model.modules():
        for name in ('pt_path','save_dir','ckpt_path'):
            if hasattr(module,name):delattr(module,name)
    pending=ROOT/'.runtime/sanitized-model.pt'
    torch.save({'model':model,'ema':None,'optimizer':None,'epoch':-1,'train_args':safe_args,
                'version':__version__,'license':'AGPL-3.0','date':'2026-10-16'},pending)
    backup=ROOT.parent/'.private/pre-retrain'
    backup.mkdir(parents=True,exist_ok=True)
    for project in (ROOT,ROOT.parent/'onboard'):
        directory=project/'models/vision';name=FILENAMES[args.task]
        if args.task==1 and project==ROOT:
            base=directory/'yolov8n_pretraining.pt.gz'
            if not base.exists():shutil.copyfile(directory/'task2_sim_digits_v1.pt.gz',base)
        for suffix in ('','.gz'):
            old=directory/(name+suffix);saved=backup/project.name/(name+suffix)
            saved.parent.mkdir(parents=True,exist_ok=True)
            # A verified compressed original is sufficient for lossless restore.
            compressed_original=backup/project.name/(name+'.gz')
            if old.exists() and not saved.exists() and not compressed_original.exists():
                shutil.move(old,saved)
        shutil.copyfile(pending,directory/name)
        with (directory/(name+'.gz')).open('wb') as target:
            with gzip.GzipFile(filename='',mode='wb',fileobj=target,mtime=0,compresslevel=6) as compressed:
                with pending.open('rb') as source:shutil.copyfileobj(source,compressed)
        for manifest,paths in [('MODEL_SHA256SUMS',[directory/FILENAMES[t] for t in (1,2)]),
                               ('ARCHIVE_SHA256SUMS',sorted(directory.glob('*.pt.gz')))]:
            (directory/manifest).write_text(''.join(f'{sha256(path)}  {path.relative_to(project).as_posix()}\n' for path in paths))
    pending.unlink()
    print(f'任务 {args.task}：已清除训练路径并同步两个模块；保留旧权重的本地备份')


if __name__=='__main__':main()
