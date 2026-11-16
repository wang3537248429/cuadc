"""固定场地配置；任务航线与内容随机种子均不改写场地。"""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
import os
from pathlib import Path

from .config import ROOT
from .field_layout import FieldLayout


@dataclass(frozen=True)
class Arena:
    layout: FieldLayout
    centers: dict[str, tuple[float, float]]
    wells: dict[str, dict]
    layout_seed: int = 2025


def load_arena(root: Path = ROOT) -> Arena:
    configured=os.environ.get('CUADC_ARENA_CONFIG')
    local=root/'config/arena.local.json'
    path=Path(configured).expanduser() if configured else local if local.exists() else root/'config/arena.json'
    if path.exists():
        data=json.loads(path.read_text(encoding='utf-8'))
    elif configured:
        raise FileNotFoundError('指定的固定场地配置不存在')
    else:
        data={}
    layout=FieldLayout(**data.get('field_layout',{}))
    centers={name:tuple(map(float,point)) for name,point in
             data.get('group_centers_ne',{'A':[200,0],'B':[-200,0]}).items()}
    if set(centers)!={'A','B'} or any(len(p)!=2 or not all(map(math.isfinite,p)) for p in centers.values()):
        raise ValueError('固定场地必须提供 A、B 两区的有限东北坐标')
    layout=replace(layout,group_centers_ne=centers)
    wells=data.get('fixed_wells',{})
    if wells and set(wells)!={f'{g}-{i:02d}' for g in ('A','B') for i in range(1,5)}:
        raise ValueError('固定井位必须完整提供两区各四个井')
    for well in wells.values():
        if not all(math.isfinite(float(well[k])) for k in
                   ('north_m','east_m','rotation_deg','arrow_rotation_deg')):
            raise ValueError('固定井位参数必须为有限数值')
    return Arena(layout,centers,wells,int(data.get('layout_seed',2025)))
