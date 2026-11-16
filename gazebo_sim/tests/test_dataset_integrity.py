"""Validate delivered data and split isolation; no training or model download."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[1]/'datasets'


@pytest.mark.parametrize('dataset,task,count',[
    ('pentagon_v1',1,1008),('pentagon_v1',2,1008),('gazebo_v1',1,480),('gazebo_v1',2,1200),
    ('domain_v2',1,1272),('domain_v2',2,1320),
    ('flight_v3',1,1712),('flight_v3',2,2320),
    ('gazebo_flight_v4',1,576),('flight_v4',1,1344),
    ('gazebo_flight_v7',1,576),('flight_v7',1,1440),
    ('gazebo_flight_v10',1,288),('flight_v10',1,864),
    ('flight_digits_v11',2,2320),
])
def test_images_labels_and_seeds_have_valid_isolated_splits(dataset,task,count):
    directory=ROOT/dataset/f'task{task}'
    if not directory.exists():
        pytest.skip("数据集为本地可选输入，不随源码分发")
    manifest=json.loads((directory/'manifest.json').read_text())
    records=manifest.get('records',manifest.get('samples'))
    assert len(records)==count
    hashes={};seeds=set();counts=Counter()
    for item in records:
        split=item['split'];counts[split]+=1
        relative=item.get('image',f"images/{split}/{item.get('frame')}.jpg")
        path=directory/relative
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest==item.get('sha256',item.get('file_sha256'))
        if item.get('raw_image'):
            assert f"/{split}/" in item['raw_image']
            assert hashlib.sha256((directory/item['raw_image']).read_bytes()).hexdigest()==item['raw_sha256']
        assert digest not in hashes or hashes[digest]==split
        hashes[digest]=split
        if 'seed' in item:
            seed=item['seed']
            assert seed not in seeds
            seeds.add(seed)
        labels=directory/'labels'/split/path.with_suffix('.txt').name
        for row in labels.read_text().splitlines():
            if not row.strip():continue
            cls,*box=map(float,row.split())
            assert cls.is_integer() and 0<=cls<(12 if task==1 else 20)
            x,y,w,h=box
            assert w>0 and h>0
            assert -1e-6<=x-w/2<=x+w/2<=1.000001
            assert -1e-6<=y-h/2<=y+h/2<=1.000001
    assert all(counts[split]>0 for split in ('train','val','test'))


def test_domain_augmentation_never_moves_a_parent_into_another_split():
    for dataset in ('domain_v2','flight_v3','flight_digits_v11'):
        for task in (1,2):
            directory=ROOT/dataset/f'task{task}'
            if not directory.is_dir():
                continue
            samples=json.loads((directory/'manifest.json').read_text())['samples']
            for item in samples:
                assert f"/{item['split']}/" in item['parent']


@pytest.mark.parametrize('version',[4,7,10])
def test_native_flight_layouts_and_panel_headings_are_independent(version):
    if not (ROOT/f'gazebo_flight_v{version}/task1').exists():
        pytest.skip("数据集为本地可选输入，不随源码分发")
    manifest=json.loads((ROOT/f'gazebo_flight_v{version}/task1/manifest.json').read_text())
    assert manifest['split_by_layout'] and manifest['independent_panels'] and manifest['shadows']
    layouts={split:set() for split in ('train','val','test')}
    for sample in manifest['samples']:
        layouts[sample['split']].add(sample['layout_id'])
        assert sample['panel_heading_rad']!=sample['arrow_heading_rad']
        if version>=7:
            assert sample['panel_visible_fraction']>=.9
    assert not (layouts['train'] & layouts['val'] or layouts['train'] & layouts['test'] or layouts['val'] & layouts['test'])
    for sample in json.loads((ROOT/f'flight_v{version}/task1/manifest.json').read_text())['samples']:
        assert f"/{sample['split']}/" in sample['parent']
