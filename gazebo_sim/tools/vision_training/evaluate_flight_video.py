#!/usr/bin/env python3
"""回放仿真原生视频；先做纯像素推理，再读取场景真值离线评分。"""
from __future__ import annotations

import argparse
import bisect
from collections import Counter
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))


def visible_center(well, telemetry, width, height, hfov):
    """仅用于事后评分：把井心投影到相机，排除画面外与过小的靶板。"""
    import numpy as np
    roll, pitch, yaw = (float(telemetry[f'{axis}_rad']) for axis in ('roll', 'pitch', 'yaw'))
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rotation = np.array([[cp*cy, sr*sp*cy-cr*sy, cr*sp*cy+sr*sy],
                         [cp*sy, sr*sp*sy+cr*cy, cr*sp*sy-sr*cy],
                         [-sp, sr*cp, cr*cp]])
    ray = rotation.T @ np.array([well['north_m']-telemetry['north_m'],
                                  well['east_m']-telemetry['east_m'],
                                  telemetry['relative_alt_m']-.034])
    if ray[2] <= .1:
        return False
    focal = width / (2*math.tan(math.radians(hfov/2)))
    u, v = width/2+focal*ray[1]/ray[2], height/2-focal*ray[0]/ray[2]
    span = focal*.6/ray[2]
    return span >= 10 and span <= u < width-span and span <= v < height-span


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--camera-stats', type=Path, required=True)
    parser.add_argument('--telemetry', type=Path, required=True)
    parser.add_argument('--scenario', type=Path, required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--group', choices=('A', 'B'), default='A')
    parser.add_argument('--stride', type=int, default=10)
    parser.add_argument('--hfov', type=float, default=100)
    parser.add_argument('--start-sim', type=float, default=45)
    parser.add_argument('--end-sim', type=float, default=270)
    args = parser.parse_args()
    if args.stride < 1:
        raise ValueError('stride 必须为正数')
    import cv2
    import torch
    from cuadc_sim.frame_timing import align_to_frame
    from cuadc_sim.vision import VisionEngine
    torch.set_num_threads(3)
    cv2.setNumThreads(1)
    stats = json.loads(args.camera_stats.read_text())
    rows = sorted((json.loads(line) for line in args.telemetry.read_text().splitlines()),
                  key=lambda row: row['sample_time_s'])
    times = [row['sample_time_s'] for row in rows]
    engine = VisionEngine({'task': 1, 'selected_group': args.group}, weights=args.weights,
                          confidence=.25, inference_hz=1000, inference_size=320,
                          horizontal_fov_rad=math.radians(args.hfov))
    if not engine.state.model_loaded:
        raise RuntimeError('模型加载失败')
    guarded = False
    forbidden = []
    def audit(event, arguments):
        if guarded and event == 'open' and arguments and isinstance(arguments[0], (str, bytes)):
            if Path(str(arguments[0])).name in {'scenario.json', args.scenario.name, 'manifest.json', 'field_layout.json', 'latest.json'}:
                forbidden.append(Path(str(arguments[0])).name)
                raise PermissionError('像素推理阶段禁止读取答案')
    sys.addaudithook(audit)
    capture = cv2.VideoCapture(str(args.video))
    count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    step = (stats['last_source_time_s']-stats['first_source_time_s'])/max(1, count-1)
    predictions = []
    try:
        for index in range(count):
            ok = capture.grab()
            if not ok:
                raise RuntimeError(f'视频在第 {index} 帧中断')
            exposure = stats['first_source_time_s'] + index*step
            if index % args.stride or not args.start_sim <= exposure <= args.end_sim:
                continue
            ok, frame = capture.retrieve()
            if not ok:
                raise RuntimeError('视频帧解码失败')
            telemetry = dict(rows[max(0, bisect.bisect_right(times, exposure)-1)])
            telemetry['camera_frame_time_s'] = exposure
            engine.last_inference = 0
            engine.tracks.clear()
            guarded = True
            try:
                engine.process(frame, telemetry)
            finally:
                guarded = False
            predictions.append({'frame': index, 'sim_time_s': exposure,
                                'telemetry': align_to_frame(telemetry),
                                'width': frame.shape[1], 'height': frame.shape[0],
                                'detections': list(engine.state.visible_detections)})
    finally:
        capture.release()
    # 场景文件在所有推理结束后首次打开，井号和真值从未提供给模型。
    scenario = json.loads(args.scenario.read_text())
    wells = [well for group in scenario['groups'] if group['name'] == args.group
             for well in group['wells'] if well['occupied']]
    counts, confusion = Counter(), Counter()
    per_well = {well['id']: Counter() for well in wells}
    records = []
    for prediction in predictions:
        if not prediction['telemetry'].get('camera_time_valid', True):
            continue
        for well in wells:
            if not visible_center(well, prediction['telemetry'], prediction['width'], prediction['height'], args.hfov):
                continue
            nearby = [row for row in prediction['detections'] if math.hypot(
                row['north_m']-well['north_m'], row['east_m']-well['east_m']) < 4]
            observed = sorted({row['value'] for row in nearby})
            expected = well['value']
            counts['visible_readings'] += 1
            counts['correct'] += int(observed == [expected])
            counts['wrong'] += int(bool(observed) and observed != [expected])
            counts['missing'] += int(not observed)
            for value in observed:
                confusion[f'{expected}->{value}'] += 1
                per_well[well['id']][str(value)] += 1
            records.append({'frame': prediction['frame'], 'well': well['id'],
                            'expected': expected, 'observed': observed})
    result = {'scope': '历史失败飞行视频回归；不是新的独立测试或实机验收',
              'model_sha256': engine.state.model_sha256,
              'frames_inferred': len(predictions), 'counts': dict(counts),
              'confusion': dict(confusion), 'per_well_votes': per_well,
              'forbidden_reads': forbidden, 'scored_after_inference': True,
              'physical_validated': False, 'records': records}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'records'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
