#!/usr/bin/env python3
"""从视频核对相机取流参数，输出本地配置；不会伪造镜头标定。"""
from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess


def analyze(path: Path, horizontal_fov: float | None = None, *, screen_recording: bool = False) -> dict:
    import cv2
    import numpy as np
    if not path.is_file():
        raise FileNotFoundError('视频文件不存在')
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise ValueError('视频无法解码')
    width, height = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps, frames = float(capture.get(cv2.CAP_PROP_FPS)), int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    codec = None
    if shutil.which('ffprobe'):
        raw = subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries',
            'stream=width,height,avg_frame_rate,r_frame_rate,codec_name,pix_fmt','-of','json',str(path)])
        stream = json.loads(raw)['streams'][0]
        codec=stream.get('codec_name')
        try: fps=float(Fraction(stream['avg_frame_rate']))
        except (ValueError,ZeroDivisionError,KeyError): pass
    samples=[]
    try:
        for fraction in np.linspace(.05,.95,12):
            capture.set(cv2.CAP_PROP_POS_FRAMES,int(frames*fraction))
            ok,frame=capture.read()
            if not ok: continue
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            samples.append({'luminance':float(np.median(gray)),
                            'sharpness_laplacian':float(cv2.Laplacian(gray,cv2.CV_64F).var())})
    finally:
        capture.release()
    if horizontal_fov is not None and not 20<=horizontal_fov<=170:
        raise ValueError('水平视场角必须在 20..170 度之间')
    camera={'device':'auto','width':width,'height':height,'fps':round(fps),
            'calibration_confirmed':False,'fx_px':0.0,'fy_px':0.0,
            'cx_px':(width-1)/2,'cy_px':(height-1)/2}
    if horizontal_fov is not None:
        camera.update(horizontal_fov_deg=horizontal_fov,
                      fx_px=width/(2*math.tan(math.radians(horizontal_fov)/2)),
                      fy_px=width/(2*math.tan(math.radians(horizontal_fov)/2)))
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return {'video_sha256':digest.hexdigest(),'width':width,'height':height,'encoded_fps':fps,
            'codec':codec,'sample_count':len(samples),'frame_statistics':samples,'camera':camera,
            'intrinsics_source':'按用户给定 FOV 估算，未经标定' if horizontal_fov is not None else '缺少标定依据，焦距保持未设定',
            'screen_recording':screen_recording,
            'limitations':['视频帧率不等于相机曝光/传输延迟','普通飞行视频不能唯一确定焦距、畸变和相机安装误差'],
            'privacy':'不导出原始帧、GPS、设备或文件路径元数据'}


def main():
    import yaml
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video',type=Path)
    parser.add_argument('--horizontal-fov',type=float)
    parser.add_argument('--screen-recording',action='store_true',help='录屏/拍屏：保留既有相机取流尺寸，不把文件尺寸当镜头参数')
    parser.add_argument('--output',type=Path,default=root/'config/camera-video.local.yaml')
    args=parser.parse_args()
    report=analyze(args.video,args.horizontal_fov,screen_recording=args.screen_recording)
    base=yaml.safe_load((root/'config/onboard.yaml').read_text())
    if args.screen_recording:
        base['camera']['calibration_confirmed']=False
    else:
        base['camera'].update(report['camera'])
    base['release']['enabled']=False
    base['vision']['validation_confirmed']=False
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(yaml.safe_dump(base,allow_unicode=True,sort_keys=False))
    args.output.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(f"视频 {report['width']}×{report['height']}，编码帧率 {report['encoded_fps']:.3f} fps；配置已生成，镜头标定仍需确认")


if __name__=='__main__':main()
