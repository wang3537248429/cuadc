#!/usr/bin/env python3
"""直接录制原生 ROS 相机流；统计真实源时间戳、到帧和编码丢帧，不复制预览截图。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import sys

import numpy as np

APP_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(APP_ROOT/'src'))
from cuadc_sim.camera_packet import unpack_frame
from cuadc_sim.config import RUNTIME_DIR


def cadence(timestamps):
    values = np.asarray(timestamps, dtype=float)
    delta = np.diff(values)
    delta = delta[delta > 0]
    return {'fps':float(1/np.mean(delta)) if len(delta) else 0.,
            'p95_interval_s':float(np.quantile(delta,.95)) if len(delta) else None,
            'maximum_interval_s':float(np.max(delta)) if len(delta) else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--fps', type=float, default=30)
    parser.add_argument('--transport',choices=('local','jpeg','raw'),default='local',
                        help='本机用原子相机帧；jpeg/raw 可订阅 ROS 跨进程传输')
    parser.add_argument('--result',type=Path,help='看到实际起飞及完成状态后结束录制')
    args = parser.parse_args()
    if args.duration <= 0 or not 1 <= args.fps <= 120:raise ValueError('录制参数越界')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stop_requested = threading.Event()
    # A parent may request shutdown while ffmpeg is flushing. A signal must
    # not interrupt the final join and discard the measured frame statistics.
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, CompressedImage
    import cv2
    cv2.setNumThreads(1)
    node=None
    if args.transport!='local':
        rclpy.init()
        node = rclpy.create_node('cuadc_native_camera_recorder')
    frames = queue.Queue(maxsize=8)
    source_times, arrival_times = [], []
    encoder = None
    error = []
    counters = {'received':0, 'encoded':0, 'duplicate_source_timestamps':0, 'encoder_queue_drops':0}
    image_size = {}
    def encode():
        nonlocal encoder
        try:
            while True:
                item = frames.get()
                if item is None:break
                width, height, pixels = item
                if encoder is None:
                    encoder = subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo',
                        '-pixel_format','rgb24','-video_size',f'{width}x{height}',
                        '-framerate',str(args.fps),'-i','pipe:0','-an','-c:v','libx264',
                        '-preset','ultrafast','-threads','2','-crf','18','-pix_fmt','yuv420p',
                        '-movflags','+faststart',str(args.output)],stdin=subprocess.PIPE)
                encoder.stdin.write(pixels)
                counters['encoded'] += 1
        except Exception as exc:error.append(f'{type(exc).__name__}: {exc}')
        finally:
            if encoder is not None:
                encoder.stdin.close()
                if encoder.wait(timeout=30):error.append('ffmpeg 编码失败')
    worker = threading.Thread(target=encode, name='native-video-encoder', daemon=True)
    worker.start()
    def receive(message):
        timestamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if args.transport in ('local','jpeg'):
            decoded=cv2.imdecode(np.frombuffer(message.data,np.uint8),cv2.IMREAD_COLOR)
            if decoded is None:
                error.append('JPEG 相机帧损坏')
                return
            pixels=cv2.cvtColor(decoded,cv2.COLOR_BGR2RGB)
            height,width=pixels.shape[:2]
        else:
            width,height=message.width,message.height
            pixels = np.frombuffer(message.data,np.uint8).reshape(height,message.step)[:, :width*3]
            pixels = pixels.reshape(height,width,3)
            if message.encoding.lower() == 'bgr8':pixels = pixels[:,:,::-1]
        image_size.update(width=int(width),height=int(height))
        counters['received'] += 1
        if source_times and timestamp <= source_times[-1]:
            counters['duplicate_source_timestamps'] += 1
            return
        source_times.append(timestamp);arrival_times.append(time.monotonic())
        try:frames.put_nowait((width,height,pixels.tobytes()))
        except queue.Full:counters['encoder_queue_drops'] += 1
    if node is not None:
        recording_qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT) if args.transport=='jpeg' else QoSProfile(depth=30,reliability=ReliabilityPolicy.RELIABLE)
        subscription = node.create_subscription(CompressedImage if args.transport=='jpeg' else Image,
            '/cuadc/camera/compressed' if args.transport=='jpeg' else '/cuadc/camera/image',receive,recording_qos)
    started = time.monotonic()
    seen_flying=False
    stop_reason='duration'
    last_sequence=0
    sequence_gaps=0
    try:
        while time.monotonic()-started < args.duration and not error and not stop_requested.is_set():
            if args.transport=='local':
                try:
                    sequence,stamp,jpeg=unpack_frame((RUNTIME_DIR/'camera_frame.packet').read_bytes())
                except (OSError,ValueError):
                    time.sleep(.003)
                    continue
                if sequence<last_sequence:
                    error.append('相机场景代次变化；请在 READY 后开始录制')
                    break
                if sequence==last_sequence:
                    time.sleep(.003)
                    continue
                if last_sequence:sequence_gaps+=max(0,sequence-last_sequence-1)
                last_sequence=sequence
                from types import SimpleNamespace
                receive(SimpleNamespace(data=jpeg,header=SimpleNamespace(stamp=SimpleNamespace(
                    sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9)))))
            else:
                rclpy.spin_once(node,timeout_sec=.1)
            if args.result:
                try:status=json.loads(args.result.read_text()).get('status')
                except (OSError,json.JSONDecodeError):status=None
                if status in {'FLYING','SCAN_RETRY','ATTACK_ROUTE_ACTIVE','PAYLOAD_RELEASED','IMPACT_RECORDED'}:seen_flying=True
                if seen_flying and status=='COMPLETED':
                    stop_reason='mission_completed'
                    break
        if stop_requested.is_set() and stop_reason != 'mission_completed':
            stop_reason='interrupted'
    except KeyboardInterrupt:
        stop_reason='interrupted'
    finally:
        capture_finished = time.monotonic()
        while worker.is_alive():
            try:frames.put(None,timeout=.2);break
            except queue.Full:
                if error:break
        worker.join(timeout=35)
        if worker.is_alive():error.append('编码线程没有正常退出')
        if node is not None:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
    source = cadence(source_times);arrival = cadence(arrival_times)
    first_wait = arrival_times[0]-started if arrival_times else None
    final_age = capture_finished-arrival_times[-1] if arrival_times else None
    gaps = np.diff(source_times)
    missed = sum(max(0,round(float(gap)*args.fps)-1) for gap in gaps)
    report = {'source':'Gazebo 原生相机帧 / ROS 消息，非截图轮播','target_fps':args.fps,
              'transport':args.transport,'jpeg_quality':94 if args.transport!='raw' else None,
              'local_sequence_gaps':sequence_gaps,
              'stop_reason':stop_reason,
              'image_size':image_size,'encoding':{'codec':'H264','crf':18,'preset':'ultrafast'},
              'source_cadence':source,'wall_arrival_cadence':arrival,'counts':counters,
              'inferred_missing_source_frames':missed,'inserted_or_repeated_preview_frames':0,
              'first_source_time_s':source_times[0] if source_times else None,
              'last_source_time_s':source_times[-1] if source_times else None,
              'errors':error,'playback_fps':args.fps,
              'capture_elapsed_s':capture_finished-started,
              'initial_frame_wait_s':first_wait,'final_frame_age_s':final_age,
              'simulation_time_per_wall_time':(source_times[-1]-source_times[0])/(arrival_times[-1]-arrival_times[0]) if len(source_times)>1 else 0.,
              'passed':bool(source['fps']>=args.fps*.98 and arrival['fps']>=args.fps*.95
                            and counters['encoder_queue_drops']==0 and not error
                            and counters['duplicate_source_timestamps']==0
                            and counters['received']==counters['encoded']
                            and first_wait is not None and first_wait<=max(2,args.duration*.05)
                            and final_age is not None and final_age<=.5
                            and missed<=max(1,counters['received']*.01))}
    args.output.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False),flush=True)
    return int(not report['passed'])


if __name__ == '__main__':raise SystemExit(main())
