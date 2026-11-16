#!/usr/bin/env python3
"""原生 Gazebo 相机采集；真值仅用于离线标签，不提供给运行时识别。"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shlex
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from cuadc_sim.scenario import Scenario, TargetGroup, Well, wells_to_sdf
from cuadc_sim.well_geometry import floor_mesh_obj, visible_panel_fraction
from pentagon_dataset import TASK1_NAMES, TASK2_NAMES

WIDTH, HEIGHT, HFOV = 640, 480, math.radians(100)
FOCAL = WIDTH / (2 * math.tan(HFOV / 2))


def camera_rotation(pitch, yaw):
    cp, sp, cy, sy = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, -sy, cy*sp], [sy*cp, cy, sy*sp], [-sp, 0, cp]])


def label_boxes(task, value, center, heading, position, pitch, yaw):
    """针孔投影到 RGB 像素；目标均在井中心，近垂直视角避免井壁遮挡。"""
    parts = [(value, -.3, .3)] if task == 1 else [(value//10, -.3, 0), (10+value%10, 0, .3)]
    c, s = math.cos(heading), math.sin(heading)
    r = np.array([[c, -s], [s, c]])
    labels = []
    for class_id, left, right in parts:
        xy = np.array([[left,-.3], [right,-.3], [right,.3], [left,.3]]) @ r.T + center
        points = np.column_stack([xy, np.full(4, .034)])
        local = (points - position) @ camera_rotation(pitch, yaw)
        uv = np.column_stack([WIDTH/2-FOCAL*local[:,1]/local[:,0], HEIGHT/2-FOCAL*local[:,2]/local[:,0]])
        lo, hi = uv.min(axis=0), uv.max(axis=0)
        if (lo < 1).any() or (hi >= [WIDTH-1,HEIGHT-1]).any():
            raise ValueError('靶板超出取景范围')
        box = np.r_[(lo+hi)/2/[WIDTH,HEIGHT], (hi-lo)/[WIDTH,HEIGHT]]
        labels.append(f"{class_id} " + ' '.join(f'{x:.7f}' for x in box))
    return labels


def projected_boards_match_pixels(frame, labels):
    """离线采集校验：投影框中须同时有白底与黑色印刷，拒绝旧姿态帧。"""
    height, width = frame.shape[:2]
    for row in labels:
        _, x, y, w, h = map(float, row.split())
        x1, y1 = round((x-w/2)*width), round((y-h/2)*height)
        x2, y2 = round((x+w/2)*width), round((y+h/2)*height)
        patch = frame[max(0,y1):min(height,y2), max(0,x1):min(width,x2)]
        if not patch.size:
            return False
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        neutral = hsv[:,:,1]<55
        white_level = float(np.percentile(hsv[:,:,2][neutral], 85)) if np.any(neutral) else 0
        white = neutral & (hsv[:,:,2]>max(90,white_level*.7))
        # Thin printed rotor lines are antialiased gray at flight scale.
        # Relative neutral contrast retains them without accepting flat concrete.
        ink = (hsv[:,:,1]<80) & (hsv[:,:,2]<min(180,white_level*.8))
        if np.mean(white)<.22 or np.mean(ink)<.01:
            return False
    return True


def main():
    global WIDTH, HEIGHT, FOCAL, HFOV
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'datasets/gazebo_v1')
    parser.add_argument('--seed', type=int, default=20261017)
    parser.add_argument('--task1-shots', type=int, default=30)
    parser.add_argument('--task2-shots', type=int, default=10)
    parser.add_argument('--task1-layouts',type=int,default=1)
    parser.add_argument('--split-layouts',action='store_true')
    parser.add_argument('--independent-panels',action='store_true')
    parser.add_argument('--min-altitude',type=float,default=1.7)
    parser.add_argument('--max-altitude',type=float,default=5.3)
    parser.add_argument('--tilt',type=float,default=.03)
    parser.add_argument('--spacing',type=float,default=7)
    parser.add_argument('--width',type=int,default=640)
    parser.add_argument('--height',type=int,default=480)
    parser.add_argument('--hfov',type=float,default=100)
    parser.add_argument('--offset-m',type=float,default=.16,
                        help='相机横向偏移范围；飞行课程应覆盖视场边缘的斜视')
    parser.add_argument('--shadows',action='store_true')
    parser.add_argument('--min-panel-visibility',type=float,default=.85)
    args = parser.parse_args()
    if args.output.exists():raise FileExistsError('采集输出必须是新目录，避免混入旧划分')
    if args.split_layouts and args.task1_layouts<3:raise ValueError('布局划分至少需要三个布局')
    if not 1<args.min_altitude<args.max_altitude<30:raise ValueError('相机高度越界')
    if min(args.width,args.height)<240 or args.offset_m<0:raise ValueError('相机尺寸或偏移越界')
    WIDTH, HEIGHT = args.width, args.height
    HFOV = math.radians(args.hfov)
    FOCAL = WIDTH / (2 * math.tan(HFOV / 2))
    rng = np.random.default_rng(args.seed)
    runtime = ROOT/'.runtime'
    runtime.mkdir(exist_ok=True)
    (runtime/'well_floor.obj').write_text(floor_mesh_obj())
    objects, groups = [], []
    for task, count in ((1,12), (2,100)):
        if (args.task1_shots if task==1 else args.task2_shots)<=0:continue
        wells_a, wells_b = [], []
        layouts=args.task1_layouts if task==1 else 1
        for object_index in range(count*layouts):
            layout,value=divmod(object_index,count)
            # Two copies cover red/blue independently of target class.
            for color in ('A','B'):
                x, y = (object_index%10)*args.spacing + (12*args.spacing if color=='B' else 0), (object_index//10)*args.spacing + task*100
                angle = float(rng.uniform(0,2*math.pi))
                board_angle=float(rng.uniform(0,2*math.pi)) if task==1 and args.independent_panels else angle
                name = TASK1_NAMES[value] if task==1 else f'number-{value:02d}'
                asset = f'assets/targets/reference/task{task}/{name}.png'
                well = Well(f't{task}_{value}_{color}_layout{layout}',color,True,x,-y,math.degrees(board_angle),math.degrees(angle),value,name,asset)
                (wells_a if color=='A' else wells_b).append(well)
                objects.append((task,value,color,np.array([x,y]),board_angle,angle,layout))
        groups.extend([TargetGroup('A','blue',0,0,tuple(wells_a)), TargetGroup('B','red',0,0,tuple(wells_b))])
    scenario = Scenario('training-only',1,args.seed,'A',tuple(groups))
    centers=np.array([item[3] for item in objects])
    ground_center=(centers.min(axis=0)+centers.max(axis=0))/2
    ground_size=centers.max(axis=0)-centers.min(axis=0)+200
    # Video reference: concrete surroundings; generated material contains no real frames/OSD.
    world = f'''<?xml version="1.0"?><sdf version="1.9"><world name="dataset">
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
      <scene><ambient>0.48 0.51 0.56 1</ambient><background>0.7 0.8 0.9 1</background><shadows>{str(args.shadows).lower()}</shadows></scene>
      <light name="sun" type="directional"><diffuse>0.94 0.91 0.83 1</diffuse><direction>-0.42 0.22 -0.88</direction></light>
      <model name="ground"><static>true</static><pose>{ground_center[0]} {ground_center[1]} 0 0 0 0</pose><link name="ground"><visual name="ground">
      <geometry><plane><normal>0 0 1</normal><size>{ground_size[0]} {ground_size[1]}</size></plane></geometry>
      <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.68 0.68 0.66 1</diffuse></material></visual></link></model>
      {wells_to_sdf(scenario)}
      <model name="capture_camera"><static>true</static><pose>0 100 3 0 1.5707963268 0</pose><link name="camera">
      <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>10</update_rate><topic>/cuadc/dataset/rgb</topic>
      <camera><horizontal_fov>{HFOV}</horizontal_fov><image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
      <clip><near>0.05</near><far>40</far></clip></camera></sensor></link></model></world></sdf>'''
    world_path = runtime/'dataset.sdf'
    world_path.write_text(world)
    env = os.environ.copy()
    env.update(GZ_PARTITION='cuadc_dataset',GZ_IP='127.0.0.1',ROS_DOMAIN_ID='89',GZ_SIM_RESOURCE_PATH=str(ROOT/'models'))
    os.environ['ROS_DOMAIN_ID']='89'
    import rclpy
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('cuadc_dataset_capture')
    received = {'count':0,'frame':None,'stamp':0.}
    def receive(msg):
        frame=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        received.update(count=received['count']+1,frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR) if msg.encoding.lower()=='rgb8' else frame.copy(),
                        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    subscription=node.create_subscription(Image,'/cuadc/dataset/rgb',receive,1)
    processes=[]
    mover = None
    manifest={1:[],2:[]}
    try:
        with (runtime/'gazebo-dataset.log').open('w') as log:
            for command in (['gz','sim','-s','-r',str(world_path)], ['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge','/cuadc/dataset/rgb@sensor_msgs/msg/Image[gz.msgs.Image']):
                processes.append(subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            deadline=time.monotonic()+45
            while received['count']<3 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
            if received['frame'] is None:raise TimeoutError('Gazebo 未输出相机画面')
            helper=runtime/'camera-pose'
            flags=subprocess.check_output(['pkg-config','--cflags','--libs','gz-transport12','gz-msgs9'],text=True)
            subprocess.run(['c++','-O2','-std=c++17',str(Path(__file__).with_name('camera_pose.cc')),'-o',str(helper),*shlex.split(flags)],check=True)
            mover=subprocess.Popen([str(helper)],env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
            for index,(task,value,color,center,heading,arrow_heading,layout) in enumerate(objects):
                shots=args.task1_shots if task==1 else args.task2_shots
                for shot in range(shots):
                    seed=int(rng.integers(0,2**63)); view=np.random.default_rng(seed)
                    # Separate seed groups, never select a model on the test split.
                    held_out=max(1,int(shots*.15))
                    split='train' if shot<shots-2*held_out else 'val' if shot<shots-held_out else 'test'
                    if task==1 and args.split_layouts:
                        split='test' if layout==args.task1_layouts-1 else 'val' if layout==args.task1_layouts-2 else 'train'
                    for _ in range(100):
                        altitude=float(view.uniform(args.min_altitude,args.max_altitude))
                        xy=center+view.uniform(-args.offset_m,args.offset_m,2)
                        position=np.r_[xy,altitude]
                        pitch=math.pi/2+float(view.uniform(-args.tilt,args.tilt)); yaw=float(view.uniform(-math.pi,math.pi))
                        try:
                            labels=label_boxes(task,value,center,heading,position,pitch,yaw)
                        except ValueError:
                            continue
                        visibility=visible_panel_fraction([*(xy-center),altitude],heading,arrow_heading)
                        if visibility<args.min_panel_visibility:
                            continue
                        break
                    else:raise ValueError('无法生成完整可见的靶板，请减小偏移或倾角')
                    # Quaternion Ry(pitch), then Rz(yaw).
                    qx=-math.sin(yaw/2)*math.sin(pitch/2); qy=math.cos(yaw/2)*math.sin(pitch/2)
                    qz=math.sin(yaw/2)*math.cos(pitch/2); qw=math.cos(yaw/2)*math.cos(pitch/2)
                    mover.stdin.write(' '.join(map(str,[xy[0],xy[1],altitude,qx,qy,qz,qw]))+'\n');mover.stdin.flush()
                    if mover.stdout.readline().strip()!='ok':raise RuntimeError('相机移动服务失败')
                    # A service ACK precedes rendering. Drain multiple exposures,
                    # then verify projected geometry against the actual RGB frame.
                    # This offline guard is never part of runtime recognition.
                    count=received['count']; stamp=received['stamp']; deadline=time.monotonic()+8
                    while time.monotonic()<deadline:
                        rclpy.spin_once(node,timeout_sec=.2)
                        if (received['count']>=count+6 and received['stamp']>=stamp+.45
                                and projected_boards_match_pixels(received['frame'],labels)):
                            break
                    else:raise TimeoutError('新相机姿态与靶板投影未对齐，拒绝写入错误标签')
                    frame=received['frame'].copy()
                    base=args.output/f'task{task}'; name=f't{task}-{value:02d}-{color}-{shot:03d}'
                    if args.task1_layouts>1 and task==1:name+=f'-layout{layout}'
                    for folder in ('images','labels'):(base/folder/split).mkdir(parents=True,exist_ok=True)
                    image_path=base/'images'/split/f'{name}.jpg'
                    cv2.imwrite(str(image_path),frame,[cv2.IMWRITE_JPEG_QUALITY,92])
                    (base/'labels'/split/f'{name}.txt').write_text('\n'.join(labels)+'\n')
                    manifest[task].append({'image':f'images/{split}/{name}.jpg','split':split,'seed':hex(seed),'value':value,'color':color,'altitude_m':altitude,'camera_offset_m':(xy-center).tolist(),'panel_visible_fraction':visibility,'pitch_rad':pitch,'yaw_rad':yaw,'panel_heading_rad':heading,'arrow_heading_rad':arrow_heading,'layout_id':layout,'sha256':hashlib.sha256(image_path.read_bytes()).hexdigest()})
                if index%12==0:print(f'[Gazebo 数据集] {index+1}/{len(objects)} 个靶场，已采集 {sum(map(len,manifest.values()))} 帧',flush=True)
    finally:
        if mover is not None:
            mover.stdin.close();mover.wait(timeout=5)
        for process in processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
        for process in processes:
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for task,items in manifest.items():
            if not items:continue
            base=args.output/f'task{task}';base.mkdir(parents=True,exist_ok=True)
            (base/'manifest.json').write_text(json.dumps({'seed':args.seed,'renderer':'Gazebo Garden Ogre2 RGB camera','camera':{'width':WIDTH,'height':HEIGHT,'horizontal_fov_deg':math.degrees(HFOV),'offset_m':args.offset_m,'tilt_rad':args.tilt},'split_by_layout':args.split_layouts,'independent_panels':args.independent_panels,'shadows':args.shadows,'image_pose_verified':True,'samples':items},ensure_ascii=False,indent=2))
            (base/'data.yaml').write_text(yaml.safe_dump({'train':'images/train','val':'images/val','test':'images/test','names':TASK1_NAMES if task==1 else TASK2_NAMES},sort_keys=False))
    print('[Gazebo 数据集] 完成',flush=True)


if __name__=='__main__':main()
