#!/usr/bin/env python3
"""以原生 Gazebo 多视角棋盘格标定仿真相机；不代表实机镜头标定。"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from cuadc_sim.config import DOWN_CAMERA_HORIZONTAL_FOV_RAD, SIM_CAMERA_WIDTH, SIM_CAMERA_HEIGHT
SIZE = (SIM_CAMERA_WIDTH,SIM_CAMERA_HEIGHT)
PATTERN = (9,6)
SQUARE_M = .12


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'config/camera_calibration_sim.json')
    args=parser.parse_args()
    runtime=ROOT/'.runtime/calibration';runtime.mkdir(parents=True,exist_ok=True)
    board=np.full((700,1000,3),255,np.uint8)
    for y in range(7):
        for x in range(10):
            if (x+y)%2==0:board[y*100:(y+1)*100,x*100:(x+1)*100]=0
    cv2.imwrite(str(runtime/'board.png'),board)
    world=f'''<?xml version="1.0"?><sdf version="1.9"><world name="dataset">
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
      <scene><ambient>0.8 0.8 0.8 1</ambient><background>0.5 0.5 0.5 1</background><shadows>false</shadows></scene>
      <light name="sun" type="directional"><diffuse>0.5 0.5 0.5 1</diffuse><direction>0 0 -1</direction></light>
      <model name="board"><static>true</static><link name="board"><visual name="board"><geometry><plane><normal>0 0 1</normal><size>1.2 0.84</size></plane></geometry>
      <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{runtime/'board.png'}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr></material></visual></link></model>
      <model name="capture_camera"><static>true</static><pose>0 0 1.5 0 1.5707963268 0</pose><link name="camera">
      <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>10</update_rate><topic>/cuadc/calibration/rgb</topic>
      <camera><horizontal_fov>{DOWN_CAMERA_HORIZONTAL_FOV_RAD}</horizontal_fov><image><width>{SIZE[0]}</width><height>{SIZE[1]}</height><format>R8G8B8</format></image>
      <clip><near>0.05</near><far>20</far></clip><lens><type>gnomonical</type><scale_to_hfov>true</scale_to_hfov></lens></camera></sensor></link></model>
      </world></sdf>'''
    (runtime/'board.sdf').write_text(world)
    flags=subprocess.check_output(['pkg-config','--cflags','--libs','gz-transport12','gz-msgs9'],text=True)
    helper=runtime/'camera-pose'
    subprocess.run(['c++','-O2','-std=c++17',str(ROOT/'tools/vision_training/camera_pose.cc'),'-o',str(helper),*shlex.split(flags)],check=True)
    env=os.environ.copy();env.update(GZ_PARTITION='cuadc_calibration',GZ_IP='127.0.0.1',ROS_DOMAIN_ID='90')
    os.environ['ROS_DOMAIN_ID']='90'
    import rclpy
    from sensor_msgs.msg import Image
    rclpy.init();node=rclpy.create_node('cuadc_calibration')
    received={'frame':None,'count':0}
    def receive(msg):
        rgb=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        received.update(frame=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY),count=received['count']+1)
    subscription=node.create_subscription(Image,'/cuadc/calibration/rgb',receive,1)
    objects=[];images=[];processes=[];mover=None
    grid=np.zeros((PATTERN[0]*PATTERN[1],3),np.float32)
    grid[:,:2]=np.mgrid[0:PATTERN[0],0:PATTERN[1]].T.reshape(-1,2)*SQUARE_M
    try:
        with (runtime/'capture.log').open('w') as log:
            for command in (['gz','sim','-s','-r',str(runtime/'board.sdf')],['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge','/cuadc/calibration/rgb@sensor_msgs/msg/Image[gz.msgs.Image']):
                processes.append(subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            deadline=time.monotonic()+45
            while received['count']<3 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
            if received['frame'] is None:raise TimeoutError('未收到标定图像')
            mover=subprocess.Popen([str(helper)],env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
            rng=np.random.default_rng(20261018)
            for index in range(30):
                z=float(rng.uniform(1.0,1.8));angle=float(rng.uniform(-math.pi,math.pi));radius=float(rng.uniform(.05,.65))
                x,y=radius*math.cos(angle),radius*math.sin(angle)
                yaw=angle+math.pi;pitch=math.atan2(z,radius)
                qx=-math.sin(yaw/2)*math.sin(pitch/2);qy=math.cos(yaw/2)*math.sin(pitch/2)
                qz=math.sin(yaw/2)*math.cos(pitch/2);qw=math.cos(yaw/2)*math.cos(pitch/2)
                mover.stdin.write(' '.join(map(str,[x,y,z,qx,qy,qz,qw]))+'\n');mover.stdin.flush()
                if mover.stdout.readline().strip()!='ok':raise RuntimeError('标定相机移动失败')
                for _ in range(3):rclpy.spin_once(node,timeout_sec=.001)
                count=received['count'];deadline=time.monotonic()+5
                while received['count']<count+2 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
                gray=received['frame'].copy()
                ok,corners=cv2.findChessboardCornersSB(gray,PATTERN,cv2.CALIB_CB_NORMALIZE_IMAGE)
                if ok:
                    objects.append(grid.copy());images.append(corners)
                    cv2.imwrite(str(runtime/f'view-{index:02d}.png'),gray)
            if len(images)<15:raise ValueError(f'有效棋盘格不足：{len(images)}/30')
    finally:
        if mover is not None:mover.stdin.close();mover.wait(timeout=5)
        for process in processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
        for process in processes:
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    rms,k,dist,rvecs,tvecs=cv2.calibrateCamera(objects,images,SIZE,None,None,flags=cv2.CALIB_FIX_K3)
    per_view=[]
    for obj,points,rvec,tvec in zip(objects,images,rvecs,tvecs):
        projected,_=cv2.projectPoints(obj,rvec,tvec,k,dist)
        per_view.append(float(np.sqrt(np.mean(np.sum((projected-points)**2,axis=2)))))
    reference=SIZE[0]/(2*math.tan(DOWN_CAMERA_HORIZONTAL_FOV_RAD/2))
    passed=rms<.5 and abs(k[0,0]/reference-1)<.02 and abs(k[1,1]/reference-1)<.02
    report={'scope':'Gazebo 仿真相机，不能用于声称实机镜头已标定','method':'30 个原生 RGB 多视角棋盘格，经 OpenCV 亚像素角点标定',
            'width':SIZE[0],'height':SIZE[1],'views':len(images),'square_m':SQUARE_M,'pattern':PATTERN,
            'camera_matrix':k.tolist(),'distortion_k1_k2_p1_p2_k3':dist.ravel().tolist(),
            'rms_reprojection_px':float(rms),'per_view_rms_px':per_view,
            'horizontal_fov_deg':math.degrees(2*math.atan(SIZE[0]/(2*k[0,0]))),
            'passed':bool(passed),'physical_camera_calibrated':False}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
