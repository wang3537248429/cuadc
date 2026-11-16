#!/usr/bin/env python3
"""生成混凝土材质；只参考色调和结构，不复制实拍像素。"""
from pathlib import Path
import cv2
import numpy as np


def main():
    rng=np.random.default_rng(261016)
    size=512
    shade=np.full((size,size),165,np.float32)
    shade+=cv2.resize(rng.normal(0,7,(24,24)).astype(np.float32),(size,size),interpolation=cv2.INTER_CUBIC)
    shade+=rng.normal(0,2,(size,size))
    for y in range(0,size,16):
        shade[y:y+1]-=rng.uniform(2,5)
    image=np.clip(shade,0,255).astype(np.uint8)
    image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
    output=Path(__file__).resolve().parents[1]/'models/cuadc_world_assets/materials/textures/concrete_synthetic.jpg'
    cv2.imwrite(str(output),image,[cv2.IMWRITE_JPEG_QUALITY,90])


if __name__=='__main__':main()
