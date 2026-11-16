"""仅从颜色像素提出候选区域；类别和最终框始终由 YOLO 给出。"""
from __future__ import annotations

from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class PixelView:
    image: np.ndarray
    origin_x: float = 0
    origin_y: float = 0
    scale: float = 1
    pad_x: float = 0
    pad_y: float = 0
    roi: bool = False
    inverse_affine: np.ndarray | None = None

    def to_original(self, box):
        x1,y1,x2,y2=box
        if self.inverse_affine is not None:
            corners=np.float32([[[x1,y1],[x2,y1],[x2,y2],[x1,y2]]])
            points=cv2.transform(corners,self.inverse_affine)[0]
            x1,y1=points.min(axis=0);x2,y2=points.max(axis=0)
        return (self.origin_x+(x1-self.pad_x)/self.scale,
                self.origin_y+(y1-self.pad_y)/self.scale,
                self.origin_x+(x2-self.pad_x)/self.scale,
                self.origin_y+(y2-self.pad_y)/self.scale)


def pixel_views(frame: np.ndarray, group: str, canonical: bool = False) -> list[PixelView]:
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    blue=cv2.inRange(hsv,np.array([85,75,35]),np.array([130,255,255]))
    red=cv2.inRange(hsv,np.array([0,75,35]),np.array([12,255,255]))
    red|=cv2.inRange(hsv,np.array([168,75,35]),np.array([179,255,255]))
    mask=blue if group=='A' else red if group=='B' else blue|red
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    height,width=frame.shape[:2]
    views=[]
    for contour in sorted(contours,key=cv2.contourArea,reverse=True):
        area=cv2.contourArea(contour)
        if area<12 or area>width*height*.35:continue
        x,y,w,h=cv2.boundingRect(contour)
        margin=max(4,round(max(w,h)*.2))
        x1,y1=max(0,x-margin),max(0,y-margin)
        x2,y2=min(width,x+w+margin),min(height,y+h+margin)
        crop=frame[y1:y2,x1:x2]
        scale=256/max(crop.shape[:2])
        resized=cv2.resize(crop,None,fx=scale,fy=scale,interpolation=cv2.INTER_LINEAR)
        rh,rw=resized.shape[:2];px,py=(640-rw)//2,(640-rh)//2
        canvas=np.full((640,640,3),114,np.uint8)
        canvas[py:py+rh,px:px+rw]=resized
        inverse=None
        if canonical:
            hull=cv2.convexHull(contour)
            polygon=cv2.approxPolyDP(hull,.035*cv2.arcLength(hull,True),True).reshape(-1,2)
            if len(polygon)==5:
                cosine=[]
                for index,point in enumerate(polygon):
                    a=polygon[index-1]-point;b=polygon[(index+1)%5]-point
                    cosine.append(float(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))))
                tip=polygon[int(np.argmax(cosine))]
                base=np.mean(np.delete(polygon,int(np.argmax(cosine)),axis=0),axis=0)
                dx,dy=tip-base
                angle=np.degrees(np.arctan2(dy,dx))+90
                affine=cv2.getRotationMatrix2D((320,320),float(angle),1)
                canvas=cv2.warpAffine(canvas,affine,(640,640),borderValue=(114,114,114))
                inverse=cv2.invertAffineTransform(affine)
        views.append(PixelView(canvas,x1,y1,scale,px,py,True,inverse))
        if len(views)==8:break
    return views or [PixelView(frame)]


def panel_pixels_support(frame: np.ndarray, box) -> bool:
    """Reject background boxes without the white substrate and dark printed ink."""
    height,width=frame.shape[:2]
    x1,y1,x2,y2=map(round,box)
    patch=frame[max(0,y1):min(height,y2),max(0,x1):min(width,x2)]
    if patch.size==0:return False
    hsv=cv2.cvtColor(patch,cv2.COLOR_BGR2HSV)
    white=(hsv[:,:,1]<55)&(hsv[:,:,2]>90)
    # At flight scale a thin black line is antialiased into gray pixels. An
    # absolute dark threshold discarded genuine high-confidence YOLO outputs.
    # Require neutral contrast relative to the local white panel instead.
    white_level=float(np.percentile(hsv[:,:,2][white],75)) if np.any(white) else 0
    ink=(hsv[:,:,1]<80)&(hsv[:,:,2]<min(180,white_level*.80))
    return float(np.mean(white))>=.22 and float(np.mean(ink))>=.01
