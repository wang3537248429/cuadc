"""本机相机帧传输：原始曝光时间、递增序号和同一帧 JPEG 原子发布。"""
from __future__ import annotations
import math
import struct

HEADER=struct.Struct('!4sQd')


def pack_frame(sequence: int, timestamp: float, jpeg: bytes) -> bytes:
    if sequence<1 or not math.isfinite(timestamp) or not jpeg.startswith(b'\xff\xd8'):
        raise ValueError('无效的相机帧')
    return HEADER.pack(b'CFR1',sequence,timestamp)+jpeg


def unpack_frame(data: bytes):
    if len(data)<=HEADER.size:
        raise ValueError('相机帧不完整')
    magic,sequence,timestamp=HEADER.unpack_from(data)
    if magic!=b'CFR1' or sequence<1 or not math.isfinite(timestamp):
        raise ValueError('相机帧头无效')
    jpeg=data[HEADER.size:]
    if not jpeg.startswith(b'\xff\xd8'):
        raise ValueError('相机帧不是 JPEG')
    return sequence,timestamp,jpeg
