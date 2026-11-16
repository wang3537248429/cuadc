import queue

import pytest

from cuadc_onboard.camera import LatestFrameCamera
from cuadc_onboard.frame_timing import align_to_frame


def test_capture_continues_when_consumer_is_busy_and_returns_latest():
    frames=queue.Queue()
    class Camera:
        def read(self):
            value=frames.get()
            return value is not None,value
        def release(self):frames.put(None)
    camera=LatestFrameCamera(Camera())
    try:
        frames.put(0)
        assert camera.read()==(True,0)
        for number in (1,2,3):frames.put(number)
        with camera.condition:
            assert camera.condition.wait_for(lambda:camera.sequence>=4,timeout=1)
        assert camera.read()==(True,3)
        assert camera.processing_skipped_frames==2
        assert camera.metrics()['camera_received_frames']==4
    finally:camera.release()
    assert not camera.reader.is_alive()


def test_frame_position_and_attitude_use_their_own_sample_times():
    telemetry={'sample_time_s':10,'attitude_sample_time_s':10.15,'camera_frame_time_s':10.2,
               'north_m':1,'east_m':2,'relative_alt_m':16,'velocity_north_mps':12.5,
               'velocity_east_mps':-2,'velocity_down_mps':1,'yaw_rad':.3,'yaw_rate_rps':.5}
    aligned=align_to_frame(telemetry)
    assert aligned['north_m']==pytest.approx(3.5)
    assert aligned['east_m']==pytest.approx(1.6)
    assert aligned['relative_alt_m']==pytest.approx(15.8)
    assert aligned['yaw_rad']==pytest.approx(.325)
    assert telemetry['north_m']==1
    assert aligned['camera_time_valid']


def test_stale_or_reset_camera_clock_cannot_create_ground_detections():
    assert not align_to_frame({'sample_time_s':10,'camera_frame_time_s':12})['camera_time_valid']
    assert not align_to_frame({'sample_time_s':100,'camera_frame_time_s':1})['camera_time_valid']
