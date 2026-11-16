from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from cuadc_sim.camera_detail import target_details
from cuadc_sim.config import (DOWN_CAMERA_HORIZONTAL_FOV_RAD, SIM_CAMERA_WIDTH,
                              SIM_CAMERA_HEIGHT, SIM_CAMERA_FPS)


def test_sensor_profile_and_payload_visibility_match_the_pixel_pipeline():
    root = ET.parse(Path(__file__).resolve().parents[1]/'models/cuadc_twin/model.sdf').getroot()
    camera = root.find('.//sensor[@name="down_camera"]/camera')
    assert int(camera.findtext('image/width')) == SIM_CAMERA_WIDTH
    assert int(camera.findtext('image/height')) == SIM_CAMERA_HEIGHT
    assert float(camera.findtext('horizontal_fov')) == pytest.approx(DOWN_CAMERA_HORIZONTAL_FOV_RAD)
    assert float(root.findtext('.//sensor[@name="down_camera"]/update_rate')) == SIM_CAMERA_FPS
    # Match the reference's readable panel detail on the actual low scan lanes.
    panel_pixels = SIM_CAMERA_WIDTH*.6/(2*math.tan(DOWN_CAMERA_HORIZONTAL_FOV_RAD/2)*13)
    assert panel_pixels >= 50
    mask = int(camera.findtext('visibility_mask'))
    payload = root.find('.//model[@name="payload"]')
    assert payload.find('.//collision[@name="bottle_collision"]') is not None
    for visual in payload.findall('.//visual'):
        flags = int(visual.findtext('visibility_flags', '4294967295'))
        assert mask & flags == 0
        assert 4294967295 & flags != 0


def test_detail_preview_uses_current_pixels_and_only_estimated_target_positions():
    image = np.zeros((720, 1280, 3), np.uint8)
    image[355:366, 635:646] = [20, 180, 60]
    state = {'north_m': 0, 'east_m': 0, 'relative_alt_m': 14}
    tracks = [{'north_m': 0, 'east_m': 0}]
    detail = target_details(image, state, tracks, math.radians(110))
    assert detail is not None
    assert np.linalg.norm(detail[120, 120].astype(float)-[20, 180, 60]) < 2
    assert target_details(image, state, [{'north_m': 1000, 'east_m': 1000}], math.radians(110)) is None
    stale = {**state, 'camera_frame_time_s': 10, 'sample_time_s': 1}
    assert target_details(image, stale, tracks, math.radians(110)) is None
