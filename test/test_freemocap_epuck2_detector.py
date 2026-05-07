from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.freemocap_epuck2_detector import detect_epuck2_measurement


def test_detect_epuck2_measurement_finds_center_from_synthetic_body():
    frame = np.zeros((120, 120), dtype=float)
    yy, xx = np.indices(frame.shape)
    body = (xx - 60) ** 2 + (yy - 50) ** 2 <= 18 ** 2
    frame[body] = 0.7
    frame[50, 78] = 1.0

    measurement = detect_epuck2_measurement(
        frame,
        timestamp=1.5,
        pixels_to_world=0.01,
        world_origin_px=(0.0, 0.0),
        invert_image_y=False,
    )

    assert measurement is not None
    assert measurement.timestamp == 1.5
    assert abs(measurement.pose.x - 0.60) < 0.03
    assert abs(measurement.pose.y - 0.50) < 0.03


def test_detect_epuck2_measurement_estimates_heading_from_bright_front_cue():
    frame = np.zeros((120, 120), dtype=float)
    yy, xx = np.indices(frame.shape)
    body = (xx - 45) ** 2 + (yy - 70) ** 2 <= 16 ** 2
    frame[body] = 0.6
    frame[70, 58:64] = 1.0

    measurement = detect_epuck2_measurement(
        frame,
        timestamp=2.0,
        pixels_to_world=1.0,
        world_origin_px=(0.0, 0.0),
        invert_image_y=False,
    )

    assert measurement is not None
    assert -0.2 <= measurement.pose.theta <= 0.2
    assert measurement.tracking_confidence >= 0.5


def test_detect_epuck2_measurement_rejects_ambiguous_heading_without_front_cue():
    frame = np.zeros((100, 100), dtype=float)
    yy, xx = np.indices(frame.shape)
    body = (xx - 50) ** 2 + (yy - 50) ** 2 <= 15 ** 2
    frame[body] = 0.7

    measurement = detect_epuck2_measurement(
        frame,
        timestamp=0.0,
        pixels_to_world=1.0,
        min_confidence=0.5,
        world_origin_px=(0.0, 0.0),
        invert_image_y=False,
    )

    assert measurement is None


def test_detect_epuck2_measurement_rejects_low_confidence_frame():
    frame = np.zeros((100, 100), dtype=float)
    frame[10, 10] = 0.4

    measurement = detect_epuck2_measurement(
        frame,
        timestamp=0.0,
        pixels_to_world=1.0,
        min_confidence=0.5,
        world_origin_px=(0.0, 0.0),
        invert_image_y=False,
    )

    assert measurement is None


def test_detect_epuck2_measurement_uses_centered_world_frame_by_default():
    frame = np.zeros((120, 120), dtype=float)
    yy, xx = np.indices(frame.shape)
    body = (xx - 60) ** 2 + (yy - 50) ** 2 <= 18 ** 2
    frame[body] = 0.7
    frame[50, 78] = 1.0

    measurement = detect_epuck2_measurement(frame, timestamp=1.5, pixels_to_world=0.01)

    assert measurement is not None
    assert abs(measurement.pose.x - 0.005) < 0.03
    assert abs(measurement.pose.y - 0.095) < 0.03


def test_detect_epuck2_measurement_validates_public_arguments():
    frame = np.zeros((20, 20), dtype=float)
    frame[8:12, 8:12] = 1.0

    with pytest.raises(ValueError):
        detect_epuck2_measurement(frame, timestamp=0.0, pixels_to_world=0.0)
