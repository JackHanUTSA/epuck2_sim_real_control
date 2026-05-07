from __future__ import annotations

import math
from dataclasses import replace

from epuck2_sim_real_control.contracts import CameraMeasurement, MotionStatus


class CameraTracker:
    def __init__(self, max_camera_age_sec: float = 0.25):
        self.max_camera_age_sec = float(max_camera_age_sec)

    def fuse(self, odometry_status: MotionStatus, camera_measurement: CameraMeasurement | None) -> MotionStatus:
        if camera_measurement is None:
            return replace(odometry_status, source='real_odometry')

        camera_age = round(abs(float(camera_measurement.timestamp) - float(odometry_status.timestamp)), 6)
        if camera_age > self.max_camera_age_sec:
            return replace(odometry_status, source='real_odometry')

        dx = float(camera_measurement.pose.x) - float(odometry_status.pose.x)
        dy = float(camera_measurement.pose.y) - float(odometry_status.pose.y)
        dtheta = float(camera_measurement.pose.theta) - float(odometry_status.pose.theta)
        error = {
            'position_error': math.hypot(dx, dy),
            'heading_error': abs(dtheta),
            'camera_age_sec': camera_age,
            'tracking_confidence': float(camera_measurement.tracking_confidence),
        }
        return replace(
            odometry_status,
            source='real_camera_fused',
            pose=camera_measurement.pose,
            state_estimation_error=error,
        )
