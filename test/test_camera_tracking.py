from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.camera_tracking import CameraTracker
from epuck2_sim_real_control.contracts import CameraMeasurement, MotionStatus, Pose2D


def _odometry_status() -> MotionStatus:
    return MotionStatus(
        timestamp=10.0,
        source='real',
        pose=Pose2D(x=1.0, y=2.0, theta=0.3),
        linear_velocity=0.1,
        angular_velocity=0.2,
        left_wheel_velocity=1.1,
        right_wheel_velocity=1.2,
    )


def test_camera_tracker_prefers_fresh_camera_measurements_for_position():
    tracker = CameraTracker(max_camera_age_sec=0.25)

    fused = tracker.fuse(
        _odometry_status(),
        CameraMeasurement(timestamp=10.1, pose=Pose2D(x=1.2, y=1.8, theta=0.28), tracking_confidence=0.9),
    )

    assert fused.source == 'real_camera_fused'
    assert fused.pose.x == 1.2
    assert fused.pose.y == 1.8
    assert fused.state_estimation_error['position_error'] > 0.0
    assert fused.state_estimation_error['camera_age_sec'] == 0.1


def test_camera_tracker_falls_back_to_odometry_when_camera_is_missing_or_stale():
    tracker = CameraTracker(max_camera_age_sec=0.25)

    missing = tracker.fuse(_odometry_status(), None)
    stale = tracker.fuse(
        _odometry_status(),
        CameraMeasurement(timestamp=9.0, pose=Pose2D(x=5.0, y=5.0, theta=1.0), tracking_confidence=0.9),
    )

    assert missing.source == 'real_odometry'
    assert missing.pose.x == 1.0
    assert stale.source == 'real_odometry'
    assert stale.pose.y == 2.0
