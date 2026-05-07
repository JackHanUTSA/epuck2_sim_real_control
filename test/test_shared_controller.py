from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.contracts import MotionStatus, Pose2D
from epuck2_sim_real_control.session_manifest import SessionManifest
from epuck2_sim_real_control.shared_controller import SharedPoseController


def _status(x: float, y: float, theta: float = 0.0) -> MotionStatus:
    return MotionStatus(
        timestamp=0.0,
        source='sim',
        pose=Pose2D(x=x, y=y, theta=theta),
        linear_velocity=0.0,
        angular_velocity=0.0,
        left_wheel_velocity=0.0,
        right_wheel_velocity=0.0,
    )


def test_shared_controller_moves_forward_toward_goal():
    manifest = SessionManifest(max_linear_velocity=0.13, max_angular_velocity=4.5)
    controller = SharedPoseController(manifest)

    command = controller.compute_command(_status(0.0, 0.0, 0.0), Pose2D(x=1.0, y=0.0, theta=0.0))

    assert command.linear > 0.0
    assert command.angular == 0.0
    assert command.linear <= 0.13


def test_shared_controller_turns_toward_lateral_goal_and_stops_when_close():
    manifest = SessionManifest(goal_tolerance_m=0.05, max_linear_velocity=0.13, max_angular_velocity=4.5)
    controller = SharedPoseController(manifest)

    turn = controller.compute_command(_status(0.0, 0.0, 0.0), Pose2D(x=0.0, y=1.0, theta=0.0))
    stop = controller.compute_command(_status(0.01, 0.01, 0.0), Pose2D(x=0.0, y=0.0, theta=0.0))

    assert turn.angular > 0.0
    assert turn.angular <= 4.5
    assert stop.linear == 0.0
    assert stop.angular == 0.0
