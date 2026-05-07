from pathlib import Path
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.adapters import RealRobotAdapter, WebotsSimAdapter
from epuck2_sim_real_control.camera_tracking import CameraTracker
from epuck2_sim_real_control.contracts import CameraMeasurement, MotionStatus, Pose2D
from epuck2_sim_real_control.dataset_logging import DatasetWriter
from epuck2_sim_real_control.episode_runner import run_episode
from epuck2_sim_real_control.session_manifest import SessionManifest
from epuck2_sim_real_control.shared_controller import SharedPoseController


def _status(step: int, source: str = 'sim') -> MotionStatus:
    return MotionStatus(
        timestamp=float(step),
        source=source,
        pose=Pose2D(x=0.1 * step, y=0.0, theta=0.0),
        linear_velocity=0.05,
        angular_velocity=0.0,
        left_wheel_velocity=1.0,
        right_wheel_velocity=1.0,
    )


def test_run_episode_logs_shared_controller_samples_for_sim(tmp_path: Path):
    manifest = SessionManifest(mode='sim', dataset_root=str(tmp_path))
    writer = DatasetWriter(tmp_path / 'sim_run', manifest, controller_name='shared_webots_controller')
    controller = SharedPoseController(manifest)
    adapter = WebotsSimAdapter(manifest)

    result = run_episode(
        run_id='sim-001',
        statuses=[_status(0), _status(1), _status(2)],
        goal_pose=Pose2D(x=1.0, y=0.0, theta=0.0),
        adapter=adapter,
        controller=controller,
        writer=writer,
    )

    rows = [json.loads(line) for line in (tmp_path / 'sim_run' / 'samples.jsonl').read_text().splitlines() if line.strip()]
    assert result['sample_count'] == 3
    assert result['controller_name'] == 'shared_webots_controller'
    assert rows[0]['status']['source'] == 'sim'
    assert 'actuation' in rows[0]


def test_run_episode_can_fuse_real_status_with_camera_measurements(tmp_path: Path):
    manifest = SessionManifest(mode='real', dataset_root=str(tmp_path))
    writer = DatasetWriter(tmp_path / 'real_run', manifest, controller_name='shared_webots_controller')
    controller = SharedPoseController(manifest)
    adapter = RealRobotAdapter(manifest)
    tracker = CameraTracker(max_camera_age_sec=0.5)

    result = run_episode(
        run_id='real-001',
        statuses=[_status(0, source='real'), _status(1, source='real')],
        goal_pose=Pose2D(x=1.0, y=0.0, theta=0.0),
        adapter=adapter,
        controller=controller,
        writer=writer,
        tracker=tracker,
        camera_measurements=[
            CameraMeasurement(timestamp=0.1, pose=Pose2D(x=0.05, y=0.02, theta=0.0), tracking_confidence=0.9),
            CameraMeasurement(timestamp=1.1, pose=Pose2D(x=0.2, y=0.01, theta=0.0), tracking_confidence=0.9),
        ],
    )

    rows = [json.loads(line) for line in (tmp_path / 'real_run' / 'samples.jsonl').read_text().splitlines() if line.strip()]
    assert result['sample_count'] == 2
    assert result['fused_camera_samples'] == 2
    assert rows[0]['status']['source'] == 'real_camera_fused'
    assert rows[0]['status']['state_estimation_error']['tracking_confidence'] == 0.9
