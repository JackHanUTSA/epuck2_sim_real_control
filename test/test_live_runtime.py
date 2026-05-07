from pathlib import Path
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.contracts import MotionStatus, Pose2D
from epuck2_sim_real_control.live_runtime import build_runtime_from_sources, run_live_episode
from epuck2_sim_real_control.session_manifest import SessionManifest


def _status(step: int, source: str) -> MotionStatus:
    return MotionStatus(
        timestamp=float(step),
        source=source,
        pose=Pose2D(x=0.1 * step, y=0.0, theta=0.0),
        linear_velocity=0.05,
        angular_velocity=0.01,
        left_wheel_velocity=1.0,
        right_wheel_velocity=1.1,
    )


def test_build_runtime_from_sources_selects_webots_or_real_adapter():
    sim_runtime = build_runtime_from_sources(
        manifest=SessionManifest(mode='sim'),
        status_source=[_status(0, 'sim')],
    )
    real_runtime = build_runtime_from_sources(
        manifest=SessionManifest(mode='real'),
        status_source=[_status(0, 'real')],
    )

    assert sim_runtime.adapter.__class__.__name__ == 'WebotsSimAdapter'
    assert real_runtime.adapter.__class__.__name__ == 'RealRobotAdapter'


def test_run_live_episode_writes_samples_and_camera_log(tmp_path: Path):
    manifest = SessionManifest(mode='real', dataset_root=str(tmp_path))
    runtime = build_runtime_from_sources(
        manifest=manifest,
        status_source=[_status(0, 'real'), _status(1, 'real')],
        camera_source=[
            {'timestamp': 0.1, 'pose': {'x': 0.02, 'y': 0.0, 'theta': 0.0}, 'tracking_confidence': 0.9},
            {'timestamp': 1.1, 'pose': {'x': 0.12, 'y': 0.0, 'theta': 0.0}, 'tracking_confidence': 0.95},
        ],
    )

    result = run_live_episode(runtime=runtime, run_dir=tmp_path / 'real_live')

    samples = [json.loads(line) for line in (tmp_path / 'real_live' / 'samples.jsonl').read_text().splitlines() if line.strip()]
    camera_rows = [json.loads(line) for line in (tmp_path / 'real_live' / 'camera_observations.jsonl').read_text().splitlines() if line.strip()]

    assert result['sample_count'] == 2
    assert result['mode'] == 'real'
    assert samples[0]['status']['source'] == 'real_camera_fused'
    assert camera_rows[1]['tracking_confidence'] == 0.95
