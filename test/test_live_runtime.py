from pathlib import Path
import json
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.camera_runtime import export_freemocap_session_to_camera_jsonl
from epuck2_sim_real_control.contracts import MotionStatus, Pose2D
from epuck2_sim_real_control.freemocap_session_loader import FreeMoCapSession
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


def test_build_runtime_from_exported_freemocap_jsonl_fuses_camera_measurements(tmp_path: Path):
    session = FreeMoCapSession(
        session_root=tmp_path / 'freemocap_session',
        video_paths=[tmp_path / 'freemocap_session' / 'synchronized_videos' / 'cam0.mp4'],
        frames_per_second=2.0,
    )
    frames = []
    positions = [(0.0, 0.0), (0.1, 0.0)]
    image_centers = [(30, 30), (35, 30)]
    for center_x, center_y in image_centers:
        frame = np.zeros((80, 80), dtype=float)
        yy, xx = np.indices(frame.shape)
        body = (xx - center_x) ** 2 + (yy - center_y) ** 2 <= 10 ** 2
        frame[body] = 0.65
        frame[center_y, center_x + 8 : center_x + 11] = 1.0
        frames.append(frame)

    camera_jsonl = export_freemocap_session_to_camera_jsonl(
        session=session,
        output_path=tmp_path / 'exported_camera.jsonl',
        pixels_to_world=0.02,
        world_origin_px=(30.0, 30.0),
        invert_image_y=False,
        frame_source=frames,
    )

    runtime = build_runtime_from_sources(
        manifest=SessionManifest(mode='real', dataset_root=str(tmp_path)),
        status_source=[_status(0, 'real'), _status(1, 'real')],
        camera_source=camera_jsonl,
    )

    result = run_live_episode(runtime=runtime, run_dir=tmp_path / 'real_from_export')
    samples = [json.loads(line) for line in (tmp_path / 'real_from_export' / 'samples.jsonl').read_text().splitlines() if line.strip()]

    assert result['sample_count'] == 2
    assert samples[0]['status']['source'] == 'real_camera_fused'
    assert samples[1]['status']['source'] == 'real_camera_fused'
    assert abs(samples[0]['status']['pose']['x'] - positions[0][0]) < 0.03
    assert abs(samples[1]['status']['pose']['x'] - positions[1][0]) < 0.03
