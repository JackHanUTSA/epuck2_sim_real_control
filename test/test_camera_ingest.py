from pathlib import Path
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.camera_ingest import camera_measurement_from_dict, load_camera_measurements_jsonl


def test_camera_measurement_from_dict_parses_pose_and_confidence():
    measurement = camera_measurement_from_dict(
        {
            'timestamp': 12.5,
            'pose': {'x': 1.2, 'y': -0.4, 'theta': 0.9},
            'tracking_confidence': 0.85,
        }
    )

    assert measurement.timestamp == 12.5
    assert measurement.pose.x == 1.2
    assert measurement.pose.theta == 0.9
    assert measurement.tracking_confidence == 0.85


def test_load_camera_measurements_jsonl_filters_low_confidence(tmp_path: Path):
    path = tmp_path / 'camera.jsonl'
    path.write_text(
        '\n'.join(
            [
                json.dumps({'timestamp': 1.0, 'pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0}, 'tracking_confidence': 0.95}),
                json.dumps({'timestamp': 2.0, 'pose': {'x': 1.0, 'y': 1.0, 'theta': 0.2}, 'tracking_confidence': 0.4}),
            ]
        )
        + '\n'
    )

    measurements = load_camera_measurements_jsonl(path, min_confidence=0.5)

    assert len(measurements) == 1
    assert measurements[0].timestamp == 1.0
