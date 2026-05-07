from pathlib import Path
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.runtime_sources import iter_camera_measurements, iter_motion_statuses


def test_iter_motion_statuses_normalizes_raw_dicts():
    rows = [
        {
            'timestamp': 1.0,
            'pose': {'x': 0.2, 'y': 0.3, 'theta': 0.4},
            'twist': {'linear': 0.05, 'angular': 0.01},
            'wheel': {'left': 1.0, 'right': 1.1},
        }
    ]

    statuses = list(iter_motion_statuses(rows, source='sim'))

    assert statuses[0].source == 'sim'
    assert statuses[0].pose.y == 0.3


def test_iter_camera_measurements_reads_jsonl_and_filters_confidence(tmp_path: Path):
    path = tmp_path / 'camera.jsonl'
    path.write_text(
        '\n'.join(
            [
                json.dumps({'timestamp': 1.0, 'pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0}, 'tracking_confidence': 0.9}),
                json.dumps({'timestamp': 2.0, 'pose': {'x': 1.0, 'y': 1.0, 'theta': 0.1}, 'tracking_confidence': 0.2}),
            ]
        )
        + '\n'
    )

    measurements = list(iter_camera_measurements(path, min_confidence=0.5))

    assert len(measurements) == 1
    assert measurements[0].pose.x == 0.0
