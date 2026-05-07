from pathlib import Path
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.camera_runtime import CameraObservationRecorder
from epuck2_sim_real_control.contracts import CameraMeasurement, Pose2D


def test_camera_observation_recorder_writes_jsonl_measurements(tmp_path: Path):
    recorder = CameraObservationRecorder(tmp_path / 'camera_observations.jsonl')
    measurements = [
        CameraMeasurement(timestamp=1.0, pose=Pose2D(x=0.1, y=0.2, theta=0.3), tracking_confidence=0.9),
        CameraMeasurement(timestamp=2.0, pose=Pose2D(x=0.4, y=0.5, theta=0.6), tracking_confidence=0.8),
    ]

    saved_path = recorder.record(measurements)
    rows = [json.loads(line) for line in saved_path.read_text().splitlines() if line.strip()]

    assert rows[0]['pose']['x'] == 0.1
    assert rows[1]['tracking_confidence'] == 0.8
