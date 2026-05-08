from pathlib import Path
import json
import sys
import types

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.camera_ingest import load_camera_measurements_jsonl
from epuck2_sim_real_control.camera_runtime import (
    CameraObservationRecorder,
    _iter_video_frames,
    export_freemocap_session_to_camera_jsonl,
)
from epuck2_sim_real_control.contracts import CameraMeasurement, Pose2D
from epuck2_sim_real_control.freemocap_session_loader import FreeMoCapSession


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


def test_export_freemocap_session_to_camera_jsonl_records_detected_frames(tmp_path: Path):
    output_path = tmp_path / 'camera_observations.jsonl'
    session = FreeMoCapSession(
        session_root=tmp_path / 'freemocap_session',
        video_paths=[tmp_path / 'freemocap_session' / 'synchronized_videos' / 'cam0.mp4'],
        frames_per_second=10.0,
    )

    frames = []
    detectable_centers = [(40, 35), (42, 35)]
    for center_x, center_y in detectable_centers:
        frame = np.zeros((90, 90), dtype=float)
        yy, xx = np.indices(frame.shape)
        body = (xx - center_x) ** 2 + (yy - center_y) ** 2 <= 12 ** 2
        frame[body] = 0.65
        frame[center_y, center_x + 10 : center_x + 14] = 1.0
        frames.append(frame)

    ambiguous = np.zeros((90, 90), dtype=float)
    yy, xx = np.indices(ambiguous.shape)
    body = (xx - 45) ** 2 + (yy - 50) ** 2 <= 12 ** 2
    ambiguous[body] = 0.65
    frames.append(ambiguous)

    saved_path = export_freemocap_session_to_camera_jsonl(
        session=session,
        output_path=output_path,
        pixels_to_world=0.02,
        world_origin_px=(0.0, 0.0),
        invert_image_y=False,
        frame_source=frames,
    )

    measurements = load_camera_measurements_jsonl(saved_path, min_confidence=0.5)

    assert saved_path == output_path.resolve()
    assert len(measurements) == 2
    assert measurements[0].timestamp == 0.0
    assert measurements[1].timestamp == 0.1
    assert measurements[0].pose.x < measurements[1].pose.x


def test_iter_video_frames_uses_opencv_fallback_when_ffmpeg_is_unavailable(monkeypatch, tmp_path: Path):
    class FakeCapture:
        def __init__(self):
            self.calls = 0
            self.released = False

        def isOpened(self):
            return True

        def read(self):
            self.calls += 1
            if self.calls == 1:
                return True, np.ones((4, 4), dtype=np.uint8)
            return False, None

        def release(self):
            self.released = True

    capture = FakeCapture()
    fake_cv2 = types.SimpleNamespace(VideoCapture=lambda _: capture)
    monkeypatch.setattr('epuck2_sim_real_control.camera_runtime._ffmpeg_binaries_available', lambda: False)
    monkeypatch.setitem(sys.modules, 'cv2', fake_cv2)

    frames = list(_iter_video_frames(tmp_path / 'fallback.mp4'))

    assert len(frames) == 1
    assert capture.released is True
