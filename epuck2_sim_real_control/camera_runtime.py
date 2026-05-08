from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
from typing import Callable, Iterable

import numpy as np

from epuck2_sim_real_control.contracts import CameraMeasurement
from epuck2_sim_real_control.freemocap_epuck2_detector import detect_epuck2_measurement
from epuck2_sim_real_control.freemocap_session_loader import FreeMoCapSession


class CameraObservationRecorder:
    def __init__(self, output_path: str | Path):
        self.output_path = Path(output_path).expanduser().resolve()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

    def record(self, measurements: Iterable[CameraMeasurement]) -> Path:
        with self.output_path.open('w', encoding='utf-8') as handle:
            for measurement in measurements:
                handle.write(json.dumps(measurement.to_dict(), sort_keys=True) + '\n')
        return self.output_path


def export_freemocap_session_to_camera_jsonl(
    *,
    session: FreeMoCapSession,
    output_path: str | Path,
    pixels_to_world: float,
    min_confidence: float = 0.5,
    world_origin_px: tuple[float, float] | None = None,
    invert_image_y: bool = True,
    video_index: int = 0,
    detector: Callable[..., CameraMeasurement | None] = detect_epuck2_measurement,
    frame_source: Iterable[np.ndarray] | None = None,
) -> Path:
    if video_index < 0 or video_index >= len(session.video_paths):
        raise IndexError('video_index out of range for FreeMoCap session')

    frames = frame_source if frame_source is not None else _iter_video_frames(session.video_paths[video_index])

    def iter_measurements() -> Iterable[CameraMeasurement]:
        for frame_index, frame in enumerate(frames):
            measurement = detector(
                np.asarray(frame),
                timestamp=session.timestamp_for_frame(frame_index),
                pixels_to_world=pixels_to_world,
                min_confidence=min_confidence,
                world_origin_px=world_origin_px,
                invert_image_y=invert_image_y,
            )
            if measurement is not None:
                yield measurement

    return CameraObservationRecorder(output_path).record(iter_measurements())


def _iter_video_frames(video_path: Path) -> Iterable[np.ndarray]:
    if _ffmpeg_binaries_available():
        yield from _iter_ffmpeg_video_frames(video_path)
        return
    yield from _iter_opencv_video_frames(video_path)


def _ffmpeg_binaries_available() -> bool:
    return shutil.which('ffmpeg') is not None and shutil.which('ffprobe') is not None


def _iter_ffmpeg_video_frames(video_path: Path) -> Iterable[np.ndarray]:
    if not video_path.is_file():
        raise FileNotFoundError(f'unable to open video file: {video_path}')

    width, height = _probe_video_dimensions(video_path)
    frame_bytes = width * height * 3
    command = [
        'ffmpeg',
        '-v',
        'error',
        '-i',
        str(video_path),
        '-f',
        'rawvideo',
        '-pix_fmt',
        'rgb24',
        '-',
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    try:
        assert process.stdout is not None
        while True:
            chunk = process.stdout.read(frame_bytes)
            if not chunk:
                break
            if len(chunk) != frame_bytes:
                process.kill()
                process.wait()
                stderr = process.stderr.read().decode('utf-8', errors='replace') if process.stderr is not None else ''
                raise RuntimeError(f'incomplete raw frame while decoding {video_path}: {stderr.strip()}')
            frame = np.frombuffer(chunk, dtype=np.uint8).reshape((height, width, 3))
            yield frame
    finally:
        if process.stdout is not None:
            process.stdout.close()

    stderr_text = ''
    if process.stderr is not None:
        stderr_text = process.stderr.read().decode('utf-8', errors='replace').strip()
        process.stderr.close()
    return_code = process.wait()
    if return_code != 0:
        raise RuntimeError(f'ffmpeg failed while decoding {video_path}: {stderr_text or return_code}')


def _probe_video_dimensions(video_path: Path) -> tuple[int, int]:
    probe = subprocess.run(
        [
            'ffprobe',
            '-v',
            'error',
            '-select_streams',
            'v:0',
            '-show_entries',
            'stream=width,height',
            '-of',
            'json',
            str(video_path),
        ],
        capture_output=True,
        check=False,
        text=True,
    )
    if probe.returncode != 0:
        raise RuntimeError(f'ffprobe failed for {video_path}: {probe.stderr.strip() or probe.returncode}')
    payload = json.loads(probe.stdout or '{}')
    streams = payload.get('streams') or []
    if not streams:
        raise RuntimeError(f'ffprobe returned no video stream metadata for {video_path}')
    width = int(streams[0]['width'])
    height = int(streams[0]['height'])
    if width <= 0 or height <= 0:
        raise RuntimeError(f'invalid video dimensions reported for {video_path}')
    return width, height


def _iter_opencv_video_frames(video_path: Path) -> Iterable[np.ndarray]:
    try:
        import cv2  # type: ignore
    except ImportError as exc:  # pragma: no cover - depends on environment
        raise RuntimeError('Either ffmpeg/ffprobe or OpenCV is required to decode FreeMoCap video files when frame_source is not provided') from exc

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        capture.release()
        raise FileNotFoundError(f'unable to open video file: {video_path}')

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            yield frame
    finally:
        capture.release()
