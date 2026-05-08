from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json


@dataclass(frozen=True)
class FreeMoCapSession:
    session_root: Path
    video_paths: list[Path]
    frames_per_second: float

    def timestamp_for_frame(self, frame_index: int) -> float:
        if frame_index < 0:
            raise ValueError('frame_index must be non-negative')
        if self.frames_per_second <= 0.0:
            raise ValueError('frames_per_second must be positive')
        return float(frame_index) / float(self.frames_per_second)


def load_freemocap_session(session_root: str | Path) -> FreeMoCapSession:
    resolved_root = Path(session_root).expanduser().resolve()
    videos_dir = resolved_root / 'synchronized_videos'
    if not videos_dir.is_dir():
        raise FileNotFoundError(f'synchronized_videos not found under {resolved_root}')

    video_paths = sorted(path for path in videos_dir.iterdir() if path.is_file() and path.suffix.lower() in {'.mp4', '.mov', '.avi', '.mkv'})
    if not video_paths:
        raise FileNotFoundError(f'no video files found under {videos_dir}')

    metadata = _load_recording_parameters(resolved_root)
    frames_per_second = _extract_frames_per_second(metadata, resolved_root)

    return FreeMoCapSession(
        session_root=resolved_root,
        video_paths=video_paths,
        frames_per_second=frames_per_second,
    )


def _load_recording_parameters(session_root: Path) -> dict:
    candidate_paths = [
        session_root / 'recording_parameters.json',
        session_root / 'output_data' / 'recording_parameters.json',
    ]
    for parameters_path in candidate_paths:
        if parameters_path.is_file():
            payload = json.loads(parameters_path.read_text(encoding='utf-8'))
            if not isinstance(payload, dict):
                raise ValueError(f'recording_parameters.json must contain a JSON object under {parameters_path.parent}')
            return payload
    raise FileNotFoundError(f'recording_parameters.json not found under {session_root}')


def _extract_frames_per_second(metadata: dict, session_root: Path) -> float:
    post_processing = metadata.get('post_processing_parameters_model') or {}
    raw_value = metadata.get('frames_per_second', metadata.get('fps'))
    if raw_value is None and isinstance(post_processing, dict):
        raw_value = post_processing.get('framerate', post_processing.get('sampling_rate'))
    try:
        frames_per_second = float(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'frames_per_second missing or invalid in recording parameters for {session_root}') from exc
    if frames_per_second <= 0.0:
        raise ValueError(f'frames_per_second missing or invalid in recording parameters for {session_root}')
    return frames_per_second
