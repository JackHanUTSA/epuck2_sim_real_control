from pathlib import Path
import json
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.freemocap_session_loader import FreeMoCapSession, load_freemocap_session


def test_load_freemocap_session_discovers_synchronized_videos_and_metadata(tmp_path: Path):
    session_root = tmp_path / 'freemocap_session'
    videos_dir = session_root / 'synchronized_videos'
    videos_dir.mkdir(parents=True)
    (videos_dir / 'cam0.mp4').write_bytes(b'')
    (videos_dir / 'cam1.mp4').write_bytes(b'')
    (session_root / 'recording_parameters.json').write_text(json.dumps({'frames_per_second': 30.0}))

    session = load_freemocap_session(session_root)

    assert isinstance(session, FreeMoCapSession)
    assert session.session_root == session_root.resolve()
    assert [path.name for path in session.video_paths] == ['cam0.mp4', 'cam1.mp4']
    assert session.frames_per_second == 30.0


def test_load_freemocap_session_accepts_fps_fallback_key(tmp_path: Path):
    session_root = tmp_path / 'fps_fallback'
    videos_dir = session_root / 'synchronized_videos'
    videos_dir.mkdir(parents=True)
    (videos_dir / 'cam0.mp4').write_bytes(b'')
    (session_root / 'recording_parameters.json').write_text(json.dumps({'fps': 25.0}))

    session = load_freemocap_session(session_root)

    assert session.frames_per_second == 25.0


def test_freemocap_session_computes_frame_timestamps():
    session = FreeMoCapSession(
        session_root=Path('/tmp/freemocap'),
        video_paths=[Path('/tmp/freemocap/synchronized_videos/cam0.mp4')],
        frames_per_second=20.0,
    )

    assert session.timestamp_for_frame(0) == 0.0
    assert session.timestamp_for_frame(5) == 0.25


def test_load_freemocap_session_rejects_missing_video_directory(tmp_path: Path):
    session_root = tmp_path / 'missing_videos'
    session_root.mkdir()
    (session_root / 'recording_parameters.json').write_text(json.dumps({'frames_per_second': 30.0}))

    with pytest.raises(FileNotFoundError):
        load_freemocap_session(session_root)


def test_load_freemocap_session_rejects_missing_or_empty_metadata_inputs(tmp_path: Path):
    session_root = tmp_path / 'bad_metadata'
    videos_dir = session_root / 'synchronized_videos'
    videos_dir.mkdir(parents=True)
    (videos_dir / 'cam0.mp4').write_bytes(b'')

    with pytest.raises(FileNotFoundError):
        load_freemocap_session(session_root)

    (session_root / 'recording_parameters.json').write_text(json.dumps([]))
    with pytest.raises(ValueError):
        load_freemocap_session(session_root)

    (session_root / 'recording_parameters.json').write_text(json.dumps({'frames_per_second': None}))
    with pytest.raises(ValueError):
        load_freemocap_session(session_root)

    (session_root / 'recording_parameters.json').write_text(json.dumps({'fps': 'abc'}))
    with pytest.raises(ValueError):
        load_freemocap_session(session_root)


def test_load_freemocap_session_rejects_empty_video_directory(tmp_path: Path):
    session_root = tmp_path / 'empty_videos'
    videos_dir = session_root / 'synchronized_videos'
    videos_dir.mkdir(parents=True)
    (session_root / 'recording_parameters.json').write_text(json.dumps({'frames_per_second': 30.0}))

    with pytest.raises(FileNotFoundError):
        load_freemocap_session(session_root)


def test_freemocap_session_rejects_non_positive_frame_index_or_fps():
    session = FreeMoCapSession(
        session_root=Path('/tmp/freemocap'),
        video_paths=[Path('/tmp/freemocap/synchronized_videos/cam0.mp4')],
        frames_per_second=20.0,
    )

    with pytest.raises(ValueError):
        session.timestamp_for_frame(-1)

    broken_session = FreeMoCapSession(
        session_root=Path('/tmp/freemocap'),
        video_paths=[Path('/tmp/freemocap/synchronized_videos/cam0.mp4')],
        frames_per_second=0.0,
    )

    with pytest.raises(ValueError):
        broken_session.timestamp_for_frame(0)
