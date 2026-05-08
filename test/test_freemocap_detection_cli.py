from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control import freemocap_detection_cli


def test_main_invokes_export_with_expected_arguments(monkeypatch, tmp_path: Path):
    session_root = tmp_path / 'session'
    session_root.mkdir()
    output_path = tmp_path / 'detections.jsonl'
    session_object = object()
    captured: dict[str, object] = {}

    def fake_load(path):
        captured['loaded_path'] = path
        return session_object

    monkeypatch.setattr(
        freemocap_detection_cli,
        'load_freemocap_session',
        fake_load,
    )

    def fake_export(**kwargs):
        captured.update(kwargs)
        return output_path

    monkeypatch.setattr(freemocap_detection_cli, 'export_freemocap_session_to_camera_jsonl', fake_export)

    exit_code = freemocap_detection_cli.main(
        [
            '--session-root',
            str(session_root),
            '--output-path',
            str(output_path),
            '--pixels-to-world',
            '0.02',
            '--world-origin-px',
            '320',
            '240',
            '--video-index',
            '1',
            '--min-confidence',
            '0.7',
            '--keep-image-y',
        ]
    )

    assert exit_code == 0
    assert captured['loaded_path'] == str(session_root)
    assert captured['session'] is session_object
    assert captured['output_path'] == output_path
    assert captured['pixels_to_world'] == 0.02
    assert captured['world_origin_px'] == (320.0, 240.0)
    assert captured['video_index'] == 1
    assert captured['min_confidence'] == 0.7
    assert captured['invert_image_y'] is False


def test_main_uses_default_output_path_when_not_provided(monkeypatch, tmp_path: Path):
    session_root = tmp_path / 'session'
    session_root.mkdir()
    session_object = object()
    captured: dict[str, object] = {}

    monkeypatch.setattr(freemocap_detection_cli, 'load_freemocap_session', lambda path: session_object)

    def fake_export(**kwargs):
        captured.update(kwargs)
        return kwargs['output_path']

    monkeypatch.setattr(freemocap_detection_cli, 'export_freemocap_session_to_camera_jsonl', fake_export)

    exit_code = freemocap_detection_cli.main(
        [
            '--session-root',
            str(session_root),
            '--pixels-to-world',
            '0.01',
        ]
    )

    assert exit_code == 0
    assert captured['output_path'] == session_root / 'camera_observations.jsonl'
    assert captured['world_origin_px'] is None
    assert captured['invert_image_y'] is True


def test_main_returns_nonzero_and_reports_load_or_export_errors(monkeypatch, tmp_path: Path, capsys):
    session_root = tmp_path / 'missing_session'
    monkeypatch.setattr(
        freemocap_detection_cli,
        'load_freemocap_session',
        lambda path: (_ for _ in ()).throw(FileNotFoundError('session missing')),
    )

    exit_code = freemocap_detection_cli.main(
        [
            '--session-root',
            str(session_root),
            '--pixels-to-world',
            '0.01',
        ]
    )

    captured = capsys.readouterr()
    assert exit_code == 1
    assert 'session missing' in captured.err


@pytest.mark.parametrize(
    ('flag', 'value'),
    [
        ('--pixels-to-world', '0'),
        ('--min-confidence', '1.5'),
        ('--video-index', '-1'),
    ],
)
def test_main_rejects_invalid_numeric_arguments(flag: str, value: str, tmp_path: Path):
    session_root = tmp_path / 'session'
    session_root.mkdir()
    args = ['--session-root', str(session_root), '--pixels-to-world', '0.01']

    if flag == '--pixels-to-world':
        args = ['--session-root', str(session_root), '--pixels-to-world', value]
    else:
        args.extend([flag, value])

    with pytest.raises(SystemExit):
        freemocap_detection_cli.main(args)
