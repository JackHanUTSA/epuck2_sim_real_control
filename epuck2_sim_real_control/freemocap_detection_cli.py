from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Sequence

from epuck2_sim_real_control.camera_runtime import export_freemocap_session_to_camera_jsonl
from epuck2_sim_real_control.freemocap_session_loader import load_freemocap_session


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Export FreeMoCap e-puck2 detections to CameraMeasurement JSONL.'
    )
    parser.add_argument('--session-root', required=True, help='Path to the FreeMoCap session directory.')
    parser.add_argument(
        '--output-path',
        help='Output JSONL path. Defaults to <session-root>/camera_observations.jsonl.',
    )
    parser.add_argument(
        '--pixels-to-world',
        required=True,
        type=float,
        help='Scale factor converting image pixels into world-frame distance units.',
    )
    parser.add_argument(
        '--world-origin-px',
        nargs=2,
        metavar=('X', 'Y'),
        type=float,
        help='Optional image-space origin used to convert detections into the world frame.',
    )
    parser.add_argument('--video-index', type=int, default=0, help='FreeMoCap synchronized video index to decode.')
    parser.add_argument(
        '--min-confidence',
        type=float,
        default=0.5,
        help='Minimum tracking confidence to keep a detection row.',
    )
    parser.add_argument(
        '--keep-image-y',
        action='store_true',
        help='Keep image Y increasing downward instead of inverting to a cartesian-style world frame.',
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    _validate_args(parser, args)

    session_root = Path(args.session_root).expanduser().resolve()
    output_path = Path(args.output_path).expanduser().resolve() if args.output_path else session_root / 'camera_observations.jsonl'
    world_origin_px = tuple(args.world_origin_px) if args.world_origin_px is not None else None

    try:
        session = load_freemocap_session(str(session_root))
        saved_path = export_freemocap_session_to_camera_jsonl(
            session=session,
            output_path=output_path,
            pixels_to_world=float(args.pixels_to_world),
            min_confidence=float(args.min_confidence),
            world_origin_px=world_origin_px,
            invert_image_y=not bool(args.keep_image_y),
            video_index=int(args.video_index),
        )
    except Exception as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1

    print(saved_path)
    return 0


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if float(args.pixels_to_world) <= 0.0:
        parser.error('--pixels-to-world must be positive')
    if not 0.0 <= float(args.min_confidence) <= 1.0:
        parser.error('--min-confidence must be between 0.0 and 1.0')
    if int(args.video_index) < 0:
        parser.error('--video-index must be non-negative')


if __name__ == '__main__':
    raise SystemExit(main())
