from __future__ import annotations

import json
from pathlib import Path

from epuck2_sim_real_control.contracts import CameraMeasurement, Pose2D


def camera_measurement_from_dict(payload: dict) -> CameraMeasurement:
    pose = payload.get('pose') or {}
    return CameraMeasurement(
        timestamp=float(payload['timestamp']),
        pose=Pose2D(x=float(pose['x']), y=float(pose['y']), theta=float(pose['theta'])),
        tracking_confidence=float(payload.get('tracking_confidence', 0.0)),
    )


def load_camera_measurements_jsonl(path: str | Path, min_confidence: float = 0.0) -> list[CameraMeasurement]:
    resolved = Path(path).expanduser().resolve()
    measurements: list[CameraMeasurement] = []
    for line in resolved.read_text().splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        measurement = camera_measurement_from_dict(json.loads(stripped))
        if measurement.tracking_confidence >= float(min_confidence):
            measurements.append(measurement)
    return measurements
