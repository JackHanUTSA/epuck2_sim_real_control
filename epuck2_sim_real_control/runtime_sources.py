from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

from epuck2_sim_real_control.camera_ingest import camera_measurement_from_dict
from epuck2_sim_real_control.contracts import MotionStatus


def iter_motion_statuses(rows: Iterable[dict], source: str) -> Iterable[MotionStatus]:
    from epuck2_sim_real_control.adapters import RealRobotAdapter, WebotsSimAdapter
    from epuck2_sim_real_control.session_manifest import SessionManifest

    adapter = WebotsSimAdapter(SessionManifest()) if source == 'sim' else RealRobotAdapter(SessionManifest(mode='real'))
    for row in rows:
        status = adapter.normalize_status(dict(row))
        yield MotionStatus(
            timestamp=status.timestamp,
            source=source,
            pose=status.pose,
            linear_velocity=status.linear_velocity,
            angular_velocity=status.angular_velocity,
            left_wheel_velocity=status.left_wheel_velocity,
            right_wheel_velocity=status.right_wheel_velocity,
            battery_voltage=status.battery_voltage,
            controller_name=status.controller_name,
            state_estimation_error=status.state_estimation_error,
        )


def iter_camera_measurements(source: str | Path | Iterable[dict], min_confidence: float = 0.0):
    if isinstance(source, (str, Path)):
        rows = [json.loads(line) for line in Path(source).expanduser().resolve().read_text().splitlines() if line.strip()]
    else:
        rows = list(source)
    for row in rows:
        measurement = camera_measurement_from_dict(dict(row))
        if measurement.tracking_confidence >= float(min_confidence):
            yield measurement
