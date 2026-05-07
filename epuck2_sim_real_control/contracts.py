from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class VelocityCommand:
    linear: float
    angular: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class MotionStatus:
    timestamp: float
    source: str
    pose: Pose2D
    linear_velocity: float
    angular_velocity: float
    left_wheel_velocity: float
    right_wheel_velocity: float
    battery_voltage: float | None = None
    controller_name: str | None = None
    state_estimation_error: dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            'timestamp': float(self.timestamp),
            'source': self.source,
            'pose': self.pose.to_dict(),
            'linear_velocity': float(self.linear_velocity),
            'angular_velocity': float(self.angular_velocity),
            'left_wheel_velocity': float(self.left_wheel_velocity),
            'right_wheel_velocity': float(self.right_wheel_velocity),
            'battery_voltage': None if self.battery_voltage is None else float(self.battery_voltage),
            'controller_name': self.controller_name,
            'state_estimation_error': dict(self.state_estimation_error),
        }


@dataclass(frozen=True)
class CameraMeasurement:
    timestamp: float
    pose: Pose2D
    tracking_confidence: float

    def to_dict(self) -> dict[str, Any]:
        return {
            'timestamp': float(self.timestamp),
            'pose': self.pose.to_dict(),
            'tracking_confidence': float(self.tracking_confidence),
        }


@dataclass(frozen=True)
class DatasetSample:
    run_id: str
    step_index: int
    command: VelocityCommand
    status: MotionStatus

    def to_dict(self) -> dict[str, Any]:
        return {
            'run_id': self.run_id,
            'step_index': int(self.step_index),
            'command': self.command.to_dict(),
            'status': self.status.to_dict(),
        }
