from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from epuck2_sim_real_control.contracts import MotionStatus, Pose2D, VelocityCommand
from epuck2_sim_real_control.session_manifest import SessionManifest


@dataclass
class _BaseAdapter:
    manifest: SessionManifest
    source_name: str

    def command_contract(self) -> dict[str, float | str]:
        return {
            'command_type': 'velocity_command',
            'policy_topic': self.manifest.policy_topic,
            'max_linear_velocity': float(self.manifest.max_linear_velocity),
            'max_angular_velocity': float(self.manifest.max_angular_velocity),
            'sample_rate_hz': float(self.manifest.sample_rate_hz),
        }

    def clamp_command(self, command: VelocityCommand) -> VelocityCommand:
        return VelocityCommand(
            linear=max(-self.manifest.max_linear_velocity, min(self.manifest.max_linear_velocity, float(command.linear))),
            angular=max(-self.manifest.max_angular_velocity, min(self.manifest.max_angular_velocity, float(command.angular))),
        )

    def normalize_status(self, raw_status: dict[str, Any]) -> MotionStatus:
        pose = raw_status.get('pose') or {}
        twist = raw_status.get('twist') or {}
        wheel = raw_status.get('wheel') or {}
        return MotionStatus(
            timestamp=float(raw_status.get('timestamp', 0.0)),
            source=self.source_name,
            pose=Pose2D(
                x=float(pose.get('x', 0.0)),
                y=float(pose.get('y', 0.0)),
                theta=float(pose.get('theta', 0.0)),
            ),
            linear_velocity=float(twist.get('linear', 0.0)),
            angular_velocity=float(twist.get('angular', 0.0)),
            left_wheel_velocity=float(wheel.get('left', 0.0)),
            right_wheel_velocity=float(wheel.get('right', 0.0)),
            battery_voltage=None if raw_status.get('battery_voltage') is None else float(raw_status['battery_voltage']),
            controller_name=self.manifest.controller_name,
        )


class WebotsSimAdapter(_BaseAdapter):
    def __init__(self, manifest: SessionManifest):
        super().__init__(manifest=manifest, source_name='sim')


class RealRobotAdapter(_BaseAdapter):
    def __init__(self, manifest: SessionManifest):
        super().__init__(manifest=manifest, source_name='real')
