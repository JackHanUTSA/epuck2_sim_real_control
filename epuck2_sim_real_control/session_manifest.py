from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class SessionManifest:
    project_name: str = 'epuck2_sim_real_control'
    robot_name: str = 'epuck2'
    mode: str = 'sim'
    sim_backend: str = 'webots'
    real_backend: str = 'epuck2_driver'
    controller_name: str = 'shared_webots_controller'
    policy_topic: str = '/epuck2/cmd_vel'
    observation_topic: str = '/epuck2/odom'
    camera_topic: str = '/overhead_camera/pose2d'
    dataset_root: str = './datasets'
    sample_rate_hz: float = 20.0
    max_linear_velocity: float = 0.13
    max_angular_velocity: float = 4.5
    enable_real_robot: bool = False
    require_operator_confirm: bool = True
    notes: list[str] = field(default_factory=lambda: [
        'Start in Webots simulation first.',
        'Run the same controller in sim and real whenever possible.',
        'Log matched command/status datasets before sim-real adaptation.',
        'Use overhead-camera trajectories to improve real-state estimation.',
    ])

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


DEFAULT_MANIFEST = SessionManifest()


def load_manifest(path: str | Path) -> SessionManifest:
    resolved = Path(path).expanduser().resolve()
    data = yaml.safe_load(resolved.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError('session manifest must be a YAML mapping')
    merged = DEFAULT_MANIFEST.to_dict() | data
    return SessionManifest(**merged)


def save_manifest(path: str | Path, manifest: SessionManifest) -> Path:
    resolved = Path(path).expanduser().resolve()
    resolved.parent.mkdir(parents=True, exist_ok=True)
    resolved.write_text(yaml.safe_dump(manifest.to_dict(), sort_keys=False))
    return resolved
