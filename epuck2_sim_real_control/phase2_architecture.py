from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from epuck2_sim_real_control.session_manifest import SessionManifest


@dataclass(frozen=True)
class Phase2Architecture:
    simulator_runtime: str
    shared_controller: str
    real_robot_runtime: str
    dataset_pipeline: str
    camera_pipeline: str
    outputs: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def build_phase2_architecture(manifest: SessionManifest) -> Phase2Architecture:
    return Phase2Architecture(
        simulator_runtime=f'{manifest.sim_backend} differential-drive actuation adapter',
        shared_controller='goal-directed shared pose controller using one VelocityCommand contract',
        real_robot_runtime=f'{manifest.real_backend} cmd_vel adapter with matching command limits',
        dataset_pipeline='episode runner -> dataset samples -> metadata.json + samples.jsonl',
        camera_pipeline='JSONL camera ingestion -> CameraTracker fusion -> real state estimate',
        outputs=[
            'matched sim samples',
            'matched real samples',
            'camera-fused real samples',
            'actuation records for sim and real',
        ],
    )
