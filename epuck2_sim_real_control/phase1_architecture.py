from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from epuck2_sim_real_control.session_manifest import SessionManifest


@dataclass(frozen=True)
class Phase1Architecture:
    simulator: str
    controller_strategy: str
    real_robot_backend: str
    camera_state_estimation: str
    dataset_alignment_goal: str
    artifacts: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def build_phase1_architecture(manifest: SessionManifest) -> Phase1Architecture:
    return Phase1Architecture(
        simulator=manifest.sim_backend,
        controller_strategy='single shared controller across sim and real',
        real_robot_backend=manifest.real_backend,
        camera_state_estimation='overhead camera fused with robot odometry',
        dataset_alignment_goal='collect matched sim and real motion-status datasets with the same command contract',
        artifacts=[
            'metadata.json',
            'samples.jsonl',
            'camera trajectories',
            'sim-real comparison reports',
        ],
    )
