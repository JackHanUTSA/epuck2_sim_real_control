from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

from epuck2_sim_real_control.contracts import DatasetSample, MotionStatus, VelocityCommand
from epuck2_sim_real_control.session_manifest import SessionManifest


class DatasetWriter:
    def __init__(self, run_dir: str | Path, manifest: SessionManifest, controller_name: str):
        self.run_dir = Path(run_dir).expanduser().resolve()
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.manifest = manifest
        self.controller_name = controller_name
        self.metadata_path = self.run_dir / 'metadata.json'
        self.samples_path = self.run_dir / 'samples.jsonl'

    def write_metadata(self) -> Path:
        payload = {
            'project_name': self.manifest.project_name,
            'robot_name': self.manifest.robot_name,
            'mode': self.manifest.mode,
            'sim_backend': self.manifest.sim_backend,
            'real_backend': self.manifest.real_backend,
            'camera_topic': self.manifest.camera_topic,
            'dataset_root': self.manifest.dataset_root,
            'controller_name': self.controller_name,
            'policy_topic': self.manifest.policy_topic,
            'observation_topic': self.manifest.observation_topic,
            'sample_rate_hz': float(self.manifest.sample_rate_hz),
        }
        self.metadata_path.write_text(json.dumps(payload, indent=2, sort_keys=True))
        return self.metadata_path

    def append_samples(self, samples: Iterable[DatasetSample]) -> Path:
        with self.samples_path.open('a', encoding='utf-8') as handle:
            for sample in samples:
                handle.write(json.dumps(sample.to_dict(), sort_keys=True) + '\n')
        return self.samples_path


def build_dataset_sample(run_id: str, step_index: int, command: VelocityCommand, status: MotionStatus) -> DatasetSample:
    return DatasetSample(run_id=run_id, step_index=step_index, command=command, status=status)
