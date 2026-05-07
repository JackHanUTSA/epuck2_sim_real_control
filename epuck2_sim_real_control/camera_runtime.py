from __future__ import annotations

from pathlib import Path
from typing import Iterable

from epuck2_sim_real_control.contracts import CameraMeasurement


class CameraObservationRecorder:
    def __init__(self, output_path: str | Path):
        self.output_path = Path(output_path).expanduser().resolve()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

    def record(self, measurements: Iterable[CameraMeasurement]) -> Path:
        with self.output_path.open('w', encoding='utf-8') as handle:
            for measurement in measurements:
                handle.write(__import__('json').dumps(measurement.to_dict(), sort_keys=True) + '\n')
        return self.output_path
