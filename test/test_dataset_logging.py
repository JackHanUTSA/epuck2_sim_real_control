from pathlib import Path
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.contracts import MotionStatus, Pose2D, VelocityCommand
from epuck2_sim_real_control.dataset_logging import DatasetWriter, build_dataset_sample
from epuck2_sim_real_control.session_manifest import SessionManifest


def test_dataset_writer_persists_metadata_and_samples(tmp_path: Path):
    manifest = SessionManifest(mode='sim', dataset_root=str(tmp_path / 'datasets'))
    writer = DatasetWriter(tmp_path / 'run', manifest=manifest, controller_name='shared_webots_controller')
    sample = build_dataset_sample(
        run_id='sim-run-001',
        step_index=3,
        command=VelocityCommand(linear=0.1, angular=0.2),
        status=MotionStatus(
            timestamp=3.5,
            source='sim',
            pose=Pose2D(x=0.5, y=0.6, theta=0.1),
            linear_velocity=0.1,
            angular_velocity=0.2,
            left_wheel_velocity=1.0,
            right_wheel_velocity=1.1,
        ),
    )

    metadata_path = writer.write_metadata()
    samples_path = writer.append_samples([sample])

    metadata = json.loads(metadata_path.read_text())
    rows = [json.loads(line) for line in samples_path.read_text().splitlines() if line.strip()]

    assert metadata['controller_name'] == 'shared_webots_controller'
    assert metadata['mode'] == 'sim'
    assert rows[0]['run_id'] == 'sim-run-001'
    assert rows[0]['command']['linear'] == 0.1
    assert rows[0]['status']['pose']['x'] == 0.5
