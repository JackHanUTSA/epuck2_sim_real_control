from __future__ import annotations

from typing import Iterable

from epuck2_sim_real_control.camera_tracking import CameraTracker
from epuck2_sim_real_control.contracts import CameraMeasurement, MotionStatus, Pose2D
from epuck2_sim_real_control.dataset_logging import DatasetWriter, build_dataset_sample
from epuck2_sim_real_control.shared_controller import SharedPoseController


def run_episode(
    run_id: str,
    statuses: Iterable[MotionStatus],
    goal_pose: Pose2D,
    adapter,
    controller: SharedPoseController,
    writer: DatasetWriter,
    tracker: CameraTracker | None = None,
    camera_measurements: Iterable[CameraMeasurement] | None = None,
) -> dict:
    writer.write_metadata()
    fused_camera_samples = 0
    samples = []
    measurements = list(camera_measurements or [])
    status_list = list(statuses)

    for step_index, status in enumerate(status_list):
        effective_status = status
        if tracker is not None and measurements:
            camera_measurement = measurements[min(step_index, len(measurements) - 1)]
            effective_status = tracker.fuse(status, camera_measurement)
            if effective_status.source == 'real_camera_fused':
                fused_camera_samples += 1
        command = controller.compute_command(effective_status, goal_pose)
        actuation = adapter.command_to_actuation(command)
        samples.append(
            build_dataset_sample(
                run_id=run_id,
                step_index=step_index,
                command=command,
                status=effective_status,
                actuation=actuation,
            )
        )

    writer.append_samples(samples)
    return {
        'run_id': run_id,
        'sample_count': len(samples),
        'controller_name': writer.controller_name,
        'fused_camera_samples': fused_camera_samples,
    }
