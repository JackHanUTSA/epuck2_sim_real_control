from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from epuck2_sim_real_control.adapters import RealRobotAdapter, WebotsSimAdapter
from epuck2_sim_real_control.camera_runtime import CameraObservationRecorder
from epuck2_sim_real_control.camera_tracking import CameraTracker
from epuck2_sim_real_control.contracts import CameraMeasurement, MotionStatus, Pose2D
from epuck2_sim_real_control.dataset_logging import DatasetWriter
from epuck2_sim_real_control.episode_runner import run_episode
from epuck2_sim_real_control.runtime_sources import iter_camera_measurements
from epuck2_sim_real_control.session_manifest import SessionManifest
from epuck2_sim_real_control.shared_controller import SharedPoseController


@dataclass
class LiveRuntime:
    manifest: SessionManifest
    adapter: object
    controller: SharedPoseController
    tracker: CameraTracker | None
    statuses: list[MotionStatus]
    camera_measurements: list[CameraMeasurement]


def build_runtime_from_sources(
    manifest: SessionManifest,
    status_source: Iterable[MotionStatus],
    camera_source: Iterable[dict] | str | Path | None = None,
) -> LiveRuntime:
    adapter = WebotsSimAdapter(manifest) if manifest.mode == 'sim' else RealRobotAdapter(manifest)
    controller = SharedPoseController(manifest)
    tracker = None if manifest.mode == 'sim' else CameraTracker(max_camera_age_sec=0.5)
    camera_measurements = list(iter_camera_measurements(camera_source or [], min_confidence=0.5))
    return LiveRuntime(
        manifest=manifest,
        adapter=adapter,
        controller=controller,
        tracker=tracker,
        statuses=list(status_source),
        camera_measurements=camera_measurements,
    )


def run_live_episode(runtime: LiveRuntime, run_dir: str | Path, goal_pose: Pose2D | None = None) -> dict:
    resolved_run_dir = Path(run_dir).expanduser().resolve()
    writer = DatasetWriter(resolved_run_dir, runtime.manifest, controller_name=runtime.manifest.controller_name)
    camera_path = CameraObservationRecorder(resolved_run_dir / 'camera_observations.jsonl').record(runtime.camera_measurements)
    result = run_episode(
        run_id=resolved_run_dir.name,
        statuses=runtime.statuses,
        goal_pose=goal_pose or Pose2D(x=1.0, y=0.0, theta=0.0),
        adapter=runtime.adapter,
        controller=runtime.controller,
        writer=writer,
        tracker=runtime.tracker,
        camera_measurements=runtime.camera_measurements,
    )
    return result | {'mode': runtime.manifest.mode, 'camera_log_path': str(camera_path)}
