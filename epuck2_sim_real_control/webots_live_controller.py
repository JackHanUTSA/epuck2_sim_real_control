from __future__ import annotations

import os
from datetime import UTC, datetime
from pathlib import Path

from epuck2_sim_real_control.adapters import WebotsSimAdapter
from epuck2_sim_real_control.contracts import Pose2D
from epuck2_sim_real_control.dataset_logging import DatasetWriter, build_dataset_sample
from epuck2_sim_real_control.session_manifest import DEFAULT_MANIFEST, SessionManifest, load_manifest
from epuck2_sim_real_control.shared_controller import SharedPoseController
from epuck2_sim_real_control.webots_connector import WebotsStatusReader, ensure_webots_controller_importable


class WebotsLiveController:
    def __init__(self, manifest: SessionManifest, robot) -> None:
        self.manifest = manifest
        self.robot = robot
        self.adapter = WebotsSimAdapter(manifest)
        self.reader = WebotsStatusReader(robot, manifest)
        self.controller = SharedPoseController(manifest)
        self.goal_pose = Pose2D(
            x=float(manifest.goal_x),
            y=float(manifest.goal_y),
            theta=float(manifest.goal_theta),
        )
        self.reader.configure_velocity_control()
        self.run_dir = _make_run_dir(manifest.dataset_root)
        self.writer = DatasetWriter(self.run_dir, manifest, controller_name=manifest.controller_name)

    def run(self, max_steps: int | None = None, goal_pose: Pose2D | None = None) -> dict:
        active_goal = goal_pose or self.goal_pose
        max_steps = int(self.manifest.max_steps if max_steps is None else max_steps)
        run_id = self.run_dir.name
        samples = []
        step_index = 0
        self.writer.write_metadata()

        try:
            while self.robot.step(int(self.robot.getBasicTimeStep())) != -1:
                status = self.reader.read_status()
                command = self.controller.compute_command(status, active_goal)
                actuation = self.adapter.command_to_actuation(command)
                self.reader.apply_wheel_velocities(
                    float(actuation['left_motor_velocity']),
                    float(actuation['right_motor_velocity']),
                )
                samples.append(
                    build_dataset_sample(
                        run_id=run_id,
                        step_index=step_index,
                        command=command,
                        status=status,
                        actuation=actuation,
                    )
                )
                step_index += 1
                if _goal_reached(status, active_goal, self.manifest):
                    break
                if max_steps > 0 and step_index >= max_steps:
                    break
        finally:
            self.reader.stop()
            self.writer.append_samples(samples)
        return {
            'run_id': run_id,
            'sample_count': len(samples),
            'controller_name': self.manifest.controller_name,
            'run_dir': str(self.run_dir),
        }


def load_controller_manifest(manifest_path: str | None = None) -> SessionManifest:
    chosen_path = manifest_path or os.environ.get('EPUCK2_MANIFEST_PATH', '').strip()
    if chosen_path:
        manifest = load_manifest(chosen_path)
    else:
        manifest = SessionManifest(**DEFAULT_MANIFEST.to_dict())

    dataset_root = os.environ.get('EPUCK2_DATASET_ROOT', '').strip()
    if dataset_root:
        manifest.dataset_root = dataset_root
    return manifest


def main(manifest_path: str | None = None, robot=None) -> dict:
    manifest = load_controller_manifest(manifest_path)
    if robot is None:
        controller = ensure_webots_controller_importable()
        robot = controller.Robot()
    return WebotsLiveController(manifest, robot).run()


def _make_run_dir(dataset_root: str) -> Path:
    timestamp = datetime.now(UTC).strftime('%Y%m%dT%H%M%S%fZ')
    run_dir = Path(dataset_root).expanduser().resolve() / f'webots_live_{timestamp}'
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def _goal_reached(status, goal_pose: Pose2D, manifest: SessionManifest) -> bool:
    dx = float(goal_pose.x) - float(status.pose.x)
    dy = float(goal_pose.y) - float(status.pose.y)
    distance_sq = (dx * dx) + (dy * dy)
    if distance_sq > float(manifest.goal_tolerance_m) ** 2:
        return False

    heading_error = SharedPoseController._wrap_angle(float(goal_pose.theta) - float(status.pose.theta))
    return abs(heading_error) <= float(manifest.goal_heading_tolerance_rad)
