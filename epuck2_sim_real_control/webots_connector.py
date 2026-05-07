from __future__ import annotations

import importlib
import math
import os
import shutil
import sys
from pathlib import Path
from typing import Any

from epuck2_sim_real_control.adapters import WebotsSimAdapter
from epuck2_sim_real_control.contracts import MotionStatus
from epuck2_sim_real_control.session_manifest import SessionManifest


def discover_webots_home() -> Path | None:
    env_home = os.environ.get('WEBOTS_HOME')
    if env_home:
        candidate = Path(env_home).expanduser().resolve()
        if (candidate / 'lib' / 'controller' / 'python').exists():
            return candidate

    webots_bin = shutil.which('webots')
    if not webots_bin:
        return None
    candidate = Path(webots_bin).resolve().parent
    if (candidate / 'lib' / 'controller' / 'python').exists():
        return candidate
    return None


def ensure_webots_controller_importable():
    try:
        return importlib.import_module('controller')
    except Exception:
        home = discover_webots_home()
        if home is None:
            raise RuntimeError('Webots installation not found; cannot import controller module')
        os.environ.setdefault('WEBOTS_HOME', str(home))
        controller_path = home / 'lib' / 'controller' / 'python'
        if str(controller_path) not in sys.path:
            sys.path.insert(0, str(controller_path))
        return importlib.import_module('controller')


class WebotsStatusReader:
    def __init__(self, robot: Any, manifest: SessionManifest):
        self.robot = robot
        self.manifest = manifest
        self.adapter = WebotsSimAdapter(manifest)
        self.time_step_ms = int(robot.getBasicTimeStep())
        self.gps = robot.getDevice(manifest.gps_device_name)
        self.compass = robot.getDevice(manifest.compass_device_name)
        self.left_motor = robot.getDevice(manifest.left_motor_device_name)
        self.right_motor = robot.getDevice(manifest.right_motor_device_name)
        self.gps.enable(self.time_step_ms)
        self.compass.enable(self.time_step_ms)

    def read_raw_status(self) -> dict[str, Any]:
        gps_values = list(self.gps.getValues())
        compass_values = list(self.compass.getValues())
        left_velocity = float(self.left_motor.getVelocity())
        right_velocity = float(self.right_motor.getVelocity())
        theta = math.atan2(float(compass_values[2]), float(compass_values[0]))
        linear_velocity = 0.5 * float(self.manifest.wheel_radius_m) * (left_velocity + right_velocity)
        angular_velocity = float(self.manifest.wheel_radius_m) * (right_velocity - left_velocity) / float(self.manifest.axle_length_m)
        return {
            'timestamp': float(self.robot.getTime()),
            'pose': {
                'x': float(gps_values[0]),
                'y': float(gps_values[2]),
                'theta': theta,
            },
            'twist': {
                'linear': linear_velocity,
                'angular': angular_velocity,
            },
            'wheel': {
                'left': left_velocity,
                'right': right_velocity,
            },
        }

    def read_status(self) -> MotionStatus:
        return self.adapter.normalize_status(self.read_raw_status())

    def configure_velocity_control(self) -> None:
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def apply_wheel_velocities(self, left_velocity: float, right_velocity: float) -> None:
        self.left_motor.setVelocity(float(left_velocity))
        self.right_motor.setVelocity(float(right_velocity))

    def stop(self) -> None:
        self.apply_wheel_velocities(0.0, 0.0)
