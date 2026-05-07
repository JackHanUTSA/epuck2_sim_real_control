from pathlib import Path
import math
import os
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.session_manifest import SessionManifest
from epuck2_sim_real_control.webots_connector import (
    WebotsStatusReader,
    discover_webots_home,
    ensure_webots_controller_importable,
)


class _FakeGPS:
    def __init__(self, values):
        self._values = values

    def enable(self, _step_ms):
        return None

    def getValues(self):
        return list(self._values)


class _FakeCompass:
    def __init__(self, values):
        self._values = values

    def enable(self, _step_ms):
        return None

    def getValues(self):
        return list(self._values)


class _FakeMotor:
    def __init__(self, velocity):
        self._velocity = velocity

    def getVelocity(self):
        return float(self._velocity)


class _FakeRobot:
    def __init__(self):
        self.devices = {
            'gps': _FakeGPS((0.25, 0.0, -0.4)),
            'compass': _FakeCompass((1.0, 0.0, 0.0)),
            'left wheel motor': _FakeMotor(1.2),
            'right wheel motor': _FakeMotor(1.5),
        }

    def getBasicTimeStep(self):
        return 32

    def getTime(self):
        return 4.5

    def getDevice(self, name):
        return self.devices[name]


def test_discover_webots_home_matches_local_installation():
    home = discover_webots_home()
    assert home is not None
    assert home.name == 'webots'
    assert (home / 'lib' / 'controller' / 'python').exists()


def test_ensure_webots_controller_importable_bootstraps_controller_module():
    controller = ensure_webots_controller_importable()
    assert hasattr(controller, 'Robot')
    assert os.environ.get('WEBOTS_HOME')


def test_webots_status_reader_extracts_pose_heading_and_wheel_velocity():
    manifest = SessionManifest(
        gps_device_name='gps',
        compass_device_name='compass',
        left_motor_device_name='left wheel motor',
        right_motor_device_name='right wheel motor',
        wheel_radius_m=0.021,
        axle_length_m=0.053,
    )
    reader = WebotsStatusReader(_FakeRobot(), manifest)

    raw_status = reader.read_raw_status()
    status = reader.read_status()

    assert raw_status['timestamp'] == 4.5
    assert raw_status['pose']['x'] == 0.25
    assert raw_status['pose']['y'] == -0.4
    assert math.isclose(status.linear_velocity, 0.02835, rel_tol=1e-6)
    assert math.isclose(status.angular_velocity, 0.11886792452830171, rel_tol=1e-6)
    assert status.left_wheel_velocity == 1.2
    assert status.right_wheel_velocity == 1.5
    assert status.source == 'sim'
