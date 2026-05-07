from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.session_manifest import SessionManifest
from epuck2_sim_real_control.webots_live_controller import WebotsLiveController, load_controller_manifest


class _FakeGPS:
    def __init__(self):
        self.values = [0.0, 0.0, 0.0]
        self.enabled = None

    def enable(self, step_ms):
        self.enabled = step_ms

    def getValues(self):
        return list(self.values)


class _FakeCompass:
    def __init__(self):
        self.values = [1.0, 0.0, 0.0]
        self.enabled = None

    def enable(self, step_ms):
        self.enabled = step_ms

    def getValues(self):
        return list(self.values)


class _FakeMotor:
    def __init__(self):
        self.position = None
        self.velocity = 0.0
        self.history = []

    def setPosition(self, position):
        self.position = position

    def setVelocity(self, velocity):
        self.velocity = float(velocity)
        self.history.append(float(velocity))

    def getVelocity(self):
        return self.velocity


class _FakeRobot:
    def __init__(self):
        self.step_count = 0
        self.time = 0.0
        self.gps = _FakeGPS()
        self.compass = _FakeCompass()
        self.left_motor = _FakeMotor()
        self.right_motor = _FakeMotor()
        self.devices = {
            'gps': self.gps,
            'compass': self.compass,
            'left wheel motor': self.left_motor,
            'right wheel motor': self.right_motor,
        }

    def getBasicTimeStep(self):
        return 32

    def getTime(self):
        return self.time

    def getDevice(self, name):
        return self.devices[name]

    def step(self, timestep):
        if self.step_count >= 3:
            return -1
        self.step_count += 1
        self.time += timestep / 1000.0
        if self.step_count == 1:
            self.gps.values = [0.05, 0.0, 0.0]
        elif self.step_count == 2:
            self.gps.values = [0.12, 0.0, 0.0]
        else:
            self.gps.values = [0.20, 0.0, 0.0]
        return 0


def test_webots_live_controller_writes_dataset_and_applies_motor_commands(tmp_path: Path):
    manifest = SessionManifest(
        dataset_root=str(tmp_path),
        goal_x=0.20,
        goal_y=0.0,
        goal_theta=0.0,
        max_steps=5,
    )
    robot = _FakeRobot()
    result = WebotsLiveController(manifest, robot).run()

    run_dir = Path(result['run_dir'])
    assert result['sample_count'] == 3
    assert (run_dir / 'metadata.json').exists()
    assert (run_dir / 'samples.jsonl').exists()
    assert robot.left_motor.position == float('inf')
    assert robot.right_motor.position == float('inf')
    assert any(value != 0.0 for value in robot.left_motor.history)
    assert any(value != 0.0 for value in robot.right_motor.history)
    assert robot.left_motor.history[-1] == 0.0
    assert robot.right_motor.history[-1] == 0.0


def test_load_controller_manifest_uses_env_override_without_mutating_defaults(monkeypatch):
    monkeypatch.setenv('EPUCK2_DATASET_ROOT', '/tmp/epuck2-live-env')
    overridden = load_controller_manifest()
    monkeypatch.delenv('EPUCK2_DATASET_ROOT')
    defaulted = load_controller_manifest()

    assert overridden.dataset_root == '/tmp/epuck2-live-env'
    assert defaulted.dataset_root != '/tmp/epuck2-live-env'
