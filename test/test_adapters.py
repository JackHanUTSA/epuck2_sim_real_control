from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from epuck2_sim_real_control.adapters import RealRobotAdapter, WebotsSimAdapter
from epuck2_sim_real_control.contracts import VelocityCommand
from epuck2_sim_real_control.session_manifest import SessionManifest


def test_sim_and_real_adapters_share_the_same_command_contract():
    manifest = SessionManifest(
        mode='sim',
        policy_topic='/epuck2/cmd_vel',
        sample_rate_hz=20.0,
        max_linear_velocity=0.13,
        max_angular_velocity=4.5,
    )
    sim_adapter = WebotsSimAdapter(manifest)
    real_adapter = RealRobotAdapter(manifest)

    assert sim_adapter.command_contract() == real_adapter.command_contract()
    assert sim_adapter.command_contract() == {
        'command_type': 'velocity_command',
        'policy_topic': '/epuck2/cmd_vel',
        'max_linear_velocity': 0.13,
        'max_angular_velocity': 4.5,
        'sample_rate_hz': 20.0,
    }


def test_adapters_normalize_motion_status_into_one_schema():
    manifest = SessionManifest(mode='sim')
    sim_adapter = WebotsSimAdapter(manifest)
    real_adapter = RealRobotAdapter(manifest)

    sim_status = sim_adapter.normalize_status(
        {
            'timestamp': 1.0,
            'pose': {'x': 0.4, 'y': -0.1, 'theta': 0.2},
            'twist': {'linear': 0.08, 'angular': -0.3},
            'wheel': {'left': 1.2, 'right': 1.4},
        }
    )
    real_status = real_adapter.normalize_status(
        {
            'timestamp': 1.5,
            'pose': {'x': 0.42, 'y': -0.09, 'theta': 0.18},
            'twist': {'linear': 0.07, 'angular': -0.28},
            'wheel': {'left': 1.1, 'right': 1.35},
            'battery_voltage': 3.78,
        }
    )

    assert sim_status.source == 'sim'
    assert real_status.source == 'real'
    assert sim_status.pose.x == 0.4
    assert real_status.pose.y == -0.09
    assert sim_status.left_wheel_velocity == 1.2
    assert real_status.right_wheel_velocity == 1.35
    assert real_status.battery_voltage == 3.78


def test_velocity_command_is_clamped_to_manifest_limits():
    manifest = SessionManifest(max_linear_velocity=0.13, max_angular_velocity=4.5)
    sim_adapter = WebotsSimAdapter(manifest)

    clamped = sim_adapter.clamp_command(VelocityCommand(linear=0.25, angular=-9.0))

    assert clamped.linear == 0.13
    assert clamped.angular == -4.5
