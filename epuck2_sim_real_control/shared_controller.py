from __future__ import annotations

import math

from epuck2_sim_real_control.contracts import MotionStatus, Pose2D, VelocityCommand
from epuck2_sim_real_control.session_manifest import SessionManifest


class SharedPoseController:
    def __init__(self, manifest: SessionManifest):
        self.manifest = manifest

    def compute_command(self, status: MotionStatus, goal_pose: Pose2D) -> VelocityCommand:
        dx = float(goal_pose.x) - float(status.pose.x)
        dy = float(goal_pose.y) - float(status.pose.y)
        distance = math.hypot(dx, dy)
        heading_to_goal = self._wrap_angle(float(goal_pose.theta) - float(status.pose.theta))
        if (
            distance <= float(self.manifest.goal_tolerance_m)
            and abs(heading_to_goal) <= float(self.manifest.goal_heading_tolerance_rad)
        ):
            return VelocityCommand(linear=0.0, angular=0.0)

        if distance <= float(self.manifest.goal_tolerance_m):
            angular = max(
                -float(self.manifest.max_angular_velocity),
                min(float(self.manifest.max_angular_velocity), float(self.manifest.angular_gain) * heading_to_goal),
            )
            return VelocityCommand(linear=0.0, angular=angular)

        target_heading = math.atan2(dy, dx)
        heading_error = self._wrap_angle(target_heading - float(status.pose.theta))
        linear = min(float(self.manifest.max_linear_velocity), float(self.manifest.linear_gain) * distance)
        angular = max(
            -float(self.manifest.max_angular_velocity),
            min(float(self.manifest.max_angular_velocity), float(self.manifest.angular_gain) * heading_error),
        )

        if abs(heading_error) > math.pi / 3.0:
            linear = 0.0
        return VelocityCommand(linear=linear, angular=angular)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
