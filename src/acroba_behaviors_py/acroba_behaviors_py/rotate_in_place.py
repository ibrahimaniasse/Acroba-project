"""Rotate In Place — Rotate the robot to face a target heading.

Pure rotation behavior using a proportional controller.
Succeeds when the heading error is below the configured tolerance.

Parameters (via BehaviorContext.parameters):
    target_theta (float): Target heading in radians (default: 0.0)
    angular_speed (float): Maximum angular velocity rad/s (default: 1.0)
    heading_tolerance (float): Success threshold in radians (default: 0.05)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from acroba_behaviors_py.base_behavior import BaseBehavior, BehaviorContext, Status


class RotateInPlace(BaseBehavior):
    """Pure rotation to a target heading."""

    def __init__(self, name: str = 'rotate_in_place', description: str = '') -> None:
        super().__init__(name, description)
        self._target_theta: float = 0.0
        self._angular_speed: float = 1.0
        self._tolerance: float = 0.05
        self._cmd_pub = None

    def on_enter(self, context: BehaviorContext) -> None:
        params = context.parameters
        self._target_theta = float(params.get('target_theta', '0.0'))
        self._angular_speed = float(params.get('angular_speed', '1.0'))
        self._tolerance = float(params.get('heading_tolerance', '0.05'))

        if 'node' in context.shared_data:
            node = context.shared_data['node']
            self._cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)

        context.shared_data['detail'] = (
            f'Rotating to {math.degrees(self._target_theta):.1f}°'
        )

    def on_tick(self, context: BehaviorContext) -> Status:
        if context.robot_pose is None:
            return Status.RUNNING

        _, _, theta = context.robot_pose
        error = self._normalize_angle(self._target_theta - theta)

        if abs(error) <= self._tolerance:
            context.shared_data['progress'] = 1.0
            context.shared_data['detail'] = 'Target heading reached'
            self._publish_cmd(Twist())
            return Status.SUCCEEDED

        cmd = Twist()
        cmd.angular.z = max(-self._angular_speed,
                            min(self._angular_speed, error * 2.0))

        context.shared_data['progress'] = 1.0 - abs(error) / math.pi
        context.shared_data['detail'] = f'Heading error: {math.degrees(error):.1f}°'
        self._publish_cmd(cmd)
        return Status.RUNNING

    def on_exit(self, context: BehaviorContext) -> None:
        self._publish_cmd(Twist())

    def _publish_cmd(self, cmd: Twist) -> None:
        if self._cmd_pub is not None:
            self._cmd_pub.publish(cmd)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('rotate_in_place_standalone')
    node.get_logger().info('RotateInPlace standalone node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
