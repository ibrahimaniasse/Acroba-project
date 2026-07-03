"""Return To Home — Navigate back to the pose recorded at behavior startup.

Records the robot's current pose when activated, then on subsequent calls
drives back to that recorded origin. Delegates actual movement logic to
MoveToWaypoint internally.

Parameters (via BehaviorContext.parameters):
    linear_speed (float): Max linear velocity m/s (default: 0.4)
    angular_speed (float): Max angular velocity rad/s (default: 1.0)
    distance_tolerance (float): Goal threshold in meters (default: 0.15)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from acroba_behaviors_py.base_behavior import BaseBehavior, BehaviorContext, Status


class ReturnToHome(BaseBehavior):
    """Return to the pose recorded when the behavior was first activated."""

    def __init__(self, name: str = 'return_to_home', description: str = '') -> None:
        super().__init__(name, description)
        self._home_x: float = 0.0
        self._home_y: float = 0.0
        self._home_theta: float = 0.0
        self._linear_speed: float = 0.4
        self._angular_speed: float = 1.0
        self._dist_tolerance: float = 0.15
        self._cmd_pub = None

    def on_enter(self, context: BehaviorContext) -> None:
        params = context.parameters
        self._linear_speed = float(params.get('linear_speed', '0.4'))
        self._angular_speed = float(params.get('angular_speed', '1.0'))
        self._dist_tolerance = float(params.get('distance_tolerance', '0.15'))

        # Record current pose as "home" if not previously stored
        if 'home_pose' not in context.shared_data and context.robot_pose is not None:
            context.shared_data['home_pose'] = context.robot_pose

        home = context.shared_data.get('home_pose', (0.0, 0.0, 0.0))
        self._home_x, self._home_y, self._home_theta = home

        if 'node' in context.shared_data:
            node = context.shared_data['node']
            self._cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)

        context.shared_data['detail'] = (
            f'Returning to home ({self._home_x:.1f}, {self._home_y:.1f})'
        )

    def on_tick(self, context: BehaviorContext) -> Status:
        if context.robot_pose is None:
            context.shared_data['detail'] = 'Waiting for odometry...'
            return Status.RUNNING

        x, y, theta = context.robot_pose
        dx = self._home_x - x
        dy = self._home_y - y
        distance = math.hypot(dx, dy)

        if distance <= self._dist_tolerance:
            context.shared_data['progress'] = 1.0
            context.shared_data['detail'] = 'Home reached'
            self._publish_cmd(Twist())
            return Status.SUCCEEDED

        angle_to_home = math.atan2(dy, dx)
        heading_error = self._normalize_angle(angle_to_home - theta)

        cmd = Twist()
        if abs(heading_error) > 0.3:
            # Turn toward home first
            cmd.angular.z = max(-self._angular_speed,
                                min(self._angular_speed, heading_error * 2.0))
        else:
            cmd.linear.x = max(0.0, min(self._linear_speed, distance * 0.8))
            cmd.angular.z = max(-self._angular_speed,
                                min(self._angular_speed, heading_error * 1.5))

        context.shared_data['detail'] = f'Distance to home: {distance:.2f}m'
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
    node = rclpy.create_node('return_to_home_standalone')
    node.get_logger().info('ReturnToHome standalone node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
