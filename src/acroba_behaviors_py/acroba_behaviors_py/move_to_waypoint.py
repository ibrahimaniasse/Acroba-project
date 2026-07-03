"""Move To Waypoint — Navigate the robot to a target (x, y, θ) pose.

Uses a proportional controller to compute linear and angular velocities
toward the target pose. Succeeds when the robot is within configurable
distance and heading tolerances.

Parameters (via BehaviorContext.parameters):
    target_x (float): Target X coordinate in meters (default: 0.0)
    target_y (float): Target Y coordinate in meters (default: 0.0)
    target_theta (float): Target heading in radians (default: 0.0)
    linear_speed (float): Maximum linear velocity m/s (default: 0.5)
    angular_speed (float): Maximum angular velocity rad/s (default: 1.0)
    distance_tolerance (float): Goal reached threshold in meters (default: 0.1)
    heading_tolerance (float): Heading aligned threshold in radians (default: 0.1)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from acroba_behaviors_py.base_behavior import BaseBehavior, BehaviorContext, Status


class MoveToWaypoint(BaseBehavior):
    """Navigate to a specific (x, y, theta) pose using P-control."""

    def __init__(self, name: str = 'move_to_waypoint', description: str = '') -> None:
        super().__init__(name, description)
        self._target_x: float = 0.0
        self._target_y: float = 0.0
        self._target_theta: float = 0.0
        self._linear_speed: float = 0.5
        self._angular_speed: float = 1.0
        self._dist_tolerance: float = 0.1
        self._heading_tolerance: float = 0.1
        self._node: Optional[Node] = None
        self._cmd_pub = None

    def on_enter(self, context: BehaviorContext) -> None:
        """Read target pose and speed parameters from context."""
        params = context.parameters
        self._target_x = float(params.get('target_x', '0.0'))
        self._target_y = float(params.get('target_y', '0.0'))
        self._target_theta = float(params.get('target_theta', '0.0'))
        self._linear_speed = float(params.get('linear_speed', '0.5'))
        self._angular_speed = float(params.get('angular_speed', '1.0'))
        self._dist_tolerance = float(params.get('distance_tolerance', '0.1'))
        self._heading_tolerance = float(params.get('heading_tolerance', '0.1'))

        # Get ROS node handle if available
        if 'node' in context.shared_data:
            self._node = context.shared_data['node']
            self._cmd_pub = self._node.create_publisher(Twist, 'cmd_vel', 10)

        context.shared_data['detail'] = (
            f'Moving to ({self._target_x:.1f}, {self._target_y:.1f}, '
            f'{math.degrees(self._target_theta):.0f}°)'
        )

    def on_tick(self, context: BehaviorContext) -> Status:
        """Compute and publish velocity commands toward the target."""
        if context.robot_pose is None:
            context.shared_data['detail'] = 'Waiting for odometry...'
            return Status.RUNNING

        x, y, theta = context.robot_pose
        dx = self._target_x - x
        dy = self._target_y - y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        heading_error = self._normalize_angle(angle_to_target - theta)

        cmd = Twist()

        if distance > self._dist_tolerance:
            # Phase 1: navigate to position
            if abs(heading_error) > self._heading_tolerance:
                # Turn toward target first
                cmd.angular.z = self._clamp(
                    heading_error * 2.0,
                    -self._angular_speed,
                    self._angular_speed,
                )
            else:
                # Drive forward
                cmd.linear.x = self._clamp(
                    distance * 1.0,
                    0.0,
                    self._linear_speed,
                )
                cmd.angular.z = self._clamp(
                    heading_error * 1.5,
                    -self._angular_speed,
                    self._angular_speed,
                )

            context.shared_data['progress'] = max(
                0.0, 1.0 - distance / max(
                    math.hypot(self._target_x, self._target_y), 0.01
                )
            )
            context.shared_data['detail'] = f'Distance: {distance:.2f}m'
        else:
            # Phase 2: align final heading
            final_error = self._normalize_angle(self._target_theta - theta)
            if abs(final_error) > self._heading_tolerance:
                cmd.angular.z = self._clamp(
                    final_error * 2.0,
                    -self._angular_speed,
                    self._angular_speed,
                )
                context.shared_data['detail'] = (
                    f'Aligning heading: {math.degrees(final_error):.1f}° remaining'
                )
            else:
                context.shared_data['progress'] = 1.0
                context.shared_data['detail'] = 'Waypoint reached'
                self._publish_cmd(Twist())  # Stop
                return Status.SUCCEEDED

        self._publish_cmd(cmd)
        return Status.RUNNING

    def on_exit(self, context: BehaviorContext) -> None:
        """Stop the robot."""
        self._publish_cmd(Twist())

    def _publish_cmd(self, cmd: Twist) -> None:
        """Publish a velocity command if publisher is available."""
        if self._cmd_pub is not None:
            self._cmd_pub.publish(cmd)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp a value between min and max."""
        return max(min_val, min(value, max_val))


def main(args=None):
    rclpy.init(args=args)
    # Standalone node for direct testing
    node = rclpy.create_node('move_to_waypoint_standalone')
    behavior = MoveToWaypoint(name='move_to_waypoint')
    context = BehaviorContext()
    context.shared_data['node'] = node
    behavior.on_enter(context)
    node.get_logger().info('MoveToWaypoint standalone node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        behavior.on_exit(context)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
