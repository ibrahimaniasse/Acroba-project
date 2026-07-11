"""Formation Hold — Maintain a relative offset from a reference pose.

Subscribes to a reference pose topic (e.g. a leader robot's odometry)
and maintains a configurable offset. Useful for multi-robot coordination
scenarios where followers need to hold position relative to a leader.

Parameters (via BehaviorContext.parameters):
    offset_x (float): Forward offset from reference in meters (default: 1.0)
    offset_y (float): Lateral offset from reference in meters (default: 0.0)
    reference_topic (str): Topic for reference pose (default: '/leader/odom')
    linear_speed (float): Max linear velocity m/s (default: 0.4)
    distance_tolerance (float): Acceptable position error in meters (default: 0.2)
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from acroba_behaviors_py.base_behavior import BaseBehavior, BehaviorContext, Status


def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FormationHold(BaseBehavior):
    """Maintain a relative offset from a reference pose (leader)."""

    def __init__(self, name: str = 'formation_hold', description: str = '') -> None:
        super().__init__(name, description)
        self._offset_x: float = 1.0
        self._offset_y: float = 0.0
        self._linear_speed: float = 0.4
        self._dist_tolerance: float = 0.2
        self._reference_pose: Optional[Tuple[float, float, float]] = None
        self._cmd_pub = None
        self._ref_sub = None

    def on_enter(self, context: BehaviorContext) -> None:
        params = context.parameters
        self._offset_x = float(params.get('offset_x', '1.0'))
        self._offset_y = float(params.get('offset_y', '0.0'))
        ref_topic = params.get('reference_topic', '/leader/odom')
        self._linear_speed = float(params.get('linear_speed', '0.4'))
        self._dist_tolerance = float(params.get('distance_tolerance', '0.2'))
        self._reference_pose = None

        if 'node' in context.shared_data:
            node: Node = context.shared_data['node']
            self._cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)
            self._ref_sub = node.create_subscription(
                Odometry, ref_topic, self._ref_callback, 10
            )

        context.shared_data['detail'] = f'Waiting for reference on {ref_topic}'

    def on_tick(self, context: BehaviorContext) -> Status:
        if context.robot_pose is None or self._reference_pose is None:
            return Status.RUNNING

        # Compute target position in world frame from reference + offset
        ref_x, ref_y, ref_theta = self._reference_pose
        target_x = ref_x + (
            self._offset_x * math.cos(ref_theta)
            - self._offset_y * math.sin(ref_theta)
        )
        target_y = ref_y + (
            self._offset_x * math.sin(ref_theta)
            + self._offset_y * math.cos(ref_theta)
        )

        x, y, theta = context.robot_pose
        dx, dy = target_x - x, target_y - y
        distance = math.hypot(dx, dy)

        if distance <= self._dist_tolerance:
            # In position — hold (publish zero vel)
            self._publish_cmd(Twist())
            context.shared_data['detail'] = 'Formation position held'
            return Status.RUNNING  # Never "succeeds" — holds indefinitely

        # Navigate toward formation position
        angle_to_target = math.atan2(dy, dx)
        heading_error = self._normalize_angle(angle_to_target - theta)

        cmd = Twist()
        if abs(heading_error) > 0.3:
            cmd.angular.z = max(-1.0, min(1.0, heading_error * 2.0))
        else:
            cmd.linear.x = max(0.0, min(self._linear_speed, distance * 0.8))
            cmd.angular.z = max(-1.0, min(1.0, heading_error * 1.5))

        context.shared_data['detail'] = (
            f'Correcting formation — {distance:.2f}m from target'
        )
        self._publish_cmd(cmd)
        return Status.RUNNING

    def on_exit(self, context: BehaviorContext) -> None:
        self._publish_cmd(Twist())
        if self._ref_sub is not None and 'node' in context.shared_data:
            context.shared_data['node'].destroy_subscription(self._ref_sub)
            self._ref_sub = None

    def _ref_callback(self, msg: Odometry) -> None:
        """Update reference pose from leader odometry."""
        pos = msg.pose.pose.position
        yaw = _quaternion_to_yaw(msg.pose.pose.orientation)
        self._reference_pose = (pos.x, pos.y, yaw)

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
    node = rclpy.create_node('formation_hold_standalone')
    node.get_logger().info('FormationHold standalone node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
