"""Dock At Station — Approach and align with a docking station.

Two-phase docking maneuver:
  1. Navigate to an approach point offset from the station
  2. Final slow approach with tight alignment tolerance

Parameters (via BehaviorContext.parameters):
    station_x (float): Docking station X coordinate (default: 0.0)
    station_y (float): Docking station Y coordinate (default: 0.0)
    station_theta (float): Station facing direction in radians (default: 0.0)
    approach_distance (float): Distance from station to begin final approach (default: 0.5)
    approach_speed (float): Speed during final approach m/s (default: 0.15)
    final_distance (float): Docked threshold in meters (default: 0.05)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from acroba_behaviors_py.base_behavior import BaseBehavior, BehaviorContext, Status


class DockAtStation(BaseBehavior):
    """Two-phase docking: approach point then final slow alignment."""

    def __init__(self, name: str = 'dock_at_station', description: str = '') -> None:
        super().__init__(name, description)
        self._station_x: float = 0.0
        self._station_y: float = 0.0
        self._station_theta: float = 0.0
        self._approach_dist: float = 0.5
        self._approach_speed: float = 0.15
        self._final_dist: float = 0.05
        self._phase: str = 'approach'  # 'approach' or 'dock'
        self._cmd_pub = None

    def on_enter(self, context: BehaviorContext) -> None:
        params = context.parameters
        self._station_x = float(params.get('station_x', '0.0'))
        self._station_y = float(params.get('station_y', '0.0'))
        self._station_theta = float(params.get('station_theta', '0.0'))
        self._approach_dist = float(params.get('approach_distance', '0.5'))
        self._approach_speed = float(params.get('approach_speed', '0.15'))
        self._final_dist = float(params.get('final_distance', '0.05'))
        self._phase = 'approach'

        if 'node' in context.shared_data:
            self._cmd_pub = context.shared_data['node'].create_publisher(
                Twist, 'cmd_vel', 10
            )

        context.shared_data['detail'] = 'Approaching docking station'

    def on_tick(self, context: BehaviorContext) -> Status:
        if context.robot_pose is None:
            return Status.RUNNING

        x, y, theta = context.robot_pose

        if self._phase == 'approach':
            return self._approach_phase(x, y, theta, context)
        else:
            return self._dock_phase(x, y, theta, context)

    def _approach_phase(
        self, x: float, y: float, theta: float, context: BehaviorContext
    ) -> Status:
        """Navigate to the approach point (offset from station)."""
        # Approach point is behind the station at approach_distance
        ap_x = self._station_x - self._approach_dist * math.cos(self._station_theta)
        ap_y = self._station_y - self._approach_dist * math.sin(self._station_theta)

        dx, dy = ap_x - x, ap_y - y
        distance = math.hypot(dx, dy)

        if distance <= 0.15:
            self._phase = 'dock'
            context.shared_data['detail'] = 'Final docking approach'
            context.shared_data['progress'] = 0.5
            return Status.RUNNING

        angle_to_ap = math.atan2(dy, dx)
        heading_error = self._normalize_angle(angle_to_ap - theta)

        cmd = Twist()
        if abs(heading_error) > 0.3:
            cmd.angular.z = max(-1.0, min(1.0, heading_error * 2.0))
        else:
            cmd.linear.x = max(0.0, min(0.3, distance * 0.6))
            cmd.angular.z = max(-1.0, min(1.0, heading_error * 1.5))

        context.shared_data['detail'] = f'Approach: {distance:.2f}m remaining'
        self._publish_cmd(cmd)
        return Status.RUNNING

    def _dock_phase(
        self, x: float, y: float, theta: float, context: BehaviorContext
    ) -> Status:
        """Final slow approach with tight alignment."""
        dx = self._station_x - x
        dy = self._station_y - y
        distance = math.hypot(dx, dy)

        if distance <= self._final_dist:
            context.shared_data['progress'] = 1.0
            context.shared_data['detail'] = 'Docked successfully'
            self._publish_cmd(Twist())
            return Status.SUCCEEDED

        # Align with station heading
        heading_error = self._normalize_angle(self._station_theta - theta)

        cmd = Twist()
        if abs(heading_error) > 0.1:
            cmd.angular.z = max(-0.5, min(0.5, heading_error * 1.5))
        else:
            cmd.linear.x = min(self._approach_speed, distance * 0.5)
            cmd.angular.z = max(-0.3, min(0.3, heading_error * 1.0))

        context.shared_data['detail'] = f'Docking: {distance:.3f}m remaining'
        context.shared_data['progress'] = 0.5 + 0.5 * (
            1.0 - distance / max(self._approach_dist, 0.01)
        )
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
    node = rclpy.create_node('dock_at_station_standalone')
    node.get_logger().info('DockAtStation standalone node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
