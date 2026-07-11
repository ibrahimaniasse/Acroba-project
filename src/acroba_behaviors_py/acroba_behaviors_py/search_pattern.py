"""Search Pattern — Systematic area coverage using lawnmower or spiral patterns.

Generates a set of waypoints covering a rectangular area, then navigates
through them sequentially. Supports two patterns:
  - lawnmower: parallel sweeps with configurable spacing
  - spiral: inward spiral from the edges

Parameters (via BehaviorContext.parameters):
    pattern (str): 'lawnmower' or 'spiral' (default: 'lawnmower')
    area_width (float): Width of the search area in meters (default: 10.0)
    area_height (float): Height of the search area in meters (default: 10.0)
    spacing (float): Distance between sweeps in meters (default: 2.0)
    linear_speed (float): Max linear velocity m/s (default: 0.4)
    distance_tolerance (float): Waypoint reached threshold (default: 0.3)
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from acroba_behaviors_py.base_behavior import BaseBehavior, BehaviorContext, Status


class SearchPattern(BaseBehavior):
    """Systematic area coverage using generated waypoint patterns."""

    def __init__(self, name: str = 'search_pattern', description: str = '') -> None:
        super().__init__(name, description)
        self._waypoints: List[Tuple[float, float]] = []
        self._current_idx: int = 0
        self._linear_speed: float = 0.4
        self._angular_speed: float = 1.0
        self._dist_tolerance: float = 0.3
        self._cmd_pub = None

    def on_enter(self, context: BehaviorContext) -> None:
        params = context.parameters
        pattern = params.get('pattern', 'lawnmower')
        width = float(params.get('area_width', '10.0'))
        height = float(params.get('area_height', '10.0'))
        spacing = float(params.get('spacing', '2.0'))
        self._linear_speed = float(params.get('linear_speed', '0.4'))
        self._dist_tolerance = float(params.get('distance_tolerance', '0.3'))

        origin = context.robot_pose[:2] if context.robot_pose else (0.0, 0.0)

        if pattern == 'spiral':
            self._waypoints = self._generate_spiral(origin, width, height, spacing)
        else:
            self._waypoints = self._generate_lawnmower(origin, width, height, spacing)

        self._current_idx = 0

        if 'node' in context.shared_data:
            self._cmd_pub = context.shared_data['node'].create_publisher(
                Twist, 'cmd_vel', 10
            )

        context.shared_data['detail'] = (
            f'Searching {pattern} pattern — {len(self._waypoints)} waypoints'
        )

    def on_tick(self, context: BehaviorContext) -> Status:
        if context.robot_pose is None or not self._waypoints:
            return Status.RUNNING

        if self._current_idx >= len(self._waypoints):
            context.shared_data['progress'] = 1.0
            context.shared_data['detail'] = 'Search pattern complete'
            self._publish_cmd(Twist())
            return Status.SUCCEEDED

        tx, ty = self._waypoints[self._current_idx]
        x, y, theta = context.robot_pose
        dx, dy = tx - x, ty - y
        distance = math.hypot(dx, dy)

        if distance <= self._dist_tolerance:
            self._current_idx += 1
            context.shared_data['progress'] = self._current_idx / len(self._waypoints)
            context.shared_data['detail'] = (
                f'Waypoint {self._current_idx}/{len(self._waypoints)}'
            )
            return Status.RUNNING

        angle_to_target = math.atan2(dy, dx)
        heading_error = self._normalize_angle(angle_to_target - theta)

        cmd = Twist()
        if abs(heading_error) > 0.3:
            cmd.angular.z = max(-self._angular_speed,
                                min(self._angular_speed, heading_error * 2.0))
        else:
            cmd.linear.x = max(0.0, min(self._linear_speed, distance * 0.8))
            cmd.angular.z = max(-self._angular_speed,
                                min(self._angular_speed, heading_error * 1.5))

        self._publish_cmd(cmd)
        return Status.RUNNING

    def on_exit(self, context: BehaviorContext) -> None:
        self._publish_cmd(Twist())

    @staticmethod
    def _generate_lawnmower(
        origin: Tuple[float, float],
        width: float,
        height: float,
        spacing: float,
    ) -> List[Tuple[float, float]]:
        """Generate lawnmower sweep waypoints over a rectangular area."""
        ox, oy = origin
        waypoints = []
        num_sweeps = max(1, int(height / spacing))
        forward = True

        for i in range(num_sweeps + 1):
            y = oy + i * spacing
            if forward:
                waypoints.append((ox, y))
                waypoints.append((ox + width, y))
            else:
                waypoints.append((ox + width, y))
                waypoints.append((ox, y))
            forward = not forward

        return waypoints

    @staticmethod
    def _generate_spiral(
        origin: Tuple[float, float],
        width: float,
        height: float,
        spacing: float,
    ) -> List[Tuple[float, float]]:
        """Generate inward spiral waypoints."""
        ox, oy = origin
        waypoints = []
        x_min, x_max = ox, ox + width
        y_min, y_max = oy, oy + height

        while x_min < x_max and y_min < y_max:
            # Bottom edge (left to right)
            waypoints.append((x_min, y_min))
            waypoints.append((x_max, y_min))
            # Right edge (bottom to top)
            waypoints.append((x_max, y_max))
            # Top edge (right to left)
            waypoints.append((x_min, y_max))

            x_min += spacing
            x_max -= spacing
            y_min += spacing
            y_max -= spacing

        return waypoints

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
    node = rclpy.create_node('search_pattern_standalone')
    node.get_logger().info('SearchPattern standalone node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
