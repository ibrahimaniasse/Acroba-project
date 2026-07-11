"""Patrol Loop — Cycle through a list of waypoints.

Iterates through an ordered list of waypoints, moving to each one
sequentially. Can loop indefinitely or for a configured number of cycles.

Parameters (via BehaviorContext.parameters):
    waypoints (str): JSON list of [x, y, theta] waypoints
        (default: '[[2,0,0],[-2,0,3.14],[0,2,1.57],[0,-2,-1.57]]')
    linear_speed (float): Max linear velocity m/s (default: 0.5)
    angular_speed (float): Max angular velocity rad/s (default: 1.0)
    distance_tolerance (float): Waypoint reached threshold (default: 0.2)
    loop_count (int): Number of full cycles, 0 = infinite (default: 0)
"""

import json
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from acroba_behaviors_py.base_behavior import BaseBehavior, BehaviorContext, Status


class PatrolLoop(BaseBehavior):
    """Cycle through a list of waypoints with configurable loop count."""

    def __init__(self, name: str = 'patrol_loop', description: str = '') -> None:
        super().__init__(name, description)
        self._waypoints: List[Tuple[float, float, float]] = []
        self._current_idx: int = 0
        self._loops_completed: int = 0
        self._max_loops: int = 0
        self._linear_speed: float = 0.5
        self._angular_speed: float = 1.0
        self._dist_tolerance: float = 0.2
        self._cmd_pub = None

    def on_enter(self, context: BehaviorContext) -> None:
        params = context.parameters
        waypoints_str = params.get(
            'waypoints', '[[2,0,0],[-2,0,3.14],[0,2,1.57],[0,-2,-1.57]]'
        )
        self._waypoints = [tuple(wp) for wp in json.loads(waypoints_str)]
        self._current_idx = 0
        self._loops_completed = 0
        self._max_loops = int(params.get('loop_count', '0'))
        self._linear_speed = float(params.get('linear_speed', '0.5'))
        self._angular_speed = float(params.get('angular_speed', '1.0'))
        self._dist_tolerance = float(params.get('distance_tolerance', '0.2'))

        if 'node' in context.shared_data:
            self._cmd_pub = context.shared_data['node'].create_publisher(
                Twist, 'cmd_vel', 10
            )

        context.shared_data['detail'] = (
            f'Patrolling {len(self._waypoints)} waypoints'
        )

    def on_tick(self, context: BehaviorContext) -> Status:
        if context.robot_pose is None or not self._waypoints:
            return Status.RUNNING

        target = self._waypoints[self._current_idx]
        tx, ty = target[0], target[1]
        x, y, theta = context.robot_pose

        dx, dy = tx - x, ty - y
        distance = math.hypot(dx, dy)

        if distance <= self._dist_tolerance:
            # Waypoint reached — advance to next
            self._current_idx += 1
            if self._current_idx >= len(self._waypoints):
                self._current_idx = 0
                self._loops_completed += 1
                if self._max_loops > 0 and self._loops_completed >= self._max_loops:
                    context.shared_data['progress'] = 1.0
                    context.shared_data['detail'] = (
                        f'Patrol complete ({self._loops_completed} loops)'
                    )
                    return Status.SUCCEEDED

            context.shared_data['detail'] = (
                f'Loop {self._loops_completed + 1}, '
                f'waypoint {self._current_idx + 1}/{len(self._waypoints)}'
            )
            return Status.RUNNING

        # Navigate toward current waypoint
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
    node = rclpy.create_node('patrol_loop_standalone')
    node.get_logger().info('PatrolLoop standalone node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
