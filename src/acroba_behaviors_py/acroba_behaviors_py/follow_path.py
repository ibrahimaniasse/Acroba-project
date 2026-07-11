"""Follow Path — Follow a predefined sequence of poses.

Navigates through an ordered list of poses using pure pursuit-like
lookahead tracking. Unlike patrol_loop, this behavior is designed
for following smooth pre-planned paths (e.g. from a path planner)
with tighter tracking tolerances.

Parameters (via BehaviorContext.parameters):
    path_topic (str): Topic to receive nav_msgs/Path (default: '/planned_path')
    lookahead_distance (float): Lookahead for tracking in meters (default: 0.5)
    linear_speed (float): Max linear velocity m/s (default: 0.4)
    goal_tolerance (float): Final goal threshold in meters (default: 0.1)
"""

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

from acroba_behaviors_py.base_behavior import BaseBehavior, BehaviorContext, Status


class FollowPath(BaseBehavior):
    """Follow a nav_msgs/Path using pure-pursuit-inspired tracking."""

    def __init__(self, name: str = 'follow_path', description: str = '') -> None:
        super().__init__(name, description)
        self._path: List[Tuple[float, float]] = []
        self._current_idx: int = 0
        self._lookahead: float = 0.5
        self._linear_speed: float = 0.4
        self._goal_tolerance: float = 0.1
        self._cmd_pub = None
        self._path_sub = None

    def on_enter(self, context: BehaviorContext) -> None:
        params = context.parameters
        path_topic = params.get('path_topic', '/planned_path')
        self._lookahead = float(params.get('lookahead_distance', '0.5'))
        self._linear_speed = float(params.get('linear_speed', '0.4'))
        self._goal_tolerance = float(params.get('goal_tolerance', '0.1'))
        self._path = []
        self._current_idx = 0

        if 'node' in context.shared_data:
            node: Node = context.shared_data['node']
            self._cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)
            self._path_sub = node.create_subscription(
                Path, path_topic, self._path_callback, 10
            )

        context.shared_data['detail'] = f'Waiting for path on {path_topic}'

    def on_tick(self, context: BehaviorContext) -> Status:
        if context.robot_pose is None or not self._path:
            return Status.RUNNING

        x, y, theta = context.robot_pose

        # Find the lookahead point on the path
        lookahead_point = self._find_lookahead(x, y)
        if lookahead_point is None:
            # Past the last point — check if goal reached
            last = self._path[-1]
            dist_to_goal = math.hypot(last[0] - x, last[1] - y)
            if dist_to_goal <= self._goal_tolerance:
                context.shared_data['progress'] = 1.0
                context.shared_data['detail'] = 'Path completed'
                self._publish_cmd(Twist())
                return Status.SUCCEEDED
            # Navigate directly to goal
            lookahead_point = last

        lx, ly = lookahead_point
        dx, dy = lx - x, ly - y
        angle_to_target = math.atan2(dy, dx)
        heading_error = self._normalize_angle(angle_to_target - theta)
        distance = math.hypot(dx, dy)

        cmd = Twist()
        cmd.linear.x = max(0.0, min(self._linear_speed, distance * 0.8))
        cmd.angular.z = max(-1.5, min(1.5, heading_error * 2.5))

        # Slow down on tight turns
        if abs(heading_error) > 0.5:
            cmd.linear.x *= 0.3

        context.shared_data['progress'] = min(
            1.0, self._current_idx / max(len(self._path), 1)
        )
        context.shared_data['detail'] = (
            f'Following path — point {self._current_idx}/{len(self._path)}'
        )

        self._publish_cmd(cmd)
        return Status.RUNNING

    def on_exit(self, context: BehaviorContext) -> None:
        self._publish_cmd(Twist())
        if self._path_sub is not None and 'node' in context.shared_data:
            context.shared_data['node'].destroy_subscription(self._path_sub)
            self._path_sub = None

    def _path_callback(self, msg: Path) -> None:
        """Store incoming path as a list of (x, y) tuples."""
        self._path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self._current_idx = 0

    def _find_lookahead(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """Find the first path point beyond the lookahead distance."""
        for i in range(self._current_idx, len(self._path)):
            px, py = self._path[i]
            dist = math.hypot(px - x, py - y)
            if dist >= self._lookahead:
                self._current_idx = i
                return (px, py)
        self._current_idx = len(self._path)
        return None

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
    node = rclpy.create_node('follow_path_standalone')
    node.get_logger().info('FollowPath standalone node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
