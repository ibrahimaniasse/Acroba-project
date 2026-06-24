"""Behavior Manager Node — Central orchestration for the Acroba framework.

This node acts as the single point of control for all robot behaviors:
  - Exposes an ExecuteBehavior action server for external callers
  - Loads the behavior registry from a YAML configuration file
  - Manages behavior lifecycle (activate, tick, preempt, deactivate)
  - Publishes continuous status on /acroba/behavior_status
  - Provides a /acroba/list_behaviors service for introspection

The manager subscribes to odometry and laser scan topics to maintain
an up-to-date BehaviorContext that is passed to behaviors on each tick.
"""

import importlib
import math
import time
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_group import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from acroba_msgs.action import ExecuteBehavior
from acroba_msgs.msg import BehaviorStatus
from acroba_msgs.srv import ListBehaviors

from acroba_behaviors_py.base_behavior import (
    BaseBehavior,
    BehaviorContext,
    Status,
)


def _quaternion_to_yaw(q) -> float:
    """Extract yaw angle from a quaternion orientation."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class BehaviorManagerNode(Node):
    """Central behavior orchestration node.

    Parameters (from YAML or command-line overrides):
        tick_rate_hz: How often to tick the active behavior (default: 20.0)
        behaviors_config: Path to the behaviors.yaml registry file
    """

    def __init__(self) -> None:
        super().__init__('behavior_manager')

        # Parameters
        self.declare_parameter('tick_rate_hz', 20.0)
        self.declare_parameter('behaviors_config', '')

        self._tick_rate = self.get_parameter('tick_rate_hz').value
        config_path = self.get_parameter('behaviors_config').value

        # Callback group for concurrent action handling
        self._cb_group = ReentrantCallbackGroup()

        # Behavior registry: name -> BaseBehavior instance
        self._registry: Dict[str, BaseBehavior] = {}
        self._load_behaviors(config_path)

        # Active behavior state
        self._active_behavior: Optional[BaseBehavior] = None
        self._context = BehaviorContext()
        self._preempt_requested = False

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._status_pub = self.create_publisher(
            BehaviorStatus, '/acroba/behavior_status', 10
        )

        # Subscribers
        self.create_subscription(
            Odometry, 'odom', self._odom_callback, 10
        )
        self.create_subscription(
            LaserScan, 'scan', self._scan_callback, 10
        )

        # Action server
        self._action_server = ActionServer(
            self,
            ExecuteBehavior,
            '/acroba/execute_behavior',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        # Service
        self.create_service(
            ListBehaviors,
            '/acroba/list_behaviors',
            self._list_behaviors_callback,
        )

        self.get_logger().info(
            f'Behavior Manager initialized — {len(self._registry)} behaviors registered, '
            f'tick rate: {self._tick_rate} Hz'
        )

    # ──────────────────────────────────────────────────────────
    # Registry loading
    # ──────────────────────────────────────────────────────────

    def _load_behaviors(self, config_path: str) -> None:
        """Load behavior plugins from YAML configuration.

        Expected YAML format:
            behaviors:
              - name: move_to_waypoint
                module: acroba_behaviors_py.move_to_waypoint
                class: MoveToWaypoint
                description: Navigate to a target (x, y, theta) pose
                default_params:
                  linear_speed: 0.5
                  angular_speed: 1.0
        """
        if not config_path:
            self.get_logger().warn(
                'No behaviors_config provided — loading built-in defaults'
            )
            self._load_builtin_behaviors()
            return

        try:
            import yaml
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            for entry in config.get('behaviors', []):
                name = entry['name']
                module_path = entry['module']
                class_name = entry['class']
                description = entry.get('description', '')

                try:
                    module = importlib.import_module(module_path)
                    cls = getattr(module, class_name)
                    behavior = cls(name=name, description=description)
                    self._registry[name] = behavior
                    self.get_logger().info(f'  Loaded behavior: {name} ({class_name})')
                except (ImportError, AttributeError) as e:
                    self.get_logger().error(f'  Failed to load {name}: {e}')

        except Exception as e:
            self.get_logger().error(f'Failed to read config {config_path}: {e}')
            self._load_builtin_behaviors()

    def _load_builtin_behaviors(self) -> None:
        """Fallback: load all built-in Python behaviors."""
        from acroba_behaviors_py.move_to_waypoint import MoveToWaypoint
        from acroba_behaviors_py.rotate_in_place import RotateInPlace
        from acroba_behaviors_py.wait_for_trigger import WaitForTrigger
        from acroba_behaviors_py.return_to_home import ReturnToHome
        from acroba_behaviors_py.patrol_loop import PatrolLoop
        from acroba_behaviors_py.search_pattern import SearchPattern
        from acroba_behaviors_py.follow_path import FollowPath
        from acroba_behaviors_py.dock_at_station import DockAtStation
        from acroba_behaviors_py.formation_hold import FormationHold

        builtins = [
            MoveToWaypoint(name='move_to_waypoint', description='Navigate to a target (x, y, theta) pose'),
            RotateInPlace(name='rotate_in_place', description='Rotate to face a target heading'),
            WaitForTrigger(name='wait_for_trigger', description='Wait for an external trigger signal'),
            ReturnToHome(name='return_to_home', description='Return to the pose recorded at startup'),
            PatrolLoop(name='patrol_loop', description='Cycle through a list of waypoints'),
            SearchPattern(name='search_pattern', description='Systematic area coverage (lawnmower/spiral)'),
            FollowPath(name='follow_path', description='Follow a predefined nav_msgs/Path'),
            DockAtStation(name='dock_at_station', description='Approach and align with a docking station'),
            FormationHold(name='formation_hold', description='Maintain a relative offset from a reference pose'),
        ]
        for b in builtins:
            self._registry[b.name] = b
            self.get_logger().info(f'  Loaded builtin: {b.name}')

    # ──────────────────────────────────────────────────────────
    # Subscriber callbacks
    # ──────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        """Update robot pose in the shared context from odometry."""
        pos = msg.pose.pose.position
        yaw = _quaternion_to_yaw(msg.pose.pose.orientation)
        self._context.robot_pose = (pos.x, pos.y, yaw)

    def _scan_callback(self, msg: LaserScan) -> None:
        """Update LiDAR ranges in the shared context."""
        self._context.scan_ranges = list(msg.ranges)

    # ──────────────────────────────────────────────────────────
    # Action server callbacks
    # ──────────────────────────────────────────────────────────

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept or reject a new behavior execution goal."""
        behavior_name = goal_request.behavior_name
        if behavior_name not in self._registry:
            self.get_logger().warn(f'Rejected: unknown behavior "{behavior_name}"')
            return GoalResponse.REJECT

        self.get_logger().info(f'Accepted goal: {behavior_name}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Handle preemption requests."""
        self.get_logger().info('Preemption requested')
        self._preempt_requested = True
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle) -> ExecuteBehavior.Result:
        """Execute a behavior through its full lifecycle."""
        behavior_name = goal_handle.request.behavior_name
        behavior = self._registry[behavior_name]
        behavior.reset()

        # Parse parameters from goal
        param_names = goal_handle.request.parameter_names
        param_values = goal_handle.request.parameter_values
        self._context.parameters = dict(zip(param_names, param_values))
        self._context.shared_data = {}

        self._active_behavior = behavior
        self._preempt_requested = False
        start_time = time.monotonic()

        # on_enter
        self.get_logger().info(f'Entering behavior: {behavior_name}')
        behavior.on_enter(self._context)
        behavior.status = Status.RUNNING

        # Tick loop
        tick_period = 1.0 / self._tick_rate
        result = ExecuteBehavior.Result()

        while rclpy.ok():
            # Check preemption
            if self._preempt_requested or goal_handle.is_cancel_requested:
                behavior.status = Status.PREEMPTED
                behavior.on_exit(self._context)
                self._stop_robot()
                self._publish_status(behavior_name, Status.PREEMPTED)

                goal_handle.canceled()
                result.success = False
                result.message = f'Behavior "{behavior_name}" preempted'
                result.execution_time = time.monotonic() - start_time
                self._active_behavior = None
                return result

            # Tick
            tick_status = behavior.on_tick(self._context)
            behavior.status = tick_status

            # Publish feedback
            feedback = ExecuteBehavior.Feedback()
            feedback.current_status = tick_status.name
            feedback.progress = self._context.shared_data.get('progress', 0.0)
            feedback.detail = self._context.shared_data.get('detail', '')
            goal_handle.publish_feedback(feedback)

            # Publish status topic
            self._publish_status(behavior_name, tick_status)

            # Check termination
            if tick_status == Status.SUCCEEDED:
                behavior.on_exit(self._context)
                self._stop_robot()
                goal_handle.succeed()
                result.success = True
                result.message = f'Behavior "{behavior_name}" completed successfully'
                result.execution_time = time.monotonic() - start_time
                self._active_behavior = None
                return result

            if tick_status == Status.FAILED:
                behavior.on_exit(self._context)
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = (
                    f'Behavior "{behavior_name}" failed: '
                    f'{self._context.shared_data.get("error", "unknown")}'
                )
                result.execution_time = time.monotonic() - start_time
                self._active_behavior = None
                return result

            # Sleep until next tick
            self.get_clock().sleep_for(
                rclpy.duration.Duration(seconds=tick_period)
            )

    # ──────────────────────────────────────────────────────────
    # Service callback
    # ──────────────────────────────────────────────────────────

    def _list_behaviors_callback(
        self,
        request: ListBehaviors.Request,
        response: ListBehaviors.Response,
    ) -> ListBehaviors.Response:
        """Return the list of all registered behaviors."""
        for name, behavior in self._registry.items():
            response.behavior_names.append(name)
            response.behavior_types.append(behavior.__class__.__name__)
            response.behavior_descriptions.append(behavior.description)
        return response

    # ──────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────

    def _publish_status(self, behavior_name: str, status: Status) -> None:
        """Publish current behavior status on the status topic."""
        msg = BehaviorStatus()
        msg.behavior_name = behavior_name
        msg.status = status.value
        msg.detail = self._context.shared_data.get('detail', '')
        msg.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(msg)

    def _stop_robot(self) -> None:
        """Publish zero velocity to stop the robot."""
        self._cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
