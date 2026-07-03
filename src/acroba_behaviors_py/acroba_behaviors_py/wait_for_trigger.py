"""Wait For Trigger — Block until an external signal is received.

Subscribes to a configurable topic and waits for a message.
Useful for coordinating multi-step missions where the robot
must hold position until a human or upstream node gives the go-ahead.

Parameters (via BehaviorContext.parameters):
    trigger_topic (str): Topic to subscribe to (default: '/acroba/trigger')
    timeout_sec (float): Max wait time in seconds, 0 = infinite (default: 0.0)
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from acroba_behaviors_py.base_behavior import BaseBehavior, BehaviorContext, Status


class WaitForTrigger(BaseBehavior):
    """Wait for an external trigger signal on a configurable topic."""

    def __init__(self, name: str = 'wait_for_trigger', description: str = '') -> None:
        super().__init__(name, description)
        self._triggered: bool = False
        self._timeout: float = 0.0
        self._start_time: float = 0.0
        self._sub = None

    def on_enter(self, context: BehaviorContext) -> None:
        params = context.parameters
        topic = params.get('trigger_topic', '/acroba/trigger')
        self._timeout = float(params.get('timeout_sec', '0.0'))
        self._triggered = False
        self._start_time = time.monotonic()

        if 'node' in context.shared_data:
            node: Node = context.shared_data['node']
            self._sub = node.create_subscription(
                Bool, topic, self._trigger_callback, 10
            )

        context.shared_data['detail'] = f'Waiting for trigger on {topic}'

    def on_tick(self, context: BehaviorContext) -> Status:
        if self._triggered:
            context.shared_data['progress'] = 1.0
            context.shared_data['detail'] = 'Trigger received'
            return Status.SUCCEEDED

        elapsed = time.monotonic() - self._start_time

        if self._timeout > 0.0 and elapsed >= self._timeout:
            context.shared_data['error'] = 'Trigger timeout'
            context.shared_data['detail'] = f'Timed out after {elapsed:.1f}s'
            return Status.FAILED

        context.shared_data['detail'] = f'Waiting... ({elapsed:.0f}s elapsed)'
        return Status.RUNNING

    def on_exit(self, context: BehaviorContext) -> None:
        if self._sub is not None and 'node' in context.shared_data:
            context.shared_data['node'].destroy_subscription(self._sub)
            self._sub = None

    def _trigger_callback(self, msg: Bool) -> None:
        """Set trigger flag when a True message is received."""
        if msg.data:
            self._triggered = True


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('wait_for_trigger_standalone')
    node.get_logger().info('WaitForTrigger standalone node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
