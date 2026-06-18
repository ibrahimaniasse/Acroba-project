"""Base behavior interface for the Acroba modular framework.

All behaviors (Python and C++) must implement this contract:
  - on_enter(context): initialize state when the behavior is activated
  - on_tick(context): execute one cycle of the behavior logic
  - on_exit(context): clean up when the behavior finishes or is preempted

This module is ROS-independent — it contains zero ROS imports.
Behavior nodes wrap this interface and bridge it to ROS2 action servers.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Dict, Optional


class Status(IntEnum):
    """Execution status of a behavior, matching BehaviorStatus.msg constants."""

    IDLE = 0
    RUNNING = 1
    SUCCEEDED = 2
    FAILED = 3
    PREEMPTED = 4


@dataclass
class BehaviorContext:
    """Shared context passed to behaviors on each tick.

    Attributes:
        parameters: Configuration key-values loaded from behaviors.yaml
            or provided at runtime via the ExecuteBehavior action goal.
        shared_data: Mutable store for inter-tick state (e.g. accumulated
            distance, last known pose). Behaviors own their keys —
            the manager never reads or modifies this dict.
        robot_pose: Current (x, y, theta) of the robot, updated by the
            manager from odometry before each tick.
        scan_ranges: Latest LiDAR ranges, updated by the manager. Empty
            list if no LiDAR data is available yet.
    """

    parameters: Dict[str, Any] = field(default_factory=dict)
    shared_data: Dict[str, Any] = field(default_factory=dict)
    robot_pose: Optional[tuple] = None  # (x, y, theta)
    scan_ranges: list = field(default_factory=list)


class BaseBehavior(ABC):
    """Abstract base class for all Acroba framework behaviors.

    Subclasses must implement on_enter, on_tick, and on_exit.
    The behavior manager calls these methods in sequence:

        1. on_enter() — once, when the behavior is activated
        2. on_tick() — repeatedly at the manager tick rate until
           the returned status is not RUNNING
        3. on_exit() — once, after completion or preemption

    Example:
        class MoveToWaypoint(BaseBehavior):
            def on_enter(self, context):
                self._target = (
                    context.parameters.get('target_x', 0.0),
                    context.parameters.get('target_y', 0.0),
                )

            def on_tick(self, context):
                if self._reached_target(context.robot_pose):
                    return Status.SUCCEEDED
                self._publish_cmd_vel(context)
                return Status.RUNNING

            def on_exit(self, context):
                self._stop_robot()
    """

    def __init__(self, name: str, description: str = "") -> None:
        self._name = name
        self._description = description
        self._status = Status.IDLE

    @property
    def name(self) -> str:
        """Unique identifier for this behavior instance."""
        return self._name

    @property
    def description(self) -> str:
        """Human-readable description of what this behavior does."""
        return self._description

    @property
    def status(self) -> Status:
        """Current execution status."""
        return self._status

    @status.setter
    def status(self, value: Status) -> None:
        self._status = value

    @abstractmethod
    def on_enter(self, context: BehaviorContext) -> None:
        """Called once when the behavior is activated.

        Use this to read parameters, initialize internal state,
        and set up any subscriptions or publishers needed.

        Args:
            context: Shared context with parameters and robot state.
        """

    @abstractmethod
    def on_tick(self, context: BehaviorContext) -> Status:
        """Called repeatedly while the behavior is running.

        Must return a Status value:
          - RUNNING: continue ticking
          - SUCCEEDED: behavior completed successfully
          - FAILED: behavior failed, manager should handle the error

        Args:
            context: Updated context with latest robot state.

        Returns:
            Status indicating the behavior's progress.
        """

    @abstractmethod
    def on_exit(self, context: BehaviorContext) -> None:
        """Called once when the behavior finishes or is preempted.

        Use this to stop the robot, release resources, or log results.

        Args:
            context: Final context at the time of exit.
        """

    def reset(self) -> None:
        """Reset the behavior to IDLE state for reuse."""
        self._status = Status.IDLE

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name='{self._name}', status={self._status.name})"
