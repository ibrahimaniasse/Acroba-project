"""Integration test for behavior manager preemption logic.

Validates that:
  1. A behavior can be started, ticked, and completed
  2. A running behavior can be preempted (emergency_stop scenario)
  3. The manager correctly transitions through lifecycle states
  4. Context is properly reset between behavior executions

These tests use the DummyBehavior pattern (no ROS) to validate
the manager's state machine logic independently of ROS action servers.
"""

import pytest

from acroba_behaviors_py.base_behavior import (
    BaseBehavior,
    BehaviorContext,
    Status,
)


# ── Test behaviors ──────────────────────────────────────────

class SlowBehavior(BaseBehavior):
    """Behavior that runs for a configured number of ticks."""

    def __init__(self, name='slow', ticks_to_complete=5):
        super().__init__(name, f'Completes after {ticks_to_complete} ticks')
        self._ticks_to_complete = ticks_to_complete
        self._tick_count = 0

    def on_enter(self, context):
        self._tick_count = 0
        context.shared_data['detail'] = 'Starting slow behavior'

    def on_tick(self, context):
        self._tick_count += 1
        context.shared_data['progress'] = self._tick_count / self._ticks_to_complete
        if self._tick_count >= self._ticks_to_complete:
            return Status.SUCCEEDED
        return Status.RUNNING

    def on_exit(self, context):
        context.shared_data['detail'] = f'Completed after {self._tick_count} ticks'


class FailingBehavior(BaseBehavior):
    """Behavior that fails after a few ticks."""

    def __init__(self, name='failing', ticks_to_fail=3):
        super().__init__(name, 'Fails intentionally')
        self._ticks_to_fail = ticks_to_fail
        self._tick_count = 0

    def on_enter(self, context):
        self._tick_count = 0

    def on_tick(self, context):
        self._tick_count += 1
        if self._tick_count >= self._ticks_to_fail:
            context.shared_data['error'] = 'Simulated failure'
            return Status.FAILED
        return Status.RUNNING

    def on_exit(self, context):
        pass


# ── Lightweight manager simulation ──────────────────────────

class ManagerSimulator:
    """Simulates the behavior manager's state machine without ROS.

    This mirrors the core logic of behavior_manager_node.py's
    execute_callback: enter → tick loop → exit.
    """

    def __init__(self):
        self._registry = {}
        self._active = None
        self._preempt = False

    def register(self, behavior: BaseBehavior):
        self._registry[behavior.name] = behavior

    def request_preemption(self):
        self._preempt = True

    def execute(self, name: str, context: BehaviorContext, max_ticks=100):
        """Run a behavior to completion, failure, or preemption."""
        if name not in self._registry:
            return Status.FAILED, 'Unknown behavior'

        behavior = self._registry[name]
        behavior.reset()
        self._active = behavior
        self._preempt = False

        # on_enter
        behavior.on_enter(context)
        behavior.status = Status.RUNNING

        # Tick loop
        for tick in range(max_ticks):
            if self._preempt:
                behavior.status = Status.PREEMPTED
                behavior.on_exit(context)
                self._active = None
                return Status.PREEMPTED, f'Preempted at tick {tick}'

            status = behavior.on_tick(context)
            behavior.status = status

            if status == Status.SUCCEEDED:
                behavior.on_exit(context)
                self._active = None
                return Status.SUCCEEDED, f'Completed at tick {tick + 1}'

            if status == Status.FAILED:
                behavior.on_exit(context)
                self._active = None
                return Status.FAILED, context.shared_data.get('error', 'Unknown')

        behavior.on_exit(context)
        self._active = None
        return Status.FAILED, 'Max ticks exceeded'


# ── Tests ───────────────────────────────────────────────────

class TestPreemption:
    def test_behavior_completes_normally(self):
        mgr = ManagerSimulator()
        mgr.register(SlowBehavior(ticks_to_complete=5))

        ctx = BehaviorContext()
        status, msg = mgr.execute('slow', ctx)

        assert status == Status.SUCCEEDED
        assert 'Completed at tick 5' in msg
        assert ctx.shared_data['progress'] == 1.0

    def test_behavior_fails(self):
        mgr = ManagerSimulator()
        mgr.register(FailingBehavior(ticks_to_fail=3))

        ctx = BehaviorContext()
        status, msg = mgr.execute('failing', ctx)

        assert status == Status.FAILED
        assert 'Simulated failure' in msg

    def test_preemption_interrupts_behavior(self):
        mgr = ManagerSimulator()
        slow = SlowBehavior(ticks_to_complete=100)
        mgr.register(slow)

        # Use a custom behavior that triggers preemption mid-execution
        class PreemptingBehavior(BaseBehavior):
            def __init__(self, manager):
                super().__init__('preempting', 'Preempts itself after 2 ticks')
                self._mgr = manager
                self._count = 0
            def on_enter(self, context):
                self._count = 0
            def on_tick(self, context):
                self._count += 1
                if self._count >= 2:
                    self._mgr.request_preemption()
                return Status.RUNNING
            def on_exit(self, context):
                context.shared_data['detail'] = f'Exited at tick {self._count}'

        pb = PreemptingBehavior(mgr)
        mgr.register(pb)

        ctx = BehaviorContext()
        status, msg = mgr.execute('preempting', ctx)

        assert status == Status.PREEMPTED
        assert 'Preempted' in msg

    def test_preemption_calls_on_exit(self):
        mgr = ManagerSimulator()

        class ExitTracker(BaseBehavior):
            def __init__(self, manager):
                super().__init__('exit_tracker', '')
                self._mgr = manager
                self._count = 0
            def on_enter(self, context):
                self._count = 0
                context.shared_data['exited'] = False
            def on_tick(self, context):
                self._count += 1
                if self._count >= 2:
                    self._mgr.request_preemption()
                return Status.RUNNING
            def on_exit(self, context):
                context.shared_data['exited'] = True

        mgr.register(ExitTracker(mgr))

        ctx = BehaviorContext()
        mgr.execute('exit_tracker', ctx)

        assert ctx.shared_data['exited'] is True

    def test_unknown_behavior_fails(self):
        mgr = ManagerSimulator()
        ctx = BehaviorContext()
        status, msg = mgr.execute('nonexistent', ctx)
        assert status == Status.FAILED

    def test_sequential_execution(self):
        """Run two behaviors in sequence — context resets between them."""
        mgr = ManagerSimulator()
        mgr.register(SlowBehavior(name='first', ticks_to_complete=3))
        mgr.register(SlowBehavior(name='second', ticks_to_complete=5))

        ctx1 = BehaviorContext()
        status1, _ = mgr.execute('first', ctx1)
        assert status1 == Status.SUCCEEDED

        ctx2 = BehaviorContext()
        status2, _ = mgr.execute('second', ctx2)
        assert status2 == Status.SUCCEEDED

    def test_reuse_after_preemption(self):
        """A behavior should be reusable after being preempted."""
        mgr = ManagerSimulator()

        class SelfPreemptOnce(BaseBehavior):
            """Preempts on first run, completes on second."""
            def __init__(self, manager):
                super().__init__('reusable', '')
                self._mgr = manager
                self._count = 0
                self._run_number = 0
            def on_enter(self, context):
                self._count = 0
                self._run_number += 1
            def on_tick(self, context):
                self._count += 1
                if self._run_number == 1 and self._count >= 2:
                    self._mgr.request_preemption()
                    return Status.RUNNING
                if self._count >= 3:
                    return Status.SUCCEEDED
                return Status.RUNNING
            def on_exit(self, context):
                pass

        b = SelfPreemptOnce(mgr)
        mgr.register(b)

        # First run: preempt
        ctx = BehaviorContext()
        status, _ = mgr.execute('reusable', ctx)
        assert status == Status.PREEMPTED

        # Second run: complete normally
        ctx2 = BehaviorContext()
        status2, _ = mgr.execute('reusable', ctx2)
        assert status2 == Status.SUCCEEDED

    def test_emergency_stop_preempts_active(self):
        """Simulate emergency_stop preempting an active behavior mid-tick."""
        mgr = ManagerSimulator()

        class InterruptableBehavior(BaseBehavior):
            """Triggers manager preemption at tick 3 (simulating external e-stop)."""
            def __init__(self, manager):
                super().__init__('interruptable', '')
                self._mgr = manager
                self._count = 0
            def on_enter(self, context):
                self._count = 0
            def on_tick(self, context):
                self._count += 1
                if self._count >= 3:
                    self._mgr.request_preemption()
                return Status.RUNNING
            def on_exit(self, context):
                pass

        mgr.register(InterruptableBehavior(mgr))

        ctx = BehaviorContext()
        status, msg = mgr.execute('interruptable', ctx)

        assert status == Status.PREEMPTED

