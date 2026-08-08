"""Unit tests for BaseBehavior interface contract.

These tests validate the base behavior ABC, BehaviorContext, and Status
enum without any ROS dependencies — pure Python only.
"""

import pytest

from acroba_behaviors_py.base_behavior import (
    BaseBehavior,
    BehaviorContext,
    Status,
)


# ── Concrete test implementation ────────────────────────────

class DummyBehavior(BaseBehavior):
    """Minimal concrete implementation for testing the ABC."""

    def __init__(self, name: str = 'dummy', description: str = 'test') -> None:
        super().__init__(name, description)
        self.entered = False
        self.ticked = 0
        self.exited = False
        self._tick_status = Status.RUNNING

    def set_tick_status(self, status: Status) -> None:
        self._tick_status = status

    def on_enter(self, context: BehaviorContext) -> None:
        self.entered = True

    def on_tick(self, context: BehaviorContext) -> Status:
        self.ticked += 1
        return self._tick_status

    def on_exit(self, context: BehaviorContext) -> None:
        self.exited = True


# ── Status enum tests ───────────────────────────────────────

class TestStatus:
    def test_status_values(self):
        assert Status.IDLE == 0
        assert Status.RUNNING == 1
        assert Status.SUCCEEDED == 2
        assert Status.FAILED == 3
        assert Status.PREEMPTED == 4

    def test_status_is_comparable(self):
        assert Status.IDLE < Status.RUNNING
        assert Status.SUCCEEDED > Status.RUNNING

    def test_status_name(self):
        assert Status.RUNNING.name == 'RUNNING'
        assert Status.PREEMPTED.name == 'PREEMPTED'


# ── BehaviorContext tests ───────────────────────────────────

class TestBehaviorContext:
    def test_default_context(self):
        ctx = BehaviorContext()
        assert ctx.parameters == {}
        assert ctx.shared_data == {}
        assert ctx.robot_pose is None
        assert ctx.scan_ranges == []

    def test_context_with_data(self):
        ctx = BehaviorContext(
            parameters={'speed': '0.5'},
            robot_pose=(1.0, 2.0, 0.5),
            scan_ranges=[1.0, 2.0, 3.0],
        )
        assert ctx.parameters['speed'] == '0.5'
        assert ctx.robot_pose == (1.0, 2.0, 0.5)
        assert len(ctx.scan_ranges) == 3

    def test_context_shared_data_mutability(self):
        ctx = BehaviorContext()
        ctx.shared_data['counter'] = 0
        ctx.shared_data['counter'] += 1
        assert ctx.shared_data['counter'] == 1


# ── BaseBehavior tests ──────────────────────────────────────

class TestBaseBehavior:
    def test_cannot_instantiate_abc(self):
        with pytest.raises(TypeError):
            BaseBehavior('test')  # type: ignore[abstract]

    def test_initial_state(self):
        b = DummyBehavior()
        assert b.name == 'dummy'
        assert b.description == 'test'
        assert b.status == Status.IDLE

    def test_lifecycle_enter(self):
        b = DummyBehavior()
        ctx = BehaviorContext()
        b.on_enter(ctx)
        assert b.entered is True

    def test_lifecycle_tick(self):
        b = DummyBehavior()
        ctx = BehaviorContext()
        result = b.on_tick(ctx)
        assert result == Status.RUNNING
        assert b.ticked == 1

    def test_lifecycle_tick_succeeded(self):
        b = DummyBehavior()
        b.set_tick_status(Status.SUCCEEDED)
        ctx = BehaviorContext()
        result = b.on_tick(ctx)
        assert result == Status.SUCCEEDED

    def test_lifecycle_exit(self):
        b = DummyBehavior()
        ctx = BehaviorContext()
        b.on_exit(ctx)
        assert b.exited is True

    def test_full_lifecycle(self):
        b = DummyBehavior()
        ctx = BehaviorContext(parameters={'key': 'value'})

        b.on_enter(ctx)
        assert b.entered

        b.status = Status.RUNNING
        result = b.on_tick(ctx)
        assert result == Status.RUNNING
        assert b.ticked == 1

        b.set_tick_status(Status.SUCCEEDED)
        result = b.on_tick(ctx)
        assert result == Status.SUCCEEDED
        assert b.ticked == 2

        b.on_exit(ctx)
        assert b.exited

    def test_reset(self):
        b = DummyBehavior()
        b.status = Status.SUCCEEDED
        assert b.status == Status.SUCCEEDED
        b.reset()
        assert b.status == Status.IDLE

    def test_repr(self):
        b = DummyBehavior(name='patrol')
        assert 'DummyBehavior' in repr(b)
        assert 'patrol' in repr(b)
        assert 'IDLE' in repr(b)

    def test_multiple_ticks(self):
        b = DummyBehavior()
        ctx = BehaviorContext()
        for _ in range(10):
            b.on_tick(ctx)
        assert b.ticked == 10
