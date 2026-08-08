"""Unit tests for Python behavior modules — pure logic, no ROS.

These tests validate the core logic of each behavior by instantiating
them directly and simulating the context updates that the manager
would normally provide. No ROS node, no publisher, no subscriber.
"""

import sys
from unittest.mock import MagicMock
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['nav_msgs.msg'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()

import json
import math

import pytest

from acroba_behaviors_py.base_behavior import BehaviorContext, Status
from acroba_behaviors_py.move_to_waypoint import MoveToWaypoint
from acroba_behaviors_py.rotate_in_place import RotateInPlace
from acroba_behaviors_py.return_to_home import ReturnToHome
from acroba_behaviors_py.patrol_loop import PatrolLoop
from acroba_behaviors_py.search_pattern import SearchPattern
from acroba_behaviors_py.dock_at_station import DockAtStation
from acroba_behaviors_py.formation_hold import FormationHold


# ── Helpers ─────────────────────────────────────────────────

def make_context(**params: str) -> BehaviorContext:
    """Create a BehaviorContext with string parameters."""
    return BehaviorContext(parameters=params)


def simulate_ticks(behavior, context, max_ticks=500):
    """Tick a behavior until it finishes or max_ticks is reached."""
    for i in range(max_ticks):
        status = behavior.on_tick(context)
        if status != Status.RUNNING:
            return status, i + 1
    return Status.RUNNING, max_ticks


# ── MoveToWaypoint tests ───────────────────────────────────

class TestMoveToWaypoint:
    def test_succeeds_when_at_target(self):
        b = MoveToWaypoint()
        ctx = make_context(target_x='1.0', target_y='0.0', target_theta='0.0')
        ctx.robot_pose = (1.0, 0.0, 0.0)
        b.on_enter(ctx)
        status = b.on_tick(ctx)
        assert status == Status.SUCCEEDED

    def test_running_when_far_from_target(self):
        b = MoveToWaypoint()
        ctx = make_context(target_x='5.0', target_y='5.0')
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        status = b.on_tick(ctx)
        assert status == Status.RUNNING

    def test_running_without_odometry(self):
        b = MoveToWaypoint()
        ctx = make_context(target_x='1.0', target_y='0.0')
        ctx.robot_pose = None
        b.on_enter(ctx)
        status = b.on_tick(ctx)
        assert status == Status.RUNNING

    def test_converges_to_target(self):
        b = MoveToWaypoint()
        ctx = make_context(
            target_x='2.0', target_y='0.0', target_theta='0.0',
            distance_tolerance='0.5'
        )
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)

        # Simulate approaching by moving robot_pose closer each tick
        for i in range(20):
            x = min(2.0, 0.1 * (i + 1))
            ctx.robot_pose = (x, 0.0, 0.0)
            status = b.on_tick(ctx)
            if status == Status.SUCCEEDED:
                break
        assert status == Status.SUCCEEDED


# ── RotateInPlace tests ─────────────────────────────────────

class TestRotateInPlace:
    def test_succeeds_when_aligned(self):
        b = RotateInPlace()
        ctx = make_context(target_theta='1.57', heading_tolerance='0.1')
        ctx.robot_pose = (0.0, 0.0, 1.57)
        b.on_enter(ctx)
        status = b.on_tick(ctx)
        assert status == Status.SUCCEEDED

    def test_running_when_misaligned(self):
        b = RotateInPlace()
        ctx = make_context(target_theta='3.14')
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        status = b.on_tick(ctx)
        assert status == Status.RUNNING


# ── ReturnToHome tests ──────────────────────────────────────

class TestReturnToHome:
    def test_records_home_pose(self):
        b = ReturnToHome()
        ctx = make_context()
        ctx.robot_pose = (3.0, 4.0, 1.0)
        b.on_enter(ctx)
        assert ctx.shared_data['home_pose'] == (3.0, 4.0, 1.0)

    def test_succeeds_when_at_home(self):
        b = ReturnToHome()
        ctx = make_context(distance_tolerance='0.2')
        ctx.robot_pose = (0.0, 0.0, 0.0)
        ctx.shared_data['home_pose'] = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        status = b.on_tick(ctx)
        assert status == Status.SUCCEEDED

    def test_running_when_away_from_home(self):
        b = ReturnToHome()
        ctx = make_context()
        ctx.robot_pose = (5.0, 5.0, 0.0)
        ctx.shared_data['home_pose'] = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        status = b.on_tick(ctx)
        assert status == Status.RUNNING


# ── PatrolLoop tests ────────────────────────────────────────

class TestPatrolLoop:
    def test_loads_waypoints(self):
        b = PatrolLoop()
        wps = json.dumps([[1, 0, 0], [2, 0, 0]])
        ctx = make_context(waypoints=wps)
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        assert len(b._waypoints) == 2

    def test_advances_through_waypoints(self):
        b = PatrolLoop()
        wps = json.dumps([[0.05, 0, 0], [0.1, 0, 0]])
        ctx = make_context(waypoints=wps, distance_tolerance='0.3', loop_count='1')
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        # Robot is within tolerance of first waypoint
        status = b.on_tick(ctx)
        assert b._current_idx >= 1 or status == Status.RUNNING

    def test_completes_after_configured_loops(self):
        b = PatrolLoop()
        wps = json.dumps([[0.0, 0.0, 0.0]])
        ctx = make_context(waypoints=wps, distance_tolerance='1.0', loop_count='1')
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        status = b.on_tick(ctx)
        assert status == Status.SUCCEEDED


# ── SearchPattern tests ─────────────────────────────────────

class TestSearchPattern:
    def test_generates_lawnmower_waypoints(self):
        b = SearchPattern()
        ctx = make_context(
            pattern='lawnmower', area_width='10.0',
            area_height='10.0', spacing='5.0'
        )
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        assert len(b._waypoints) > 0

    def test_generates_spiral_waypoints(self):
        b = SearchPattern()
        ctx = make_context(
            pattern='spiral', area_width='10.0',
            area_height='10.0', spacing='3.0'
        )
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        assert len(b._waypoints) > 0

    def test_succeeds_after_covering_area(self):
        b = SearchPattern()
        ctx = make_context(
            pattern='lawnmower', area_width='1.0',
            area_height='0.5', spacing='0.5', distance_tolerance='20.0'
        )
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        # With huge tolerance, all waypoints should be "reached" immediately
        status, ticks = simulate_ticks(b, ctx, max_ticks=50)
        assert status == Status.SUCCEEDED


# ── DockAtStation tests ─────────────────────────────────────

class TestDockAtStation:
    def test_initial_phase_is_approach(self):
        b = DockAtStation()
        ctx = make_context(station_x='5.0', station_y='0.0')
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        assert b._phase == 'approach'

    def test_succeeds_when_docked(self):
        b = DockAtStation()
        ctx = make_context(
            station_x='0.0', station_y='0.0',
            station_theta='0.0', final_distance='0.1'
        )
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        b._phase = 'dock'
        status = b.on_tick(ctx)
        assert status == Status.SUCCEEDED


# ── FormationHold tests ─────────────────────────────────────

class TestFormationHold:
    def test_running_without_reference(self):
        b = FormationHold()
        ctx = make_context()
        ctx.robot_pose = (0.0, 0.0, 0.0)
        b.on_enter(ctx)
        status = b.on_tick(ctx)
        assert status == Status.RUNNING

    def test_holds_position_when_in_formation(self):
        b = FormationHold()
        ctx = make_context(
            offset_x='1.0', offset_y='0.0', distance_tolerance='0.3'
        )
        ctx.robot_pose = (1.0, 0.0, 0.0)
        b.on_enter(ctx)
        # Simulate reference at origin facing forward (theta=0)
        b._reference_pose = (0.0, 0.0, 0.0)
        status = b.on_tick(ctx)
        # Should be RUNNING (holds indefinitely) and in position
        assert status == Status.RUNNING
        assert 'held' in ctx.shared_data.get('detail', '').lower()
