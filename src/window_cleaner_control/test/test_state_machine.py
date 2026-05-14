"""Smoke tests for the cleaning controller state-transition logic.

Exercises ``_compute_state`` directly without spinning ROS, by patching the
node's input fields between calls.
"""

import math
from unittest.mock import patch

import pytest

# rclpy must be importable but we do not init it; CleaningController.__init__
# touches publishers/subscribers, so we construct a stub instance instead.


def _make_controller():
    from window_cleaner_control.cleaning_controller import CleaningController

    # Bypass __init__ (which needs a live ROS context) and seed the fields
    # _compute_state reads.
    c = CleaningController.__new__(CleaningController)
    c._state = "IDLE"
    c._mission_state = "WAITING"
    c._robot_xy = None
    c._dirt_centres = []
    c._dirt_cached = False
    c._lidar_min_range = float("inf")
    c._last_clear_time = None
    c._dirt_radius = 0.20
    c._emergency_min = 0.08
    c._emergency_clear = 1.0
    return c


def test_waiting_mission_keeps_state_idle():
    c = _make_controller()
    assert c._compute_state(now_s=0.0) == "IDLE"


def test_running_with_no_dirt_nearby_is_moving():
    c = _make_controller()
    c._mission_state = "RUNNING"
    c._robot_xy = (0.0, 0.0)
    c._dirt_centres = [(1.0, 1.0)]
    assert c._compute_state(now_s=0.0) == "MOVING"


def test_running_with_dirt_under_robot_is_cleaning():
    c = _make_controller()
    c._mission_state = "RUNNING"
    c._robot_xy = (0.50, 0.50)
    c._dirt_centres = [(0.55, 0.52)]  # 0.054 m away, inside 0.20 m
    assert c._compute_state(now_s=0.0) == "CLEANING"


def test_lidar_below_threshold_triggers_emergency_even_when_idle():
    c = _make_controller()
    c._lidar_min_range = 0.05
    assert c._compute_state(now_s=0.0) == "EMERGENCY"


def test_emergency_clears_only_after_grace_period():
    c = _make_controller()
    c._state = "EMERGENCY"
    c._mission_state = "RUNNING"
    c._robot_xy = (0.0, 0.0)
    c._lidar_min_range = 1.0  # clear

    # First clear tick — still EMERGENCY, last_clear_time stamped.
    assert c._compute_state(now_s=10.0) == "EMERGENCY"
    assert c._last_clear_time == pytest.approx(10.0)

    # 0.5 s later — still inside grace window.
    assert c._compute_state(now_s=10.5) == "EMERGENCY"

    # 1.5 s later — past grace window, drop back to MOVING.
    assert c._compute_state(now_s=11.5) == "MOVING"


def test_done_mission_returns_idle():
    c = _make_controller()
    c._mission_state = "DONE"
    c._robot_xy = (0.0, 0.0)
    c._dirt_centres = [(0.05, 0.0)]  # would be CLEANING if RUNNING
    assert c._compute_state(now_s=0.0) == "IDLE"
