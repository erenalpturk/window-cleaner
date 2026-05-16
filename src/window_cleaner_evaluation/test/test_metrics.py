"""Unit tests for the metrics node's pure helper functions.

Mirrors test_coverage_planner.py: pytest, import from the installed
package, no live ROS context (the maths lives in module-level functions).
"""

import math

import numpy as np
import pytest

from window_cleaner_evaluation.metrics_node import (
    CSV_HEADER,
    CollisionState,
    accumulate_distance,
    coverage_pct,
    disk_offsets,
    format_csv_row,
    grid_dims,
    make_blocked_grid,
    make_coverage_grid,
    mark_visited,
    min_finite_range,
    mission_transition,
    update_collision_fsm,
    world_to_cell,
)


# --------------------------------------------------------------------------- #
# Coverage grid
# --------------------------------------------------------------------------- #
def test_grid_dims_basic_world():
    # (2.50 - (-2.50)) / 0.05 = 100 ; (1.50 - (-1.50)) / 0.05 = 60
    assert grid_dims(-2.50, 2.50, -1.50, 1.50, 0.05) == (100, 60)


def test_grid_dims_small_world():
    assert grid_dims(-1.00, 1.00, -0.50, 0.50, 0.05) == (40, 20)


def test_grid_dims_rejects_bad_bounds():
    with pytest.raises(ValueError):
        grid_dims(0.0, 0.0, -1.0, 1.0, 0.05)
    with pytest.raises(ValueError):
        grid_dims(-1.0, 1.0, -1.0, 1.0, 0.0)


def test_fresh_grid_is_zero_percent():
    g = make_coverage_grid(-2.50, 2.50, -1.50, 1.50, 0.05)
    assert g.shape == (60, 100)
    assert coverage_pct(g) == 0.0


def test_full_grid_is_hundred_percent():
    g = make_coverage_grid(-1.0, 1.0, -0.5, 0.5, 0.05)
    g[:] = True
    assert coverage_pct(g) == pytest.approx(100.0)


def test_world_to_cell_corner_and_monotonic():
    # Min corner is always (0, 0) (robust: (min - min)/res == 0). Exact
    # interior indices are float-fragile at cell boundaries, so assert the
    # robust properties instead: in-range and monotone non-decreasing in x/y.
    assert world_to_cell(-2.50, -1.50, -2.50, -1.50, 0.05) == (0, 0)
    c_lo, r_lo = world_to_cell(-2.40, -1.40, -2.50, -1.50, 0.05)
    c_hi, r_hi = world_to_cell(2.40, 1.40, -2.50, -1.50, 0.05)
    assert 0 <= c_lo <= c_hi <= 100
    assert 0 <= r_lo <= r_hi <= 60


def test_disk_offsets_radius_and_membership():
    offs = disk_offsets(0.12, 0.05)
    # Centre always included; everything within the radius, nothing outside.
    assert (0, 0) in offs
    for dr, dc in offs:
        assert (dr * 0.05) ** 2 + (dc * 0.05) ** 2 <= 0.12 ** 2 + 1e-9
    assert (3, 0) not in offs  # 0.15 m > 0.12 m


def test_disk_offsets_zero_radius_is_single_cell():
    assert disk_offsets(0.0, 0.05) == [(0, 0)]


def test_mark_visited_stamps_disc_and_is_monotonic():
    g = make_coverage_grid(-1.0, 1.0, -0.5, 0.5, 0.05)
    offs = disk_offsets(0.12, 0.05)
    before = coverage_pct(g)
    mark_visited(g, 0.0, 0.0, -1.0, -0.5, 0.05, offs)
    after = coverage_pct(g)
    assert after > before
    assert g.sum() == len(offs)  # whole disc fits in the interior
    # Marking again does not un-mark anything (monotonic).
    mark_visited(g, 0.0, 0.0, -1.0, -0.5, 0.05, offs)
    assert coverage_pct(g) == pytest.approx(after)


def test_mark_visited_clamps_at_corner_without_error():
    g = make_coverage_grid(-1.0, 1.0, -0.5, 0.5, 0.05)
    offs = disk_offsets(0.12, 0.05)
    # Robot exactly at the min corner — half the disc is out of bounds.
    mark_visited(g, -1.0, -0.5, -1.0, -0.5, 0.05, offs)
    assert g[0, 0]
    assert 0 < g.sum() < len(offs)


def test_coverage_with_blocked_cells_excluded_from_denominator():
    bounds = (-1.0, 1.0, -0.5, 0.5, 0.05)
    blocked = make_blocked_grid(*bounds, [-0.1, -0.5, 0.1, 0.5])
    assert blocked.any()
    g = make_coverage_grid(*bounds)
    # Visit everything that is not blocked → 100 %.
    g[~blocked] = True
    assert coverage_pct(g, blocked) == pytest.approx(100.0)


def test_empty_obstacles_block_nothing():
    bounds = (-1.0, 1.0, -0.5, 0.5, 0.05)
    blocked = make_blocked_grid(*bounds, [])
    assert not blocked.any()


# --------------------------------------------------------------------------- #
# Distance
# --------------------------------------------------------------------------- #
def test_first_sample_has_zero_delta():
    delta, prev = accumulate_distance(None, 1.0, 2.0, 0.5)
    assert delta == 0.0
    assert prev == (1.0, 2.0)


def test_straight_line_sums_to_n_times_step():
    prev = None
    total = 0.0
    for i in range(11):  # 0.0 .. 1.0 in 0.1 steps along x
        d, prev = accumulate_distance(prev, i * 0.1, 0.0, 0.5)
        total += d
    assert total == pytest.approx(1.0, abs=1e-9)


def test_closed_loop_is_path_length_not_displacement():
    pts = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]
    prev = None
    total = 0.0
    for x, y in pts:
        d, prev = accumulate_distance(prev, float(x), float(y), 2.0)
        total += d
    assert total == pytest.approx(4.0)  # perimeter, not 0


def test_teleport_jump_is_rejected_but_prev_advances():
    d, prev = accumulate_distance((0.0, 0.0), 5.0, 0.0, 0.5)
    assert d == 0.0
    assert prev == (5.0, 0.0)


# --------------------------------------------------------------------------- #
# Lidar / collision FSM
# --------------------------------------------------------------------------- #
def test_min_finite_range_ignores_inf_nan_zero_and_out_of_band():
    ranges = [float("inf"), float("nan"), 0.0, 0.02, 0.3, 99.0]
    # range_min 0.05 drops the 0.02; range_max 5.0 drops the 99.0.
    assert min_finite_range(ranges, 0.05, 5.0) == pytest.approx(0.3)


def test_min_finite_range_empty_is_inf():
    assert min_finite_range([], 0.05, 5.0) == float("inf")


def test_single_dip_counts_once_and_clears_after_debounce():
    s = CollisionState()
    update_collision_fsm(s, 0.03, 0.0, 0.05, 1.0)  # breach
    assert s.count == 1 and s.in_collision
    update_collision_fsm(s, 0.20, 0.2, 0.05, 1.0)  # clear, within debounce
    assert s.in_collision  # not cleared yet
    update_collision_fsm(s, 0.20, 1.5, 0.05, 1.0)  # clear long enough
    assert not s.in_collision and s.count == 1


def test_sustained_breach_counts_once():
    s = CollisionState()
    for t in range(20):
        update_collision_fsm(s, 0.01, float(t) * 0.1, 0.05, 1.0)
    assert s.count == 1


def test_two_distinct_collisions_count_twice():
    s = CollisionState()
    update_collision_fsm(s, 0.01, 0.0, 0.05, 1.0)   # collision 1
    update_collision_fsm(s, 0.50, 0.1, 0.05, 1.0)   # clear starts
    update_collision_fsm(s, 0.50, 2.0, 0.05, 1.0)   # cleared
    update_collision_fsm(s, 0.01, 3.0, 0.05, 1.0)   # collision 2
    assert s.count == 2


def test_chatter_within_debounce_coalesces_to_one():
    s = CollisionState()
    update_collision_fsm(s, 0.01, 0.0, 0.05, 1.0)   # breach -> count 1
    update_collision_fsm(s, 0.50, 0.2, 0.05, 1.0)   # brief clear
    update_collision_fsm(s, 0.01, 0.4, 0.05, 1.0)   # breach again < debounce
    update_collision_fsm(s, 0.50, 0.6, 0.05, 1.0)
    update_collision_fsm(s, 0.50, 2.0, 0.05, 1.0)   # finally cleared
    assert s.count == 1


# --------------------------------------------------------------------------- #
# Mission lifecycle FSM
# --------------------------------------------------------------------------- #
def test_pre_run_stays_on_waiting():
    assert mission_transition("PRE_RUN", "WAITING") == ("PRE_RUN", None)


def test_pre_run_to_running_no_outcome():
    assert mission_transition("PRE_RUN", "RUNNING") == ("RUNNING", None)


def test_running_to_done_and_aborted():
    assert mission_transition("RUNNING", "DONE") == ("FINALIZED", "DONE")
    assert mission_transition("RUNNING", "ABORTED") == ("FINALIZED", "ABORTED")


def test_finalized_is_idempotent():
    assert mission_transition("FINALIZED", "DONE") == ("FINALIZED", None)
    assert mission_transition("FINALIZED", "ABORTED") == ("FINALIZED", None)


def test_direct_waiting_to_aborted_finalizes():
    # path_follower emits ABORTED from WAITING if the action server never
    # comes up — must finalize, not get stuck in PRE_RUN.
    assert mission_transition("PRE_RUN", "ABORTED") == ("FINALIZED", "ABORTED")


# --------------------------------------------------------------------------- #
# CSV formatting
# --------------------------------------------------------------------------- #
def test_csv_row_field_order_and_formatting():
    row = format_csv_row("2026-05-16T12:00:00", "glass_basic", 2, "DONE",
                          63.456, 1, 277.84, 12.3456)
    assert row == ("2026-05-16T12:00:00,glass_basic,2,DONE,"
                   "63.46,1,277.8,12.346")
    assert len(row.split(",")) == len(CSV_HEADER.split(","))


def test_csv_row_is_comma_safe():
    row = format_csv_row("ts", "wo,rld", 1, "AB,ORTED", 0.0, 0, 0.0, 0.0)
    # Embedded commas in world/result are sanitised so the column count holds.
    assert len(row.split(",")) == len(CSV_HEADER.split(","))
