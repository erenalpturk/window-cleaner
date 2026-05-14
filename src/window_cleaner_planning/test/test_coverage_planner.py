"""Unit tests for the boustrophedon coverage planner."""

import math

import pytest

from window_cleaner_planning.coverage_planner import (
    CoverageParams,
    compute_path,
    yaw_to_quaternion,
)


def _params(**overrides):
    base = dict(
        glass_min_x=-2.30,
        glass_max_x=2.30,
        glass_min_y=-1.30,
        glass_max_y=1.30,
        strip_width=0.30,
        start_corner="SW",
        # Use a coarse spacing in tests so each strip emits exactly two
        # waypoints (entry, exit), matching the simple structural asserts
        # below. The production default (0.50 m) is exercised separately.
        waypoint_spacing=10.0,
    )
    base.update(overrides)
    return CoverageParams(**base)


def test_default_strip_count_matches_world_dimensions():
    waypoints = compute_path(_params())
    # Usable y span (after margin) = (1.30 - 0.15) - (-1.30 + 0.15) = 2.30 m,
    # strip_width = 0.30 → ceil(2.30 / 0.30) + 1 = 8 + 1 = 9 strips.
    # With waypoint_spacing >> strip length, each strip = 2 points.
    assert len(waypoints) == 18  # 9 strips * 2 (entry + exit)


def test_first_waypoint_starts_at_sw_corner():
    p = _params()
    waypoints = compute_path(p)
    margin = p.strip_width / 2.0
    x0, y0, yaw0 = waypoints[0]
    assert x0 == pytest.approx(p.glass_min_x + margin)
    assert y0 == pytest.approx(p.glass_min_y + margin)
    assert yaw0 == pytest.approx(0.0)  # heading +x on first sweep


def test_strips_alternate_direction():
    p = _params()
    waypoints = compute_path(p)
    # With coarse spacing each strip = 2 points (entry, exit).
    for i in range(len(waypoints) // 2):
        entry_x = waypoints[2 * i][0]
        exit_x = waypoints[2 * i + 1][0]
        if i % 2 == 0:
            assert exit_x > entry_x, f"strip {i} should sweep east (+x)"
        else:
            assert exit_x < entry_x, f"strip {i} should sweep west (-x)"


def test_y_strictly_increases_for_sw_start():
    waypoints = compute_path(_params())
    strip_ys = sorted({round(wp[1], 6) for wp in waypoints})
    # Distinct strip y's, monotonically increasing for SW start.
    ys_in_order = []
    seen = set()
    for _, y, _ in waypoints:
        key = round(y, 6)
        if key not in seen:
            seen.add(key)
            ys_in_order.append(y)
    assert ys_in_order == sorted(ys_in_order)
    assert len(strip_ys) == 9


def test_nw_start_reverses_y_progression():
    waypoints = compute_path(_params(start_corner="NW"))
    ys_in_order = []
    seen = set()
    for _, y, _ in waypoints:
        key = round(y, 6)
        if key not in seen:
            seen.add(key)
            ys_in_order.append(y)
    assert ys_in_order == sorted(ys_in_order, reverse=True)


def test_se_start_first_sweep_goes_west():
    waypoints = compute_path(_params(start_corner="SE"))
    entry_x, _, yaw = waypoints[0]
    exit_x = waypoints[1][0]
    assert exit_x < entry_x
    assert yaw == pytest.approx(math.pi)


def test_ne_start_first_sweep_goes_west_and_y_descends():
    waypoints = compute_path(_params(start_corner="NE"))
    assert waypoints[1][0] < waypoints[0][0]
    ys_in_order = []
    seen = set()
    for _, y, _ in waypoints:
        key = round(y, 6)
        if key not in seen:
            seen.add(key)
            ys_in_order.append(y)
    assert ys_in_order == sorted(ys_in_order, reverse=True)


def test_strip_spacing_does_not_exceed_strip_width():
    p = _params()
    waypoints = compute_path(p)
    strip_ys = sorted({wp[1] for wp in waypoints})
    diffs = [b - a for a, b in zip(strip_ys, strip_ys[1:])]
    for d in diffs:
        assert d <= p.strip_width + 1e-9


def test_waypoints_stay_within_safety_margins():
    p = _params()
    waypoints = compute_path(p)
    margin = p.strip_width / 2.0
    for x, y, _ in waypoints:
        assert x >= p.glass_min_x + margin - 1e-9
        assert x <= p.glass_max_x - margin + 1e-9
        assert y >= p.glass_min_y + margin - 1e-9
        assert y <= p.glass_max_y - margin + 1e-9


def test_area_smaller_than_one_strip_falls_back_to_centreline():
    p = _params(
        glass_min_y=-0.10,
        glass_max_y=0.10,  # 0.20 m total < 0.30 m strip
    )
    waypoints = compute_path(p)
    assert len(waypoints) == 2
    assert waypoints[0][1] == pytest.approx(0.0)
    assert waypoints[1][1] == pytest.approx(0.0)


def test_invalid_strip_width_rejected():
    with pytest.raises(ValueError):
        compute_path(_params(strip_width=0.0))


def test_invalid_corner_rejected():
    with pytest.raises(ValueError):
        compute_path(_params(start_corner="MIDDLE"))


def test_inverted_bounds_rejected():
    with pytest.raises(ValueError):
        compute_path(_params(glass_min_x=2.0, glass_max_x=-2.0))


def test_invalid_waypoint_spacing_rejected():
    with pytest.raises(ValueError):
        compute_path(_params(waypoint_spacing=0.0))


def test_dense_waypoints_respect_spacing():
    """With production spacing, each strip should emit many intermediate
    waypoints whose neighbours are at most ``waypoint_spacing`` apart."""
    p = _params(waypoint_spacing=0.50)
    waypoints = compute_path(p)
    # 9 strips × ceil(4.30 / 0.50) + 1 = 9 × 10 = 90 waypoints.
    assert len(waypoints) >= 90
    # Within a single strip (constant y), neighbouring x's are <= spacing.
    by_y = {}
    for x, y, _ in waypoints:
        by_y.setdefault(round(y, 4), []).append(x)
    for xs in by_y.values():
        xs_sorted = sorted(xs)
        diffs = [b - a for a, b in zip(xs_sorted, xs_sorted[1:])]
        for d in diffs:
            assert d <= p.waypoint_spacing + 1e-6


def test_obstacle_waypoints_are_filtered_out():
    """Waypoints inside an inflated obstacle AABB are dropped."""
    p = _params(
        obstacles=((-0.50, -0.20, 0.50, 0.20),),  # 1m x 0.4m centred box
        obstacle_inflation=0.10,
        waypoint_spacing=0.50,
    )
    waypoints = compute_path(p)
    for x, y, _ in waypoints:
        # No waypoint may sit inside the inflated box (0.60 x 0.50 effective).
        inside_x = -0.60 <= x <= 0.60
        inside_y = -0.30 <= y <= 0.30
        assert not (inside_x and inside_y), (
            f"waypoint ({x:.2f}, {y:.2f}) lies inside inflated obstacle"
        )


def test_obstacles_do_not_affect_unblocked_strips():
    """Strips far from any obstacle keep all their waypoints."""
    p = _params(
        obstacles=((-0.10, -0.10, 0.10, 0.10),),  # tiny mid-glass box
        obstacle_inflation=0.10,
        waypoint_spacing=10.0,  # coarse: 2 points per strip
    )
    waypoints = compute_path(p)
    # Without obstacles we expect 18 waypoints (9 strips × 2). The tiny
    # inflated box (0.40x0.40) might drop one waypoint near y=0; verify
    # we have at least 16 (one or two of nine strips' midpoints clipped).
    assert len(waypoints) >= 16


def test_obstacle_invalid_shape_rejected():
    with pytest.raises(ValueError):
        compute_path(_params(obstacles=((0.0, 0.0, 0.0, 1.0),)))


def test_yaw_quaternion_is_unit_and_consistent():
    q = yaw_to_quaternion(math.pi / 2)
    norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
    assert norm == pytest.approx(1.0)
    assert q.z == pytest.approx(math.sin(math.pi / 4))
    assert q.w == pytest.approx(math.cos(math.pi / 4))
