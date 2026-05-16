"""Mission metrics collector (Phase 4, task 4.2).

A passive observer — it subscribes only, never publishes, so it cannot
perturb the frozen Phase-3 system. For every autonomous mission it records
one CSV row:

    timestamp,world,run_index,mission_result,coverage_pct,collisions,duration_s,distance_m

Metrics
-------
* coverage_pct  — glass surface discretised into ``cell_size`` cells; every
  cell within ``footprint_radius`` (the 0.12 m vacuum-pad radius, *not* the
  0.18 m chassis radius — coverage means the cleaning head passed over it)
  of the robot is marked visited while the mission is RUNNING. Percentage is
  over the *real* glass surface bounds, not the inset planner bounds, so an
  uncleaned wall-adjacent strip is honestly reflected.
* collisions    — number of distinct periods where ``min(/robot/scan)`` drops
  below ``collision_range_threshold`` (0.05 m). Edge-counted with a clear
  hysteresis so one sustained graze is one event, mirroring the controller's
  1.0 s EMERGENCY hysteresis.
* duration_s    — sim-clock seconds between the first RUNNING and the
  terminal DONE/ABORTED/TIMEOUT (sim clock because Gazebo runs below 1.0 RTF
  on Apple Silicon — wall-clock would be wrong).
* distance_m    — cumulative ``/odom`` position deltas while RUNNING.

Lifecycle
---------
PRE_RUN -> RUNNING -> FINALIZED, driven by ``/control/mission_state``
(VOLATILE depth-10 publisher in path_follower, re-published at 1 Hz, so a
late start still catches the state). ABORTED runs are recorded — most Phase-3
runs end ABORTED and those rows are the real dataset. A watchdog finalises a
stuck mission as TIMEOUT so an unattended benchmark never hangs. On finalize
the node appends the CSV row, writes ``<results_dir>/.run_complete`` (the
synchronisation primitive ``run_benchmark.sh`` polls) and exits.

The pure functions below carry all the maths so ``test/test_metrics.py``
can exercise them without a live ROS context (same pattern as
``test_coverage_planner.py`` / ``test_state_machine.py``).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from geometry_msgs.msg import Point  # noqa: F401  (kept for type clarity)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# Mission lifecycle phases (internal to the metrics node).
PHASE_PRE_RUN = "PRE_RUN"
PHASE_RUNNING = "RUNNING"
PHASE_FINALIZED = "FINALIZED"

# Terminal mission_state values published by path_follower.
TERMINAL_STATES = ("DONE", "ABORTED")

CSV_HEADER = (
    "timestamp,world,run_index,mission_result,"
    "coverage_pct,collisions,duration_s,distance_m"
)


# --------------------------------------------------------------------------- #
# Pure functions — no ROS, no self; unit-tested directly.
# --------------------------------------------------------------------------- #
def grid_dims(min_x: float, max_x: float, min_y: float, max_y: float,
              res: float) -> Tuple[int, int]:
    """Return (nx, ny) cell counts for the given bounds and resolution."""
    if res <= 0.0:
        raise ValueError("cell_size must be > 0")
    if max_x <= min_x or max_y <= min_y:
        raise ValueError("bounds must satisfy max > min on both axes")
    nx = int(round((max_x - min_x) / res))
    ny = int(round((max_y - min_y) / res))
    return nx, ny


def make_coverage_grid(min_x: float, max_x: float, min_y: float, max_y: float,
                        res: float) -> np.ndarray:
    """Boolean visited grid of shape (ny, nx), all False."""
    nx, ny = grid_dims(min_x, max_x, min_y, max_y, res)
    return np.zeros((ny, nx), dtype=bool)


def disk_offsets(footprint_radius: float, res: float) -> List[Tuple[int, int]]:
    """Precompute (drow, dcol) offsets whose cell centre lies within the
    footprint radius (metres). Computed once at startup."""
    if footprint_radius <= 0.0:
        return [(0, 0)]
    reach = int(math.ceil(footprint_radius / res))
    r2 = footprint_radius * footprint_radius
    out: List[Tuple[int, int]] = []
    for dr in range(-reach, reach + 1):
        for dc in range(-reach, reach + 1):
            if (dr * res) ** 2 + (dc * res) ** 2 <= r2:
                out.append((dr, dc))
    return out


def world_to_cell(x: float, y: float, min_x: float, min_y: float,
                   res: float) -> Tuple[int, int]:
    """World metres -> (col, row). Caller clamps/range-checks."""
    col = int((x - min_x) / res)
    row = int((y - min_y) / res)
    return col, row


def mark_visited(grid: np.ndarray, x: float, y: float, min_x: float,
                 min_y: float, res: float,
                 offsets: List[Tuple[int, int]]) -> None:
    """Stamp the footprint disc centred at (x, y) into ``grid`` (mutates).
    Cells outside the grid are silently skipped (robot at a corner)."""
    ny, nx = grid.shape
    col, row = world_to_cell(x, y, min_x, min_y, res)
    for dr, dc in offsets:
        r = row + dr
        c = col + dc
        if 0 <= r < ny and 0 <= c < nx:
            grid[r, c] = True


def coverage_pct(grid: np.ndarray,
                 blocked: Optional[np.ndarray] = None) -> float:
    """Percentage of countable cells marked visited.

    ``blocked`` (same shape, True = obstacle cell the robot cannot clean) is
    removed from both numerator and denominator. None => every cell counts.
    """
    if blocked is None:
        total = grid.size
        seen = int(grid.sum())
    else:
        countable = ~blocked
        total = int(countable.sum())
        seen = int((grid & countable).sum())
    if total == 0:
        return 0.0
    return 100.0 * seen / total


def make_blocked_grid(min_x: float, max_x: float, min_y: float, max_y: float,
                      res: float, obstacles_flat: List[float]) -> np.ndarray:
    """Boolean (ny, nx) grid, True where an obstacle AABB covers the cell.

    ``obstacles_flat`` is [xmin0, ymin0, xmax0, ymax0, xmin1, ...] in the
    same convention as ``coverage_planner.obstacles_flat``. Empty => all
    False (basic / small worlds have no interior obstacles).
    """
    grid = make_coverage_grid(min_x, max_x, min_y, max_y, res)
    n = len(obstacles_flat)
    if n == 0:
        return grid
    ny, nx = grid.shape
    for i in range(0, n - 3, 4):
        oxmin, oymin, oxmax, oymax = obstacles_flat[i:i + 4]
        c0, r0 = world_to_cell(oxmin, oymin, min_x, min_y, res)
        c1, r1 = world_to_cell(oxmax, oymax, min_x, min_y, res)
        c_lo, c_hi = sorted((c0, c1))
        r_lo, r_hi = sorted((r0, r1))
        c_lo = max(0, c_lo)
        c_hi = min(nx - 1, c_hi)
        r_lo = max(0, r_lo)
        r_hi = min(ny - 1, r_hi)
        if c_lo <= c_hi and r_lo <= r_hi:
            grid[r_lo:r_hi + 1, c_lo:c_hi + 1] = True
    return grid


def accumulate_distance(prev_xy: Optional[Tuple[float, float]], x: float,
                        y: float, jump_reject: float
                        ) -> Tuple[float, Tuple[float, float]]:
    """Return (delta_to_add, new_prev_xy).

    First sample (prev None) => delta 0. A step larger than ``jump_reject``
    (teleport / odom reset artefact) contributes 0 but still advances prev.
    """
    if prev_xy is None:
        return 0.0, (x, y)
    d = math.hypot(x - prev_xy[0], y - prev_xy[1])
    if d > jump_reject:
        return 0.0, (x, y)
    return d, (x, y)


def min_finite_range(ranges, range_min: float, range_max: float) -> float:
    """Smallest valid lidar return, or +inf if none.

    Mirrors cleaning_controller._on_scan filtering: drop inf/nan and values
    outside [range_min, range_max] (a 0.0 return means 'no echo').
    """
    best = float("inf")
    for r in ranges:
        if r is None:
            continue
        if not math.isfinite(r):
            continue
        if r <= 0.0:
            continue
        if r < range_min or r > range_max:
            continue
        if r < best:
            best = r
    return best


@dataclass
class CollisionState:
    in_collision: bool = False
    count: int = 0
    clear_since: Optional[float] = None


def update_collision_fsm(state: CollisionState, min_r: float, now_s: float,
                          threshold: float, debounce_s: float) -> None:
    """Edge-count collision periods with a clear hysteresis (mutates state).

    Rising edge (clear -> breach) increments the count once. A sustained
    breach stays one event. The collision only clears after the lane has
    been continuously clear for ``debounce_s`` seconds, so threshold chatter
    does not split one physical contact into many.
    """
    breach = min_r < threshold
    if breach:
        if not state.in_collision:
            state.count += 1
            state.in_collision = True
        state.clear_since = None
        return
    # Not breaching.
    if not state.in_collision:
        state.clear_since = None
        return
    if state.clear_since is None:
        state.clear_since = now_s
    elif now_s - state.clear_since >= debounce_s:
        state.in_collision = False
        state.clear_since = None


def mission_transition(phase: str, mission_state: str
                       ) -> Tuple[str, Optional[str]]:
    """Pure lifecycle FSM step.

    Returns (new_phase, outcome) where outcome is set exactly once, on the
    transition into FINALIZED. Idempotent in FINALIZED (path_follower
    re-publishes the terminal state at 1 Hz forever).
    """
    if phase == PHASE_FINALIZED:
        return PHASE_FINALIZED, None
    if phase == PHASE_PRE_RUN:
        if mission_state == "RUNNING":
            return PHASE_RUNNING, None
        if mission_state in TERMINAL_STATES:
            # Action server never came up: WAITING -> ABORTED directly.
            return PHASE_FINALIZED, mission_state
        return PHASE_PRE_RUN, None
    # phase == RUNNING
    if mission_state in TERMINAL_STATES:
        return PHASE_FINALIZED, mission_state
    return PHASE_RUNNING, None


def format_csv_row(timestamp: str, world: str, run_index: int,
                   mission_result: str, cov_pct: float, collisions: int,
                   duration_s: float, distance_m: float) -> str:
    """Single CSV line matching CSV_HEADER. World/result are CSV-safe by
    construction (no commas), so no quoting is needed."""
    safe_world = str(world).replace(",", "_")
    safe_result = str(mission_result).replace(",", "_")
    return (
        f"{timestamp},{safe_world},{run_index},{safe_result},"
        f"{cov_pct:.2f},{collisions},{duration_s:.1f},{distance_m:.3f}"
    )


# --------------------------------------------------------------------------- #
# ROS node
# --------------------------------------------------------------------------- #
@dataclass
class _Accum:
    distance_m: float = 0.0
    prev_xy: Optional[Tuple[float, float]] = None
    collision: CollisionState = field(default_factory=CollisionState)


class MetricsNode(Node):
    def __init__(self) -> None:
        super().__init__("metrics_node")

        self.declare_parameter("glass_min_x", -2.50)
        self.declare_parameter("glass_max_x", 2.50)
        self.declare_parameter("glass_min_y", -1.50)
        self.declare_parameter("glass_max_y", 1.50)
        self.declare_parameter("cell_size", 0.05)
        self.declare_parameter("footprint_radius", 0.12)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/robot/scan")
        self.declare_parameter("mission_state_topic", "/control/mission_state")
        self.declare_parameter("collision_range_threshold", 0.05)
        self.declare_parameter("collision_clear_hysteresis_s", 1.0)
        self.declare_parameter("max_mission_duration_s", 420.0)
        self.declare_parameter("jump_reject_m", 0.5)
        self.declare_parameter("results_dir", "/workspace/results")
        self.declare_parameter("csv_filename", "metrics.csv")
        self.declare_parameter("world_name", "glass_basic")
        self.declare_parameter("run_index", 0)
        # Empty default array params infer BYTE_ARRAY in rclpy unless the
        # type is pinned explicitly (same trap fixed in coverage_planner).
        self.declare_parameter(
            "obstacles_flat",
            Parameter.Type.DOUBLE_ARRAY,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )

        g = self.get_parameter
        self._min_x = float(g("glass_min_x").value)
        self._max_x = float(g("glass_max_x").value)
        self._min_y = float(g("glass_min_y").value)
        self._max_y = float(g("glass_max_y").value)
        self._res = float(g("cell_size").value)
        self._footprint = float(g("footprint_radius").value)
        self._collision_thr = float(g("collision_range_threshold").value)
        self._collision_hyst = float(g("collision_clear_hysteresis_s").value)
        self._max_duration = float(g("max_mission_duration_s").value)
        self._jump_reject = float(g("jump_reject_m").value)
        self._results_dir = str(g("results_dir").value)
        self._csv_path = os.path.join(
            self._results_dir, str(g("csv_filename").value))
        self._flag_path = os.path.join(self._results_dir, ".run_complete")
        self._world = str(g("world_name").value)
        self._run_index = int(g("run_index").value)
        # A type-only DOUBLE_ARRAY param that is never set raises
        # ParameterUninitializedException on .value in Humble (it does NOT
        # return None) — mirror coverage_planner's guard exactly.
        try:
            obstacles_param = g("obstacles_flat").value
        except Exception:
            obstacles_param = None
        self._obstacles = [float(v) for v in (obstacles_param or [])]

        self._grid = make_coverage_grid(
            self._min_x, self._max_x, self._min_y, self._max_y, self._res)
        self._blocked = make_blocked_grid(
            self._min_x, self._max_x, self._min_y, self._max_y, self._res,
            self._obstacles)
        self._offsets = disk_offsets(self._footprint, self._res)

        self._phase = PHASE_PRE_RUN
        self._outcome: Optional[str] = None
        self._t_start = None
        self._t_end = None
        self._acc = _Accum()
        self._should_exit = False
        self._watchdog_ticks = 0
        self._watchdog_period_s = 2.0

        odom_topic = str(g("odom_topic").value)
        scan_topic = str(g("scan_topic").value)
        state_topic = str(g("mission_state_topic").value)

        # mission_state: path_follower publishes VOLATILE/RELIABLE depth 10
        # (verified in path_follower.py). Default QoS matches.
        self.create_subscription(String, state_topic, self._on_state, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        # The Gazebo lidar bridge publishes BEST_EFFORT; a RELIABLE sub
        # would never match.
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)

        # Watchdog + shutdown poller (sim clock via use_sim_time).
        self.create_timer(self._watchdog_period_s, self._on_watchdog)
        self.create_timer(0.2, self._on_exit_poll)

        nx, ny = grid_dims(self._min_x, self._max_x, self._min_y,
                           self._max_y, self._res)
        self.get_logger().info(
            f"metrics_node up — world='{self._world}' run={self._run_index} "
            f"grid={nx}x{ny}@{self._res}m footprint={self._footprint}m "
            f"csv={self._csv_path}"
        )

    # -- time helper -------------------------------------------------------- #
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- subscriptions ------------------------------------------------------ #
    def _on_state(self, msg: String) -> None:
        new_phase, outcome = mission_transition(self._phase, msg.data)
        if new_phase == PHASE_RUNNING and self._phase == PHASE_PRE_RUN:
            self._t_start = self._now_s()
            self.get_logger().info("mission RUNNING — metrics started")
        self._phase = new_phase
        if outcome is not None:
            self._finalize(outcome)

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._phase != PHASE_RUNNING:
            # Keep prev fresh so the first RUNNING delta is not a huge jump
            # from the origin; do not accumulate or mark the grid yet.
            self._acc.prev_xy = (x, y)
            return
        delta, self._acc.prev_xy = accumulate_distance(
            self._acc.prev_xy, x, y, self._jump_reject)
        self._acc.distance_m += delta
        mark_visited(self._grid, x, y, self._min_x, self._min_y,
                     self._res, self._offsets)

    def _on_scan(self, msg: LaserScan) -> None:
        if self._phase != PHASE_RUNNING:
            return
        min_r = min_finite_range(msg.ranges, msg.range_min, msg.range_max)
        update_collision_fsm(self._acc.collision, min_r, self._now_s(),
                             self._collision_thr, self._collision_hyst)

    # -- watchdog / exit ---------------------------------------------------- #
    def _on_watchdog(self) -> None:
        if self._phase == PHASE_FINALIZED:
            return
        if self._phase == PHASE_PRE_RUN:
            # The mission never reached RUNNING (Nav2/planner/follower never
            # brought it up). Tick-count instead of sim clock here — there
            # is no t_start yet. Finalise so the benchmark always gets a row
            # and the report can tell "pipeline never started" apart from
            # "ran then ABORTED".
            self._watchdog_ticks += 1
            if self._watchdog_ticks * self._watchdog_period_s > self._max_duration:
                self.get_logger().warn(
                    "mission never reached RUNNING — finalising NO_MISSION")
                self._finalize("NO_MISSION")
            return
        # PHASE_RUNNING
        if self._t_start is None:
            return
        if self._now_s() - self._t_start > self._max_duration:
            self.get_logger().warn(
                f"mission exceeded {self._max_duration:.0f}s sim time — "
                f"finalising as TIMEOUT")
            self._finalize("TIMEOUT")

    def _on_exit_poll(self) -> None:
        if self._should_exit:
            # Raising SystemExit from a timer is the rclpy-documented way to
            # break out of spin() cleanly (not calling shutdown in-callback).
            raise SystemExit

    # -- finalize ----------------------------------------------------------- #
    def _finalize(self, result: str) -> None:
        if self._phase == PHASE_FINALIZED and self._outcome is not None:
            return  # idempotent
        self._phase = PHASE_FINALIZED
        self._outcome = result
        self._t_end = self._now_s()
        duration = 0.0
        if self._t_start is not None and self._t_end is not None:
            duration = max(0.0, self._t_end - self._t_start)
        cov = coverage_pct(self._grid, self._blocked)
        collisions = self._acc.collision.count
        distance = self._acc.distance_m
        row = format_csv_row(
            datetime.now().isoformat(timespec="seconds"),
            self._world, self._run_index, result, cov, collisions,
            duration, distance)

        try:
            os.makedirs(self._results_dir, exist_ok=True)
            need_header = (not os.path.exists(self._csv_path)
                           or os.path.getsize(self._csv_path) == 0)
            with open(self._csv_path, "a", encoding="utf-8") as f:
                if need_header:
                    f.write(CSV_HEADER + "\n")
                f.write(row + "\n")
                f.flush()
                os.fsync(f.fileno())
            with open(self._flag_path, "w", encoding="utf-8") as f:
                f.write(result + "\n")
                f.flush()
                os.fsync(f.fileno())
        except OSError as exc:
            self.get_logger().error(f"failed writing metrics: {exc}")

        self.get_logger().info(
            f"FINALIZED result={result} coverage={cov:.1f}% "
            f"collisions={collisions} duration={duration:.1f}s "
            f"distance={distance:.2f}m -> {self._csv_path}")
        self._should_exit = True


def _exit_code(outcome: Optional[str]) -> int:
    return {"DONE": 0, "ABORTED": 1, "TIMEOUT": 2,
            "NO_MISSION": 3, "INTERRUPTED": 2}.get(outcome or "INTERRUPTED", 2)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # A kill mid-run still yields a CSV row (idempotent finalize).
        if node._phase != PHASE_FINALIZED:
            node._finalize("INTERRUPTED")
        code = _exit_code(node._outcome)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


__all__ = [
    "main", "MetricsNode", "grid_dims", "make_coverage_grid",
    "disk_offsets", "world_to_cell", "mark_visited", "coverage_pct",
    "make_blocked_grid", "accumulate_distance", "min_finite_range",
    "CollisionState", "update_collision_fsm", "mission_transition",
    "format_csv_row", "CSV_HEADER",
]
