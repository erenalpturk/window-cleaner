"""Deterministic waypoint follower — drives the coverage path without Nav2.

Rationale (see PROJECT_ROADMAP Phase 3 AI Notes, plan change 2026-05-16):
the world is an empty, gravity-free planar abstraction with no dynamic
obstacles, visual-only dirt, static known walls and ground-truth odometry
from the Ignition DiffDrive plugin. Nav2's reactive stack (costmaps, RPP
carrot pursuit, behaviour tree, replanning, collision checking) is the sole
source of the path-following instability observed during retest and is
overkill here. This node replaces Nav2 + path_follower for driving with a
classic turn-then-drive controller:

    for each waypoint:
        rotate in place toward it until the heading error is small
        drive straight until within the position tolerance
        advance to the next waypoint

It consumes the SAME ``/planning/coverage_path`` produced by
``coverage_planner`` and preserves the downstream contract: it publishes
``/control/mission_state`` (WAITING -> RUNNING -> DONE) and ``/cmd_vel``,
so ``cleaning_controller`` and the evaluation layer are unaffected. The
Nav2 path (``nav2.launch.py`` + ``path_follower`` + custom BT) is retained
in the repo as the documented advanced/alternative mode.

Because consecutive strips share an x endpoint, the transition between
strip N and strip N+1 is a pure perpendicular hop: the controller naturally
performs turn ~90 deg -> advance ~strip spacing -> turn ~90 deg, i.e. the
clean two-stage U-turn, without needing explicit connector waypoints.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


STATE_WAITING = "WAITING"
STATE_RUNNING = "RUNNING"
STATE_DONE = "DONE"


def _normalize_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract the planar yaw from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class WaypointFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower")

        # --- Tunables (proportional turn-then-drive). Defaults mirror the
        #     previous Nav2 limits so motion speed/feel is unchanged. ---
        self.declare_parameter("max_linear_vel", 0.15)       # m/s
        self.declare_parameter("max_angular_vel", 1.0)       # rad/s
        # Above this heading error -> rotate in place; below -> drive.
        self.declare_parameter("turn_threshold", 0.15)       # rad (~8.6 deg)
        self.declare_parameter("heading_kp", 2.0)            # in-place turn gain
        self.declare_parameter("drive_heading_kp", 1.2)      # gentle gain while driving
        self.declare_parameter("linear_kp", 1.5)             # approach gain
        # A waypoint counts as reached within this radius. Must stay well
        # below the strip spacing (~0.29 m) so the controller never snaps
        # onto an adjacent strip.
        self.declare_parameter("position_tolerance", 0.12)   # m
        self.declare_parameter("goal_position_tolerance", 0.15)  # m (final)
        self.declare_parameter("control_frequency", 20.0)    # Hz
        self.declare_parameter("linear_accel_limit", 0.5)    # m/s^2 (slew)
        self.declare_parameter("stop_on_done", True)
        # The coverage path is published in the `map`/world frame (first
        # waypoint ~(-2.15, -1.15)). Without Nav2 there is no `map -> odom`
        # static TF, and the Ignition DiffDrive `/odom` is reported
        # RELATIVE TO THE SPAWN POINT (robot starts at odom (0, 0)). We
        # therefore shift each waypoint by the known, static spawn offset
        # so it lands in the odom frame: wp_odom = wp_map - frame_offset.
        # Defaults equal sim.launch.py's spawn x/y (= the old nav2.launch.py
        # odom_offset_x/y). Override per-world (e.g. glass_small).
        self.declare_parameter("frame_offset_x", -2.15)
        self.declare_parameter("frame_offset_y", -1.15)

        g = self.get_parameter
        self._v_max = float(g("max_linear_vel").value)
        self._w_max = float(g("max_angular_vel").value)
        self._turn_thresh = float(g("turn_threshold").value)
        self._k_head = float(g("heading_kp").value)
        self._k_drive_head = float(g("drive_heading_kp").value)
        self._k_lin = float(g("linear_kp").value)
        self._pos_tol = float(g("position_tolerance").value)
        self._goal_tol = float(g("goal_position_tolerance").value)
        self._rate = float(g("control_frequency").value)
        self._accel_lim = float(g("linear_accel_limit").value)
        self._stop_on_done = bool(g("stop_on_done").value)
        self._off_x = float(g("frame_offset_x").value)
        self._off_y = float(g("frame_offset_y").value)

        self._waypoints: list[tuple[float, float]] = []
        self._idx = 0
        self._have_path = False
        self._pose = None  # (x, y, yaw)
        self._state = STATE_WAITING
        self._last_v = 0.0

        # Match coverage_planner's latched (TRANSIENT_LOCAL) publisher so a
        # late start still receives the one-shot path.
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Path, "/planning/coverage_path", self._on_path, path_qos
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "/control/mission_state", 10)

        self._dt = 1.0 / self._rate
        self.create_timer(self._dt, self._control_step)
        self.create_timer(1.0, self._publish_state)  # heartbeat for late subs

        self._publish_state()
        self.get_logger().info(
            "waypoint_follower up (deterministic turn-then-drive); "
            "waiting for /planning/coverage_path and /odom"
        )

    # ---- subscriptions ----
    def _on_path(self, msg: Path) -> None:
        if self._have_path:
            return
        if not msg.poses:
            self.get_logger().warn("received empty Path; ignoring")
            return
        # Shift map/world waypoints into the odom frame (see frame_offset_*).
        self._waypoints = [
            (p.pose.position.x - self._off_x, p.pose.position.y - self._off_y)
            for p in msg.poses
        ]
        self._have_path = True
        self._idx = 0
        self.get_logger().info(
            f"received coverage path with {len(self._waypoints)} waypoints "
            f"(shifted by frame_offset ({self._off_x:.2f}, {self._off_y:.2f}) "
            f"into the odom frame)"
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (p.x, p.y, _yaw_from_quaternion(q.x, q.y, q.z, q.w))

    # ---- state ----
    def _set_state(self, new_state: str) -> None:
        if new_state != self._state:
            self._state = new_state
            self._publish_state()
            self.get_logger().info(f"mission_state -> {new_state}")

    def _publish_state(self) -> None:
        m = String()
        m.data = self._state
        self._state_pub.publish(m)

    def _publish_cmd(self, v: float, w: float) -> None:
        # Slew-limit linear velocity so starts/stops are not jerky.
        dv_max = self._accel_lim * self._dt
        v = max(self._last_v - dv_max, min(self._last_v + dv_max, v))
        self._last_v = v
        t = Twist()
        t.linear.x = v
        t.angular.z = max(-self._w_max, min(self._w_max, w))
        self._cmd_pub.publish(t)

    # ---- control loop ----
    def _control_step(self) -> None:
        if not self._have_path or self._pose is None:
            self._publish_cmd(0.0, 0.0)
            return

        if self._idx >= len(self._waypoints):
            if self._state != STATE_DONE:
                self._set_state(STATE_DONE)
            if self._stop_on_done:
                self._publish_cmd(0.0, 0.0)
            return

        if self._state == STATE_WAITING:
            self._set_state(STATE_RUNNING)

        rx, ry, ryaw = self._pose
        tx, ty = self._waypoints[self._idx]
        dx, dy = tx - rx, ty - ry
        dist = math.hypot(dx, dy)

        is_last = self._idx == len(self._waypoints) - 1
        reach_tol = self._goal_tol if is_last else self._pos_tol
        if dist < reach_tol:
            self._idx += 1
            return

        yaw_err = _normalize_angle(math.atan2(dy, dx) - ryaw)

        if abs(yaw_err) > self._turn_thresh:
            # Rotate in place toward the waypoint (the "turn" phase).
            w = max(-self._w_max, min(self._w_max, self._k_head * yaw_err))
            self._publish_cmd(0.0, w)
        else:
            # Drive straight with a gentle heading correction (the
            # "advance" phase). Linear velocity tapers as we approach.
            v = min(self._v_max, self._k_lin * dist)
            w = self._k_drive_head * yaw_err
            self._publish_cmd(v, w)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort stop so the robot does not coast on shutdown.
        try:
            node._cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["main", "WaypointFollowerNode"]
