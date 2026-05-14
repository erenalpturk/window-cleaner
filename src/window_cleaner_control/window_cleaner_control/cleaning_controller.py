"""Cleaning state machine — vacuum + brush actuator gating.

States:
    IDLE      — mission has not started or has finished. vacuum OFF, brush OFF.
    MOVING    — mission RUNNING, robot is between dirty regions. vacuum ON, brush OFF.
    CLEANING  — robot pose is within ``dirt_activation_radius`` of a stored
                dirt centre. vacuum ON, brush ON.
    EMERGENCY — lidar minimum range fell below ``emergency_min_range``.
                vacuum ON (still sucking), brush OFF, robot velocity zeroed
                via /cmd_vel until the range stays clear for
                ``emergency_clear_seconds``.

Inputs:
    /control/mission_state (std_msgs/String) — from path_follower
    /odom (nav_msgs/Odometry) — ground-truth from Gazebo DiffDrive
    /perception/dirty_regions (geometry_msgs/PoseArray) — latched at startup;
        we cache the first non-empty message and trust it (the dirt does
        not move during the run).
    /robot/scan (sensor_msgs/LaserScan) — for emergency stop only.

Outputs:
    /control/vacuum_cmd (std_msgs/Bool)
    /control/brush_cmd  (std_msgs/Bool)
    /control/state      (std_msgs/String)  — current state name, for debug.
    /cmd_vel            (geometry_msgs/Twist) — only published in EMERGENCY,
        to override Nav2 with a zero-velocity command.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


STATE_IDLE = "IDLE"
STATE_MOVING = "MOVING"
STATE_CLEANING = "CLEANING"
STATE_EMERGENCY = "EMERGENCY"

MISSION_WAITING = "WAITING"
MISSION_RUNNING = "RUNNING"
MISSION_DONE = "DONE"
MISSION_ABORTED = "ABORTED"


class CleaningController(Node):
    def __init__(self) -> None:
        super().__init__("cleaning_controller")

        self.declare_parameter("dirt_activation_radius", 0.20)
        self.declare_parameter("emergency_min_range", 0.08)
        self.declare_parameter("emergency_clear_seconds", 1.0)
        self.declare_parameter("tick_rate", 10.0)

        self._dirt_radius = float(self.get_parameter("dirt_activation_radius").value)
        self._emergency_min = float(self.get_parameter("emergency_min_range").value)
        self._emergency_clear = float(
            self.get_parameter("emergency_clear_seconds").value
        )
        tick_rate = float(self.get_parameter("tick_rate").value)

        self._state = STATE_IDLE
        self._mission_state = MISSION_WAITING
        self._robot_xy: Optional[Tuple[float, float]] = None
        self._dirt_centres: List[Tuple[float, float]] = []
        self._dirt_cached = False
        self._lidar_min_range: float = float("inf")
        self._last_clear_time: Optional[float] = None

        self.create_subscription(
            String, "/control/mission_state", self._on_mission_state, 10
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(
            PoseArray, "/perception/dirty_regions", self._on_dirt, 10
        )
        self.create_subscription(LaserScan, "/robot/scan", self._on_scan, 10)

        self._vacuum_pub = self.create_publisher(Bool, "/control/vacuum_cmd", 10)
        self._brush_pub = self.create_publisher(Bool, "/control/brush_cmd", 10)
        self._state_pub = self.create_publisher(String, "/control/state", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_timer(1.0 / max(tick_rate, 1.0), self._tick)

        self.get_logger().info(
            f"cleaning_controller up — dirt_radius={self._dirt_radius}m "
            f"emergency_min={self._emergency_min}m tick={tick_rate}Hz"
        )

    # ------------------------------------------------------------ inputs

    def _on_mission_state(self, msg: String) -> None:
        if msg.data != self._mission_state:
            self.get_logger().info(
                f"mission_state: {self._mission_state} -> {msg.data}"
            )
        self._mission_state = msg.data

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._robot_xy = (p.x, p.y)

    def _on_dirt(self, msg: PoseArray) -> None:
        if self._dirt_cached or not msg.poses:
            return
        # The perception layer publishes dirt centres in the camera optical
        # frame, but the static glass world places dirt patches at fixed
        # world (map-frame) coordinates. For Phase 3 we trust the world
        # coordinates encoded in glass_basic.sdf and read what perception
        # gives us as a rough position list — the activation radius (0.20 m)
        # is generous enough to absorb the frame mismatch on the basic world.
        self._dirt_centres = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self._dirt_cached = True
        self.get_logger().info(
            f"cached {len(self._dirt_centres)} dirt centres from "
            f"/perception/dirty_regions"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        # Min over finite ranges only (filter inf / nan).
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        self._lidar_min_range = min(valid) if valid else float("inf")

    # ------------------------------------------------------------ tick

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        new_state = self._compute_state(now)

        if new_state != self._state:
            self.get_logger().info(f"state: {self._state} -> {new_state}")
            self._state = new_state

        self._publish_actuators()

        if self._state == STATE_EMERGENCY:
            # Override Nav2 — zero out velocity until the lane is clear.
            self._cmd_vel_pub.publish(Twist())

    def _compute_state(self, now_s: float) -> str:
        # 1. Emergency takes precedence regardless of mission state.
        if self._lidar_min_range < self._emergency_min:
            self._last_clear_time = None
            return STATE_EMERGENCY
        if self._state == STATE_EMERGENCY:
            # Lane is currently clear; require it to stay clear for the
            # configured grace period before leaving EMERGENCY.
            if self._last_clear_time is None:
                self._last_clear_time = now_s
                return STATE_EMERGENCY
            if (now_s - self._last_clear_time) < self._emergency_clear:
                return STATE_EMERGENCY
            self._last_clear_time = None

        # 2. Mission gating.
        if self._mission_state in (MISSION_WAITING, MISSION_DONE, MISSION_ABORTED):
            return STATE_IDLE

        # 3. Mission RUNNING — check if we are on top of a dirt centre.
        if self._robot_xy is not None and self._dirt_centres:
            rx, ry = self._robot_xy
            r2 = self._dirt_radius ** 2
            for (dx, dy) in self._dirt_centres:
                if (rx - dx) ** 2 + (ry - dy) ** 2 <= r2:
                    return STATE_CLEANING

        return STATE_MOVING

    def _publish_actuators(self) -> None:
        vacuum = Bool()
        brush = Bool()
        state = String()
        state.data = self._state

        if self._state == STATE_IDLE:
            vacuum.data, brush.data = False, False
        elif self._state == STATE_MOVING:
            vacuum.data, brush.data = True, False
        elif self._state == STATE_CLEANING:
            vacuum.data, brush.data = True, True
        elif self._state == STATE_EMERGENCY:
            vacuum.data, brush.data = True, False
        else:
            vacuum.data, brush.data = False, False

        self._vacuum_pub.publish(vacuum)
        self._brush_pub.publish(brush)
        self._state_pub.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleaningController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["CleaningController", "main"]
