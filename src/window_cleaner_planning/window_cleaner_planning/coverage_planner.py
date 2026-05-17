"""Boustrophedon coverage planner.

Builds a deterministic zigzag waypoint sequence over a static rectangular
glass area and publishes it as a latched ``nav_msgs/Path`` for downstream
``path_follower`` consumption.

The path is grounded in known world geometry (declared via parameters) rather
than the live ``/perception/glass_boundary`` topic, because ``frame_detector``
publishes pixel coordinates and only ever sees a partial frame. The detector
output is monitored as a sanity check and never overrides the planned path.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PolygonStamped, PoseStamped, Quaternion
from nav_msgs.msg import Path


Waypoint = Tuple[float, float, float]  # (x, y, yaw)
Obstacle = Tuple[float, float, float, float]  # (xmin, ymin, xmax, ymax) AABB

START_CORNERS = ("SW", "SE", "NW", "NE")


@dataclass(frozen=True)
class CoverageParams:
    glass_min_x: float
    glass_max_x: float
    glass_min_y: float
    glass_max_y: float
    strip_width: float
    start_corner: str
    waypoint_spacing: float = 0.50
    # Axis-aligned bounding boxes (xmin, ymin, xmax, ymax) of known static
    # interior obstacles. Waypoints whose centres fall within an inflated
    # obstacle box are dropped, so the boustrophedon path naturally
    # detours around them.
    obstacles: Tuple[Obstacle, ...] = ()
    # Inflation applied to each obstacle AABB when filtering waypoints.
    # Should be >= robot footprint radius (0.18) + small safety margin.
    obstacle_inflation: float = 0.25

    def validate(self) -> None:
        if self.glass_max_x <= self.glass_min_x:
            raise ValueError("glass_max_x must be > glass_min_x")
        if self.glass_max_y <= self.glass_min_y:
            raise ValueError("glass_max_y must be > glass_min_y")
        if self.strip_width <= 0.0:
            raise ValueError("strip_width must be > 0")
        if self.start_corner not in START_CORNERS:
            raise ValueError(f"start_corner must be one of {START_CORNERS}")
        if self.waypoint_spacing <= 0.0:
            raise ValueError("waypoint_spacing must be > 0")
        if self.obstacle_inflation < 0.0:
            raise ValueError("obstacle_inflation must be >= 0")
        for obs in self.obstacles:
            if len(obs) != 4:
                raise ValueError(f"obstacle must be (xmin,ymin,xmax,ymax), got {obs}")
            if obs[2] <= obs[0] or obs[3] <= obs[1]:
                raise ValueError(f"obstacle has zero/negative area: {obs}")


def _point_in_inflated_obstacle(x: float, y: float,
                                obstacles: Tuple[Obstacle, ...],
                                inflation: float) -> bool:
    for (xmin, ymin, xmax, ymax) in obstacles:
        if (xmin - inflation) <= x <= (xmax + inflation) \
           and (ymin - inflation) <= y <= (ymax + inflation):
            return True
    return False


def _interpolate_strip(x_a: float, x_b: float, y: float, yaw: float,
                       spacing: float) -> List[Waypoint]:
    """Emit waypoints from (x_a, y) to (x_b, y) at approximately ``spacing`` m.

    Always includes both endpoints. Number of intermediate samples is
    chosen so the actual spacing is <= ``spacing``.
    """
    length = abs(x_b - x_a)
    if length < 1e-6:
        return [(x_a, y, yaw)]
    n_intervals = max(1, int(math.ceil(length / spacing)))
    return [
        (x_a + (x_b - x_a) * (i / n_intervals), y, yaw)
        for i in range(n_intervals + 1)
    ]


def compute_path(params: CoverageParams) -> List[Waypoint]:
    """Compute a boustrophedon waypoint sequence.

    Each strip is sampled at ``waypoint_spacing`` (default 0.50 m) so the
    Nav2 RPP controller has a dense carrot trail to follow rather than
    interpolating between two distant endpoints. The sequence begins at
    ``start_corner`` and snakes across the area in alternating +x / -x
    sweeps, advancing along y each strip.
    """
    params.validate()

    margin = params.strip_width / 2.0
    y_first = params.glass_min_y + margin
    y_last = params.glass_max_y - margin

    if y_last < y_first:
        # Area is narrower than one strip: a single centreline pass.
        ys = [(params.glass_min_y + params.glass_max_y) / 2.0]
    else:
        span = y_last - y_first
        n_strips = int(math.ceil(span / params.strip_width)) + 1
        if n_strips == 1:
            ys = [y_first]
        else:
            step = span / (n_strips - 1)
            ys = [y_first + i * step for i in range(n_strips)]

    x_low = params.glass_min_x + margin
    x_high = params.glass_max_x - margin
    if x_high < x_low:
        x_low = x_high = (params.glass_min_x + params.glass_max_x) / 2.0

    starts_north = params.start_corner in ("NW", "NE")
    starts_east = params.start_corner in ("SE", "NE")

    if starts_north:
        ys = list(reversed(ys))

    waypoints: List[Waypoint] = []
    going_east = not starts_east  # first sweep direction
    # Flip once because the loop toggles before use:
    going_east = not going_east

    for y in ys:
        going_east = not going_east
        if going_east:
            x_a, x_b, yaw = x_low, x_high, 0.0
        else:
            x_a, x_b, yaw = x_high, x_low, math.pi
        strip_pts = _interpolate_strip(x_a, x_b, y, yaw, params.waypoint_spacing)
        # Drop waypoints that fall inside any inflated obstacle.
        # The boustrophedon path naturally detours: the Nav2 controller
        # will plan from the last kept waypoint to the next one, routing
        # around the obstacle. Waypoint spacing (0.50 m) plus inflation
        # (0.25 m) means the path leaves a ~0.75 m gap on each side.
        for pt in strip_pts:
            if not _point_in_inflated_obstacle(
                pt[0], pt[1], params.obstacles, params.obstacle_inflation
            ):
                waypoints.append(pt)

    return waypoints


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw / 2.0
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class CoveragePlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("coverage_planner")

        self.declare_parameter("glass_min_x", -2.30)
        self.declare_parameter("glass_max_x", 2.30)
        self.declare_parameter("glass_min_y", -1.30)
        self.declare_parameter("glass_max_y", 1.30)
        self.declare_parameter("strip_width", 0.30)
        self.declare_parameter("start_corner", "SW")
        self.declare_parameter("waypoint_spacing", 0.50)
        # Flat-list encoding of interior obstacle AABBs:
        # [xmin1, ymin1, xmax1, ymax1, xmin2, ...]. Empty by default
        # (matches glass_basic.sdf). The obstacles world overrides via YAML.
        # An empty default needs an explicit DOUBLE_ARRAY descriptor, else
        # rclpy infers BYTE_ARRAY and the YAML override is rejected.
        self.declare_parameter(
            "obstacles_flat",
            Parameter.Type.DOUBLE_ARRAY,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )
        self.declare_parameter("obstacle_inflation", 0.25)
        self.declare_parameter("path_frame", "map")
        self.declare_parameter("latch", True)

        try:
            raw = self.get_parameter("obstacles_flat").value
        except Exception:
            raw = None
        obstacles_flat = list(raw) if raw else []
        if len(obstacles_flat) % 4 != 0:
            raise ValueError(
                "obstacles_flat must have 4 values per obstacle "
                f"(xmin,ymin,xmax,ymax); got length {len(obstacles_flat)}"
            )
        obstacles = tuple(
            tuple(float(v) for v in obstacles_flat[i:i + 4])
            for i in range(0, len(obstacles_flat), 4)
        )

        self._path_frame = self.get_parameter("path_frame").value
        self._params = CoverageParams(
            glass_min_x=float(self.get_parameter("glass_min_x").value),
            glass_max_x=float(self.get_parameter("glass_max_x").value),
            glass_min_y=float(self.get_parameter("glass_min_y").value),
            glass_max_y=float(self.get_parameter("glass_max_y").value),
            strip_width=float(self.get_parameter("strip_width").value),
            start_corner=str(self.get_parameter("start_corner").value),
            waypoint_spacing=float(self.get_parameter("waypoint_spacing").value),
            obstacles=obstacles,
            obstacle_inflation=float(self.get_parameter("obstacle_inflation").value),
        )

        latch = bool(self.get_parameter("latch").value)
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if latch
                else DurabilityPolicy.VOLATILE
            ),
        )
        self._path_pub = self.create_publisher(
            Path, "/planning/coverage_path", path_qos
        )
        # Latched visualization of the glass/window rectangle. Without Nav2
        # there is no /map OccupancyGrid, so this closed-loop Path is what
        # outlines the "window" in Foxglove 3D. Self-contained in the
        # planning layer (no Nav2 / map_server).
        self._boundary_pub = self.create_publisher(
            Path, "/planning/glass_boundary_viz", path_qos
        )

        self.create_subscription(
            PolygonStamped,
            "/perception/glass_boundary",
            self._on_boundary,
            10,
        )

        self._waypoints = compute_path(self._params)
        self._path_msg = self._build_path_msg(self._waypoints)
        self._path_pub.publish(self._path_msg)
        self._boundary_pub.publish(self._build_boundary_msg())
        distinct_ys = len({round(wp[1], 4) for wp in self._waypoints})
        self.get_logger().info(
            f"Published coverage path with {len(self._waypoints)} waypoints "
            f"({distinct_ys} strips, spacing={self._params.waypoint_spacing:.2f}m) "
            f"over x=[{self._params.glass_min_x:.2f},{self._params.glass_max_x:.2f}] "
            f"y=[{self._params.glass_min_y:.2f},{self._params.glass_max_y:.2f}] "
            f"strip_width={self._params.strip_width:.2f} "
            f"start={self._params.start_corner} "
            f"obstacles={len(self._params.obstacles)} "
            f"(inflation={self._params.obstacle_inflation:.2f}m)"
        )

    def _build_boundary_msg(self) -> Path:
        """Closed-loop Path tracing the glass/window rectangle (4 corners
        back to start) in the path frame, for Foxglove visualization."""
        p = self._params
        corners = [
            (p.glass_min_x, p.glass_min_y),
            (p.glass_max_x, p.glass_min_y),
            (p.glass_max_x, p.glass_max_y),
            (p.glass_min_x, p.glass_max_y),
            (p.glass_min_x, p.glass_min_y),  # close the loop
        ]
        return self._build_path_msg([(x, y, 0.0) for (x, y) in corners])

    def _build_path_msg(self, waypoints: List[Waypoint]) -> Path:
        path = Path()
        path.header.frame_id = self._path_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self._path_frame
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)
        return path

    def _on_boundary(self, msg: PolygonStamped) -> None:
        # Sanity check only. frame_detector publishes pixel coordinates,
        # so we cannot meaningfully compare extents — just confirm a
        # boundary message is flowing for the upcoming demo overlay.
        # PolygonStamped nests the vertices under .polygon.points; the
        # path itself is computed once from params, never from this msg.
        n = len(msg.polygon.points)
        if n != 4:
            self.get_logger().debug(
                f"glass_boundary has {n} points (expected 4)"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["CoverageParams", "compute_path", "yaw_to_quaternion", "main"]
