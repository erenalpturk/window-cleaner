"""Path follower — sends the boustrophedon path to Nav2.

Subscribes to ``/planning/coverage_path`` (transient-local QoS so that a
late start still receives the latched path) and dispatches the full
waypoint sequence as a single ``nav2_msgs/action/NavigateThroughPoses``
goal. Mission progress is exposed on ``/control/mission_state`` so the
cleaning controller (Sub-phase D) can gate its state machine.

Mission state values:
    WAITING   — no path received yet, or action server unavailable.
    RUNNING   — goal accepted by Nav2; robot is following waypoints.
    DONE      — action succeeded; path fully traversed.
    ABORTED   — action failed, was cancelled, or Nav2 reported an error.
"""

from __future__ import annotations

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateThroughPoses
from std_msgs.msg import String


STATE_WAITING = "WAITING"
STATE_RUNNING = "RUNNING"
STATE_DONE = "DONE"
STATE_ABORTED = "ABORTED"


class PathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_follower")

        self.declare_parameter("action_name", "navigate_through_poses")
        self.declare_parameter("server_wait_timeout_s", 30.0)
        self.declare_parameter("stop_cmd_on_done", True)

        self._action_name = str(self.get_parameter("action_name").value)
        self._server_wait_timeout = float(
            self.get_parameter("server_wait_timeout_s").value
        )
        self._stop_cmd_on_done = bool(
            self.get_parameter("stop_cmd_on_done").value
        )

        self._state = STATE_WAITING
        self._goal_handle = None
        self._path_dispatched = False

        # Match the latched publisher in coverage_planner.
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Path, "/planning/coverage_path", self._on_path, path_qos
        )

        self._state_pub = self.create_publisher(
            String, "/control/mission_state", 10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._action_client = ActionClient(
            self, NavigateThroughPoses, self._action_name
        )

        # Re-publish state at 1 Hz so subscribers that join late have it.
        self.create_timer(1.0, self._publish_state)

        self._publish_state()
        self.get_logger().info(
            f"path_follower up — waiting for /planning/coverage_path "
            f"and action server '{self._action_name}'"
        )

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._state
        self._state_pub.publish(msg)

    def _set_state(self, new_state: str) -> None:
        if new_state == self._state:
            return
        self._state = new_state
        self._publish_state()
        self.get_logger().info(f"mission_state -> {new_state}")

    def _on_path(self, msg: Path) -> None:
        if self._path_dispatched:
            self.get_logger().debug("ignoring extra path message; already dispatched")
            return
        if not msg.poses:
            self.get_logger().warn("received empty Path; ignoring")
            return

        self.get_logger().info(
            f"received coverage path with {len(msg.poses)} waypoints; "
            f"waiting for action server '{self._action_name}'..."
        )
        if not self._action_client.wait_for_server(
            timeout_sec=self._server_wait_timeout
        ):
            self.get_logger().error(
                f"action server '{self._action_name}' not available after "
                f"{self._server_wait_timeout}s; aborting"
            )
            self._set_state(STATE_ABORTED)
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = list(msg.poses)
        # Ensure each pose carries the path's frame_id (Nav2 requires it).
        for pose in goal.poses:
            if not pose.header.frame_id:
                pose.header.frame_id = msg.header.frame_id

        self._path_dispatched = True
        self.get_logger().info(
            f"sending NavigateThroughPoses goal with {len(goal.poses)} poses"
        )
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        self._goal_handle = future.result()
        if self._goal_handle is None or not self._goal_handle.accepted:
            self.get_logger().error("goal rejected by Nav2 action server")
            self._set_state(STATE_ABORTED)
            return
        self.get_logger().info("goal accepted by Nav2")
        self._set_state(STATE_RUNNING)
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        # Nav2's NavigateThroughPoses feedback exposes
        # number_of_poses_remaining, distance_remaining, navigation_time, etc.
        fb = feedback_msg.feedback
        remaining = getattr(fb, "number_of_poses_remaining", None)
        distance = getattr(fb, "distance_remaining", None)
        if remaining is not None and distance is not None:
            self.get_logger().debug(
                f"poses_remaining={remaining} distance_remaining={distance:.2f}m"
            )

    def _on_result(self, future) -> None:
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 reported SUCCESS — coverage complete")
            self._set_state(STATE_DONE)
            if self._stop_cmd_on_done:
                self._cmd_vel_pub.publish(Twist())
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Nav2 goal CANCELED")
            self._set_state(STATE_ABORTED)
        else:
            self.get_logger().error(
                f"Nav2 reported failure (status={status})"
            )
            self._set_state(STATE_ABORTED)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["main", "PathFollowerNode"]
