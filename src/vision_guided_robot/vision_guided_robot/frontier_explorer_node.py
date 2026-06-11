from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from vision_guided_robot.frontier_exploration import (
    FrontierCandidate,
    FrontierConfig,
    GridSpec,
    frontier_candidates,
)


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("state_topic", "/explorer/state")
        self.declare_parameter("goal_topic", "/explorer/goal")
        self.declare_parameter("marker_topic", "/explorer/frontiers")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("timer_period_s", 1.0)
        self.declare_parameter("goal_cooldown_s", 3.0)
        self.declare_parameter("max_goals", 4)
        self.declare_parameter("min_cluster_size", 4)
        self.declare_parameter("obstacle_clearance_m", 0.25)
        self.declare_parameter("information_radius_cells", 3)
        self.declare_parameter("min_goal_distance_m", 0.35)
        self.declare_parameter("max_goal_distance_m", 3.5)
        self.declare_parameter("distance_weight", 2.0)
        self.declare_parameter("free_cost_max", 20)
        self.declare_parameter("occupied_cost_min", 65)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("publish_candidate_markers", True)

        self.latest_map: OccupancyGrid | None = None
        self.active_goal = False
        self.sent_goals = 0
        self.last_goal_finished_time_s: float | None = None
        self.last_state = ""

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("nav2_action_name").value,
        )

        self.state_pub = self.create_publisher(String, self.get_parameter("state_topic").value, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter("goal_topic").value, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            10,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.on_map,
            10,
        )

        period_s = max(0.2, float(self.get_parameter("timer_period_s").value))
        self.timer = self.create_timer(period_s, self.on_timer)
        self.get_logger().info(
            f"Frontier explorer waiting for {self.get_parameter('map_topic').value}"
        )

    def on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def on_timer(self) -> None:
        if self.latest_map is None:
            self._publish_state("WAITING_FOR_MAP")
            return

        max_goals = int(self.get_parameter("max_goals").value)
        if max_goals > 0 and self.sent_goals >= max_goals and not self.active_goal:
            self._publish_state("DONE")
            return

        if self.active_goal:
            self._publish_state("NAVIGATING")
            return

        if not self._cooldown_complete():
            self._publish_state("COOLDOWN")
            return

        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            self._publish_state("WAITING_FOR_TF")
            return

        if not self.nav2_client.server_is_ready():
            self._publish_state("WAITING_FOR_NAV2")
            return

        spec = self._grid_spec(self.latest_map)
        data = list(self.latest_map.data)
        config = self._frontier_config()
        candidates = frontier_candidates(data, spec, robot_xy, config)
        if bool(self.get_parameter("publish_candidate_markers").value):
            self._publish_markers(candidates, self.latest_map.header.frame_id or "map")

        candidate = candidates[0] if candidates else None
        if candidate is None:
            self._publish_state("NO_FRONTIER")
            return

        self._send_goal(candidate, robot_xy)

    def _send_goal(
        self,
        candidate: FrontierCandidate,
        robot_xy: tuple[float, float],
    ) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = str(self.get_parameter("map_frame").value)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(candidate.x_m)
        goal_msg.pose.pose.position.y = float(candidate.y_m)
        yaw = math.atan2(candidate.y_m - robot_xy[1], candidate.x_m - robot_xy[0])
        goal_msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal_msg.pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.goal_pub.publish(goal_msg.pose)
        self.active_goal = True
        self.sent_goals += 1
        self._publish_state("SENDING_GOAL")
        self.get_logger().info(
            "Sending frontier goal "
            f"{self.sent_goals}: x={candidate.x_m:.2f}, y={candidate.y_m:.2f}, "
            f"cluster={candidate.cluster_size}, info={candidate.information_gain}, "
            f"distance={candidate.distance_m:.2f}, score={candidate.score:.2f}"
        )

        send_future = self.nav2_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_goal = False
            self.last_goal_finished_time_s = self._now_s()
            self._publish_state("GOAL_REJECTED")
            self.get_logger().warning("Frontier goal was rejected")
            return

        self._publish_state("NAVIGATING")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        result = future.result()
        self.active_goal = False
        self.last_goal_finished_time_s = self._now_s()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_state("GOAL_SUCCEEDED")
            self.get_logger().info("Frontier goal succeeded")
            return

        self._publish_state("GOAL_FAILED")
        self.get_logger().warning(f"Frontier goal finished with status {result.status}")

    def _lookup_robot_xy(self) -> tuple[float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("robot_base_frame").value),
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

        translation = transform.transform.translation
        return (float(translation.x), float(translation.y))

    def _cooldown_complete(self) -> bool:
        if self.last_goal_finished_time_s is None:
            return True
        cooldown_s = max(0.0, float(self.get_parameter("goal_cooldown_s").value))
        return self._now_s() - self.last_goal_finished_time_s >= cooldown_s

    def _grid_spec(self, msg: OccupancyGrid) -> GridSpec:
        return GridSpec(
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution_m=float(msg.info.resolution),
            origin_x_m=float(msg.info.origin.position.x),
            origin_y_m=float(msg.info.origin.position.y),
        )

    def _frontier_config(self) -> FrontierConfig:
        return FrontierConfig(
            free_cost_max=int(self.get_parameter("free_cost_max").value),
            occupied_cost_min=int(self.get_parameter("occupied_cost_min").value),
            unknown_value=int(self.get_parameter("unknown_value").value),
            min_cluster_size=int(self.get_parameter("min_cluster_size").value),
            obstacle_clearance_m=float(self.get_parameter("obstacle_clearance_m").value),
            information_radius_cells=int(
                self.get_parameter("information_radius_cells").value
            ),
            min_goal_distance_m=float(self.get_parameter("min_goal_distance_m").value),
            max_goal_distance_m=float(self.get_parameter("max_goal_distance_m").value),
            distance_weight=float(self.get_parameter("distance_weight").value),
        )

    def _publish_markers(self, candidates: list[FrontierCandidate], frame_id: str) -> None:
        markers = MarkerArray()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        stamp = self.get_clock().now().to_msg()
        for index, candidate in enumerate(candidates[:20]):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = "frontier_candidates"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(candidate.x_m)
            marker.pose.position.y = float(candidate.y_m)
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 0.1
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.75
            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        if state != self.last_state:
            self.get_logger().info(f"Explorer state: {state}")
            self.last_state = state

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
