from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from std_msgs.msg import String

from vision_guided_robot.grid_planner import (
    GridBounds,
    RectangleObstacle,
    build_blocked_cells,
    build_point_obstacle_cells,
    path_cells_intersect_blocked,
    parse_obstacle_rectangles,
    plan_grid_path,
    world_to_grid,
)
from vision_guided_robot.persistent_costmap import PersistentCostmap
from vision_guided_robot.waypoint_driver import Pose2D
from vision_guided_robot.waypoint_driver_node import yaw_from_quaternion


class GridPlannerNode(Node):
    def __init__(self):
        super().__init__("grid_planner")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("clear_costmap_topic", "/planner/clear_costmap")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("map_topic", "/planning/occupancy_grid")
        self.declare_parameter("state_topic", "/planner/state")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("start_with_parameter_goal", True)
        self.declare_parameter("goal_x_m", 2.0)
        self.declare_parameter("goal_y_m", 0.8)
        self.declare_parameter("map_origin_x_m", -1.0)
        self.declare_parameter("map_origin_y_m", -2.0)
        self.declare_parameter("map_width_m", 5.0)
        self.declare_parameter("map_height_m", 4.0)
        self.declare_parameter("map_resolution_m", 0.10)
        self.declare_parameter("inflation_radius_m", 0.25)
        self.declare_parameter("obstacle_rectangles_text", "1.2,0.4,0.10,0.80")
        self.declare_parameter("use_scan_obstacles", False)
        self.declare_parameter("require_scan_for_planning", False)
        self.declare_parameter("replan_on_scan_change", False)
        self.declare_parameter("replan_when_path_blocked", True)
        self.declare_parameter("replan_cooldown_s", 2.0)
        self.declare_parameter("keep_last_scan_map", True)
        self.declare_parameter("persistent_scan_map", False)
        self.declare_parameter("scan_memory_time_s", 12.0)
        self.declare_parameter("scan_obstacle_inflation_radius_m", 0.25)
        self.declare_parameter("scan_max_range_m", 4.0)
        self.declare_parameter("scan_min_range_m", 0.12)
        self.declare_parameter("scan_sample_stride", 2)
        self.declare_parameter("lidar_offset_x_m", 0.16)
        self.declare_parameter("lidar_offset_y_m", 0.0)

        self.latest_pose: Pose2D | None = None
        self.latest_scan_points: list[tuple[float, float]] = []
        self.latest_scan_signature: tuple[tuple[int, int], ...] = tuple()
        self.last_valid_scan_points: list[tuple[float, float]] = []
        self.last_valid_scan_signature: tuple[tuple[int, int], ...] = tuple()
        self.persistent_costmap = PersistentCostmap()
        self.last_path_cells: list[tuple[int, int]] = []
        self.goal_xy: tuple[float, float] | None = None
        self.last_plan_key: tuple[object, ...] | None = None
        self.last_path_msg: Path | None = None
        self.last_map_msg: OccupancyGrid | None = None
        self.last_state = ""
        self.last_blocked_replan_time_s: float | None = None

        if bool(self.get_parameter("start_with_parameter_goal").value):
            self.goal_xy = (
                float(self.get_parameter("goal_x_m").value),
                float(self.get_parameter("goal_y_m").value),
            )

        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            10,
        )
        self.state_pub = self.create_publisher(String, self.get_parameter("state_topic").value, 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.on_odom,
            10,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_pose_topic").value,
            self.on_goal_pose,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.clear_costmap_sub = self.create_subscription(
            Empty,
            self.get_parameter("clear_costmap_topic").value,
            self.on_clear_costmap,
            10,
        )

        publish_rate_hz = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.on_timer)

        self.get_logger().info(
            f"Grid planner publishing {self.get_parameter('path_topic').value}"
        )

    def on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.latest_pose = Pose2D(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(
                x=float(orientation.x),
                y=float(orientation.y),
                z=float(orientation.z),
                w=float(orientation.w),
            ),
        )
        self._plan_if_needed()

    def on_scan(self, msg: LaserScan) -> None:
        if not bool(self.get_parameter("use_scan_obstacles").value):
            return
        if self.latest_pose is None:
            return

        self.latest_scan_points = self._scan_points_in_odom(msg)
        bounds = self._bounds_from_parameters()
        self.latest_scan_signature = self._scan_signature(self.latest_scan_points, bounds)
        if bool(self.get_parameter("persistent_scan_map").value):
            self.persistent_costmap.memory_time_s = max(
                0.0,
                float(self.get_parameter("scan_memory_time_s").value),
            )
            self.persistent_costmap.update(
                self._scan_points_to_blocked_cells(self.latest_scan_points, bounds),
                self._now_s(),
            )
        if self.latest_scan_signature or not bool(self.get_parameter("keep_last_scan_map").value):
            self.last_valid_scan_points = self.latest_scan_points
            self.last_valid_scan_signature = self.latest_scan_signature
        self._plan_if_needed()

    def on_goal_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.get_parameter("frame_id").value:
            self.get_logger().warning(
                f"Received goal in frame '{msg.header.frame_id}', treating it as odom."
            )
        self.goal_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.last_plan_key = None
        if bool(self.get_parameter("persistent_scan_map").value):
            self.persistent_costmap.clear()
        self._plan_if_needed()

    def on_clear_costmap(self, _msg: Empty) -> None:
        self.persistent_costmap.clear()
        self.latest_scan_points = []
        self.latest_scan_signature = tuple()
        self.last_valid_scan_points = []
        self.last_valid_scan_signature = tuple()
        self.last_plan_key = None
        self.last_path_msg = None
        self.last_path_cells = []
        self.last_map_msg = None
        self._publish_state("COSTMAP_CLEARED")
        self.get_logger().info("Cleared scan-derived planner costmap")
        self._plan_if_needed()

    def on_timer(self) -> None:
        if self.last_map_msg is not None:
            self.last_map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.last_map_msg)
        if self.last_path_msg is not None:
            self.last_path_msg.header.stamp = self.get_clock().now().to_msg()
            for pose in self.last_path_msg.poses:
                pose.header.stamp = self.last_path_msg.header.stamp
            self.path_pub.publish(self.last_path_msg)
        self._publish_state(self.last_state or self._current_wait_state())

    def _plan_if_needed(self) -> None:
        if self.latest_pose is None or self.goal_xy is None:
            self._publish_state(self._current_wait_state())
            return
        if (
            bool(self.get_parameter("use_scan_obstacles").value)
            and bool(self.get_parameter("require_scan_for_planning").value)
            and not self._has_scan_map()
            and self.last_path_msg is None
        ):
            self._publish_state("WAITING_FOR_SCAN")
            return

        use_scan_obstacles = bool(self.get_parameter("use_scan_obstacles").value)
        bounds = self._bounds_from_parameters()
        obstacles = self._obstacles_from_parameters()
        scan_blocked = self._scan_blocked_cells(bounds) if use_scan_obstacles else set()
        scan_signature = tuple(sorted(scan_blocked))
        blocked_replan_allowed = self._blocked_replan_allowed()
        path_blocked = (
            use_scan_obstacles
            and bool(self.get_parameter("replan_when_path_blocked").value)
            and blocked_replan_allowed
            and self.last_path_msg is not None
            and self._current_path_is_blocked(bounds, scan_blocked)
        )
        plan_key = (
            round(self.goal_xy[0], 2),
            round(self.goal_xy[1], 2),
            str(self.get_parameter("obstacle_rectangles_text").value),
            round(float(self.get_parameter("inflation_radius_m").value), 2),
            use_scan_obstacles,
            round(float(self.get_parameter("scan_obstacle_inflation_radius_m").value), 2),
            (
                scan_signature
                if use_scan_obstacles
                and (
                    bool(self.get_parameter("replan_on_scan_change").value)
                    or self.last_path_msg is None
                )
                else tuple()
            ),
            "path_blocked" if path_blocked else "",
        )
        if plan_key == self.last_plan_key and not path_blocked:
            return

        self.last_map_msg = self._make_map_msg(bounds, obstacles, scan_blocked)

        plan = plan_grid_path(
            start=(self.latest_pose.x, self.latest_pose.y),
            goal=self.goal_xy,
            bounds=bounds,
            obstacles=obstacles,
            inflation_radius_m=float(self.get_parameter("inflation_radius_m").value),
            additional_blocked_cells=scan_blocked,
        )
        self.last_plan_key = plan_key

        if plan is None:
            self.last_path_msg = None
            self._publish_state("NO_PATH")
            self.get_logger().warning(
                f"No path from ({self.latest_pose.x:.2f}, {self.latest_pose.y:.2f}) "
                f"to ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})"
            )
            return

        self.last_path_msg = self._make_path_msg(plan.points)
        self.last_path_cells = [
            cell
            for point in plan.points
            if (cell := world_to_grid(point, bounds)) is not None
        ]
        if path_blocked:
            self.last_blocked_replan_time_s = self._now_s()
        self.path_pub.publish(self.last_path_msg)
        self.map_pub.publish(self.last_map_msg)
        self._publish_state("PLANNED")
        self.get_logger().info(
            f"Planned path with {len(plan.points)} waypoints, "
            f"{len(plan.cells)} grid cells, expanded {plan.expanded_cells} cells"
        )

    def _current_wait_state(self) -> str:
        if self.latest_pose is None:
            return "WAITING_FOR_ODOM"
        if self.goal_xy is None:
            return "WAITING_FOR_GOAL"
        if (
            bool(self.get_parameter("require_scan_for_planning").value)
            and not self._has_scan_map()
            and self.last_path_msg is None
        ):
            return "WAITING_FOR_SCAN"
        if self.last_path_msg is not None:
            return "PLANNED"
        return "WAITING"

    def _bounds_from_parameters(self) -> GridBounds:
        return GridBounds(
            origin_x=float(self.get_parameter("map_origin_x_m").value),
            origin_y=float(self.get_parameter("map_origin_y_m").value),
            width_m=float(self.get_parameter("map_width_m").value),
            height_m=float(self.get_parameter("map_height_m").value),
            resolution_m=float(self.get_parameter("map_resolution_m").value),
        )

    def _obstacles_from_parameters(self) -> list[RectangleObstacle]:
        text = str(self.get_parameter("obstacle_rectangles_text").value)
        try:
            return parse_obstacle_rectangles(text)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return []

    def _make_map_msg(
        self,
        bounds: GridBounds,
        obstacles: list[RectangleObstacle],
        additional_blocked_cells: set[tuple[int, int]] | None = None,
    ) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(bounds.resolution_m)
        msg.info.width = int(bounds.width_cells)
        msg.info.height = int(bounds.height_cells)
        msg.info.origin.position.x = float(bounds.origin_x)
        msg.info.origin.position.y = float(bounds.origin_y)
        msg.info.origin.orientation.w = 1.0

        blocked = build_blocked_cells(
            bounds,
            obstacles,
            float(self.get_parameter("inflation_radius_m").value),
        )
        if additional_blocked_cells:
            blocked = set(blocked) | additional_blocked_cells
        data = []
        for gy in range(bounds.height_cells):
            for gx in range(bounds.width_cells):
                data.append(100 if (gx, gy) in blocked else 0)
        msg.data = data
        return msg

    def _scan_blocked_cells(self, bounds: GridBounds) -> set[tuple[int, int]]:
        if bool(self.get_parameter("persistent_scan_map").value):
            return self.persistent_costmap.active_cells(self._now_s())

        return self._scan_points_to_blocked_cells(self.last_valid_scan_points, bounds)

    def _scan_points_to_blocked_cells(
        self,
        points: list[tuple[float, float]],
        bounds: GridBounds,
    ) -> set[tuple[int, int]]:
        return build_point_obstacle_cells(
            bounds,
            points,
            float(self.get_parameter("scan_obstacle_inflation_radius_m").value),
        )

    def _has_scan_map(self) -> bool:
        if bool(self.get_parameter("persistent_scan_map").value):
            return bool(self.persistent_costmap.signature(self._now_s()))
        return bool(self.last_valid_scan_signature)

    def _current_path_is_blocked(
        self,
        bounds: GridBounds,
        blocked_cells: set[tuple[int, int]],
    ) -> bool:
        if len(self.last_path_cells) < 2 or not blocked_cells:
            return False

        start_cell = (
            world_to_grid((self.latest_pose.x, self.latest_pose.y), bounds)
            if self.latest_pose is not None
            else None
        )
        path_cells = self.last_path_cells
        if start_cell is not None:
            closest_index = min(
                range(len(self.last_path_cells)),
                key=lambda index: (
                    (self.last_path_cells[index][0] - start_cell[0]) ** 2
                    + (self.last_path_cells[index][1] - start_cell[1]) ** 2
                ),
            )
            path_cells = [start_cell] + self.last_path_cells[closest_index + 1 :]

        return path_cells_intersect_blocked(path_cells, blocked_cells)

    def _blocked_replan_allowed(self) -> bool:
        if self.last_blocked_replan_time_s is None:
            return True

        cooldown_s = max(0.0, float(self.get_parameter("replan_cooldown_s").value))
        return self._now_s() - self.last_blocked_replan_time_s >= cooldown_s

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan_points_in_odom(self, msg: LaserScan) -> list[tuple[float, float]]:
        assert self.latest_pose is not None
        max_range = min(
            float(self.get_parameter("scan_max_range_m").value),
            float(msg.range_max),
        )
        min_range = max(
            float(self.get_parameter("scan_min_range_m").value),
            float(msg.range_min),
        )
        stride = max(1, int(self.get_parameter("scan_sample_stride").value))
        lidar_offset_x = float(self.get_parameter("lidar_offset_x_m").value)
        lidar_offset_y = float(self.get_parameter("lidar_offset_y_m").value)
        cos_yaw = math.cos(self.latest_pose.yaw)
        sin_yaw = math.sin(self.latest_pose.yaw)

        points = []
        for index, range_m in enumerate(msg.ranges):
            if index % stride != 0:
                continue
            if not math.isfinite(range_m) or range_m < min_range or range_m > max_range:
                continue

            angle = float(msg.angle_min) + index * float(msg.angle_increment)
            local_x = lidar_offset_x + float(range_m) * math.cos(angle)
            local_y = lidar_offset_y + float(range_m) * math.sin(angle)
            world_x = self.latest_pose.x + local_x * cos_yaw - local_y * sin_yaw
            world_y = self.latest_pose.y + local_x * sin_yaw + local_y * cos_yaw
            points.append((world_x, world_y))

        return points

    def _scan_signature(
        self,
        points: list[tuple[float, float]],
        bounds: GridBounds,
    ) -> tuple[tuple[int, int], ...]:
        cells = []
        for point in points:
            cell = self._point_cell(point, bounds)
            if cell is not None:
                cells.append(cell)
        return tuple(sorted(set(cells)))

    def _point_cell(
        self,
        point: tuple[float, float],
        bounds: GridBounds,
    ) -> tuple[int, int] | None:
        return world_to_grid(point, bounds)

    def _make_path_msg(self, points: list[tuple[float, float]]) -> Path:
        msg = Path()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.header.stamp = self.get_clock().now().to_msg()

        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            yaw = self._path_yaw(points, index)
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            msg.poses.append(pose)

        return msg

    def _path_yaw(self, points: list[tuple[float, float]], index: int) -> float:
        if len(points) < 2:
            return 0.0
        if index + 1 < len(points):
            current = points[index]
            following = points[index + 1]
        else:
            current = points[index - 1]
            following = points[index]
        return math.atan2(following[1] - current[1], following[0] - current[0])

    def _publish_state(self, state: str) -> None:
        self.last_state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
