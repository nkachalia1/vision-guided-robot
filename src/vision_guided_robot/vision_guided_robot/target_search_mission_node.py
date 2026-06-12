from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
import tf2_ros

from vision_guided_robot.target_search import (
    RobotPose,
    TargetEstimate,
    compute_approach_goal,
    normalize_angle,
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class TargetSearchMissionNode(Node):
    def __init__(self):
        super().__init__("target_search_mission")

        self.declare_parameter("target_topic", "/ball/relative_position")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("state_topic", "/target_search/state")
        self.declare_parameter("goal_topic", "/target_search/goal")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_raw")
        self.declare_parameter("safety_state_topic", "/safety/state")
        self.declare_parameter("explorer_pause_topic", "/explorer/pause")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("final_approach_target_timeout_s", 2.5)
        self.declare_parameter("control_period_s", 0.1)
        self.declare_parameter("camera_forward_offset_m", 0.19)
        self.declare_parameter("stand_off_distance_m", 0.28)
        self.declare_parameter("stop_distance_m", 0.28)
        self.declare_parameter("mission_success_distance_m", 0.30)
        self.declare_parameter("target_confirmations_required", 2)
        self.declare_parameter("resend_goal_distance_m", 0.35)
        self.declare_parameter("active_search_enabled", True)
        self.declare_parameter("require_search_pose_before_final_approach", True)
        self.declare_parameter(
            "search_waypoints_text",
            "0.85,-0.15",
        )
        self.declare_parameter(
            "search_headings_text",
            "0.0",
        )
        self.declare_parameter("search_loop", False)
        self.declare_parameter("scan_duration_s", 2.25)
        self.declare_parameter("scan_angular_speed_radps", 2.8)
        self.declare_parameter("search_goal_tolerance_m", 0.25)
        self.declare_parameter("search_linear_kp", 5.0)
        self.declare_parameter("search_angular_kp", 5.5)
        self.declare_parameter("search_max_linear_speed_mps", 2.8)
        self.declare_parameter("search_max_angular_speed_radps", 4.5)
        self.declare_parameter("search_heading_slowdown_angle_rad", 1.4)
        self.declare_parameter("search_min_alignment_scale", 0.0)
        self.declare_parameter("approach_linear_kp", 4.0)
        self.declare_parameter("approach_angular_kp", 3.0)
        self.declare_parameter("approach_max_linear_speed_mps", 2.5)
        self.declare_parameter("approach_max_angular_speed_radps", 2.2)
        self.declare_parameter("final_approach_min_linear_speed_mps", 0.90)
        self.declare_parameter("final_approach_reacquire_angular_speed_radps", 0.75)
        self.declare_parameter("approach_heading_slowdown_angle_rad", 1.2)
        self.declare_parameter("approach_min_alignment_scale", 0.65)
        self.declare_parameter("pure_visual_target_approach_enabled", True)
        self.declare_parameter("escape_trigger_s", 0.7)
        self.declare_parameter("escape_backup_time_s", 0.8)
        self.declare_parameter("escape_turn_time_s", 0.8)
        self.declare_parameter("escape_backup_speed_mps", -1.2)
        self.declare_parameter("escape_turn_speed_radps", 3.0)
        self.declare_parameter("skip_blocked_search_pose", True)
        self.declare_parameter("ignore_target_while_relocating_after_blocked_approach", True)
        self.declare_parameter("relocate_on_blocked_target", False)
        self.declare_parameter("target_memory_enabled", True)
        self.declare_parameter("target_memory_use_hint_after_detection", False)
        self.declare_parameter("target_memory_route_before_approach", False)
        self.declare_parameter("target_hint_x_m", 0.0)
        self.declare_parameter("target_hint_y_m", 0.0)
        self.declare_parameter("target_memory_lock_enabled", True)
        self.declare_parameter("target_memory_max_observed_distance_m", 3.0)
        self.declare_parameter("target_memory_side_clearance_m", 0.85)
        self.declare_parameter("target_memory_stand_off_m", 0.85)
        self.declare_parameter("target_memory_min_waypoint_spacing_m", 0.25)
        self.declare_parameter("corridor_follow_enabled", True)
        self.declare_parameter("corridor_scan_timeout_s", 0.8)
        self.declare_parameter("corridor_front_sector_angle_rad", 0.18)
        self.declare_parameter("corridor_side_sector_angle_rad", 0.42)
        self.declare_parameter("corridor_side_clearance_m", 0.22)
        self.declare_parameter("corridor_front_stop_distance_m", 0.22)
        self.declare_parameter("corridor_front_slow_distance_m", 0.40)
        self.declare_parameter("corridor_side_slow_distance_m", 0.24)
        self.declare_parameter("corridor_center_kp", 0.45)
        self.declare_parameter("corridor_wall_avoid_kp", 1.10)
        self.declare_parameter("corridor_front_escape_turn_radps", 1.2)
        self.declare_parameter("corridor_max_linear_speed_mps", 1.40)
        self.declare_parameter("corridor_gap_follow_enabled", False)
        self.declare_parameter("corridor_gap_half_angle_rad", 0.95)
        self.declare_parameter("corridor_gap_heading_kp", 2.2)
        self.declare_parameter("corridor_gap_max_score_range_m", 3.0)
        self.declare_parameter("corridor_gap_target_weight", 0.45)
        self.declare_parameter("corridor_gap_center_weight", 0.10)
        self.declare_parameter("final_approach_wall_avoid_enabled", True)
        self.declare_parameter("simple_corridor_mission_enabled", True)
        self.declare_parameter(
            "simple_corridor_waypoints_text",
            "1.25,-0.15;1.65,0.02;2.05,0.35",
        )
        self.declare_parameter("simple_corridor_goal_tolerance_m", 0.10)
        self.declare_parameter("autonomous_roam_enabled", True)
        self.declare_parameter("roam_forward_speed_mps", 0.85)
        self.declare_parameter("roam_turn_speed_radps", 1.6)
        self.declare_parameter("roam_front_stop_distance_m", 0.50)
        self.declare_parameter("roam_front_slow_distance_m", 1.10)
        self.declare_parameter("roam_side_clearance_m", 0.42)
        self.declare_parameter("roam_side_slow_distance_m", 0.62)
        self.declare_parameter("roam_side_kp", 1.2)
        self.declare_parameter("roam_turn_duration_s", 1.0)
        self.declare_parameter("roam_scan_interval_s", 3.5)

        self.latest_target: TargetEstimate | None = None
        self.latest_target_time_s: float | None = None
        self.latest_scan: LaserScan | None = None
        self.latest_scan_time_s: float | None = None
        self.front_range_m: float | None = None
        self.left_range_m: float | None = None
        self.right_range_m: float | None = None
        self.remembered_target_xy: tuple[float, float] | None = None
        self.remembered_target_time_s: float | None = None
        self.target_memory_locked = False
        self.latest_safety_state = "CLEAR"
        self.safety_state_start_time_s = self._now_s()
        self.confirmed_target_samples = 0
        self.active_goal = False
        self.active_goal_kind: str | None = None
        self.active_goal_token = 0
        self.goal_handle = None
        self.last_goal_xy: tuple[float, float] | None = None
        self.last_state = ""
        self.target_reached = False
        self.search_waypoints = self._parse_search_waypoints(
            str(self.get_parameter("search_waypoints_text").value)
        )
        self.search_headings = self._parse_search_headings(
            str(self.get_parameter("search_headings_text").value)
        )
        self.simple_corridor_waypoints = self._parse_search_waypoints(
            str(self.get_parameter("simple_corridor_waypoints_text").value)
        )
        self.search_waypoint_index = 0
        self.search_heading_index = 0
        self.simple_corridor_index = 0
        self.simple_corridor_active = False
        self.simple_corridor_completed = False
        self.final_approach_active = False
        self.final_approach_last_lateral_m: float | None = None
        self.direct_scan_active = False
        self.direct_scan_start_time_s: float | None = None
        self.escape_phase = "IDLE"
        self.escape_phase_start_time_s: float | None = None
        self.escape_count = 0
        self.relocating_after_blocked_target = False
        self.target_vantage_waypoints: list[tuple[float, float]] = []
        self.target_vantage_completed = False
        self.roam_phase = "FORWARD"
        self.roam_phase_start_time_s = self._now_s()
        self.last_roam_scan_time_s = self._now_s()
        self.roam_turn_direction = 1.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("nav2_action_name").value,
        )

        self.state_pub = self.create_publisher(String, self.get_parameter("state_topic").value, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter("goal_topic").value, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        self.pause_pub = self.create_publisher(
            Bool,
            self.get_parameter("explorer_pause_topic").value,
            10,
        )
        self.safety_sub = self.create_subscription(
            String,
            self.get_parameter("safety_state_topic").value,
            self.on_safety_state,
            10,
        )
        self.target_sub = self.create_subscription(
            PointStamped,
            self.get_parameter("target_topic").value,
            self.on_target,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
            10,
        )

        period_s = max(0.1, float(self.get_parameter("control_period_s").value))
        self.timer = self.create_timer(period_s, self.on_timer)
        self.get_logger().info("Target search mission waiting for visual target")

    def on_target(self, msg: PointStamped) -> None:
        if (
            self.relocating_after_blocked_target
            and bool(
                self.get_parameter(
                    "ignore_target_while_relocating_after_blocked_approach"
                ).value
            )
        ):
            return

        self.latest_target = TargetEstimate(
            lateral_m=float(msg.point.x),
            distance_m=float(msg.point.z),
        )
        self.latest_target_time_s = self._now_s()
        self.confirmed_target_samples += 1
        self.final_approach_last_lateral_m = self.latest_target.lateral_m
        ready_for_final_approach = self.simple_corridor_completed or (
            not bool(self.get_parameter("simple_corridor_mission_enabled").value)
            and self._search_pose_reached_for_final_approach()
        )
        if ready_for_final_approach and not self.target_reached:
            self.final_approach_active = True
            self.direct_scan_active = False
            self.direct_scan_start_time_s = None
            self.confirmed_target_samples = max(
                self.confirmed_target_samples,
                max(1, int(self.get_parameter("target_confirmations_required").value)),
            )
        self._update_remembered_target()

    def on_safety_state(self, msg: String) -> None:
        if msg.data != self.latest_safety_state:
            self.latest_safety_state = msg.data
            self.safety_state_start_time_s = self._now_s()

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time_s = self._now_s()
        front_half_angle = float(
            self.get_parameter("corridor_front_sector_angle_rad").value
        )
        side_half_angle = float(
            self.get_parameter("corridor_side_sector_angle_rad").value
        )
        self.front_range_m = sector_min_range(msg, 0.0, front_half_angle)
        self.left_range_m = sector_min_range(msg, math.pi / 2.0, side_half_angle)
        self.right_range_m = sector_min_range(msg, -math.pi / 2.0, side_half_angle)

    def on_timer(self) -> None:
        if self.target_reached:
            self._pause_explorer(True)
            self._publish_state("MISSION_SUCCEEDED")
            return

        if not self._target_is_fresh():
            if self.final_approach_active:
                self._publish_final_reacquire_scan()
                return
            if self.simple_corridor_active:
                self._drive_simple_corridor_waypoint()
                return
            if self.simple_corridor_completed:
                self._run_post_corridor_scan()
                return
            if self.active_goal and self.active_goal_kind == "target":
                self._pause_explorer(True)
                self._publish_state("APPROACHING_LAST_SEEN_TARGET")
            elif bool(self.get_parameter("active_search_enabled").value):
                self._run_active_search()
            elif not self.active_goal:
                self._pause_explorer(False)
                self.confirmed_target_samples = 0
                self._publish_state("EXPLORING")
            else:
                self._pause_explorer(True)
                self._publish_state("SEARCH_GOAL_ACTIVE")
            return

        assert self.latest_target is not None
        self._pause_explorer(True)
        if self.escape_phase != "IDLE":
            self._continue_escape()
            return
        if self.direct_scan_active:
            self._stop_direct_scan()
        if self.active_goal and self.active_goal_kind != "target":
            self._cancel_active_goal()

        success_distance_m = float(
            self.get_parameter("mission_success_distance_m").value
        )
        if self.latest_target.distance_m <= success_distance_m:
            if self.active_goal and self.goal_handle is not None:
                self._cancel_active_goal()
            self.active_goal = False
            self.target_reached = True
            self._publish_stop()
            self._publish_state("MISSION_SUCCEEDED")
            return

        if (
            not self.final_approach_active
            and bool(
                self.get_parameter(
                    "require_search_pose_before_final_approach"
                ).value
            )
            and not bool(self.get_parameter("simple_corridor_mission_enabled").value)
            and not self._search_pose_reached_for_final_approach()
        ):
            if self.direct_scan_active:
                self._stop_direct_scan()
            self._drive_to_search_waypoint()
            return

        if self.simple_corridor_completed:
            self.final_approach_active = True
            if self.direct_scan_active:
                self._stop_direct_scan()
            if self.active_goal:
                self._cancel_active_goal()
            self.confirmed_target_samples = max(
                self.confirmed_target_samples,
                max(1, int(self.get_parameter("target_confirmations_required").value)),
            )
            self._publish_approach_goal_if_possible()
            self._drive_toward_target()
            return

        required = max(1, int(self.get_parameter("target_confirmations_required").value))
        if self.confirmed_target_samples < required:
            self._publish_state("CONFIRMING_TARGET")
            return

        if not bool(self.get_parameter("simple_corridor_mission_enabled").value):
            self.final_approach_active = True
            if self.active_goal:
                self._cancel_active_goal()
            self.confirmed_target_samples = max(
                self.confirmed_target_samples,
                required,
            )
            self._publish_approach_goal_if_possible()
            self._drive_toward_target()
            return

        if self._should_drive_simple_corridor_before_approach():
            self._drive_simple_corridor_waypoint()
            return

        if self._should_route_to_target_vantage_before_approach():
            self._begin_target_vantage_route()
            return

        if (
            bool(self.get_parameter("relocate_on_blocked_target").value)
            and self._target_approach_blocked_too_long()
        ):
            self._begin_blocked_target_relocation()
            return

        self._publish_approach_goal_if_possible()
        self._drive_toward_target()

    def _search_pose_reached_for_final_approach(self) -> bool:
        waypoints = self._active_search_waypoints()
        return not waypoints or self.search_waypoint_index >= len(waypoints)

    def _run_active_search(self) -> None:
        self._pause_explorer(True)
        self.confirmed_target_samples = 0

        if self.direct_scan_active:
            self._continue_direct_scan()
            return

        if self.escape_phase != "IDLE":
            self._continue_escape()
            return

        if self.active_goal:
            if self.active_goal_kind == "search_move":
                self._publish_state("MOVING_TO_SEARCH_POSE")
            else:
                self._publish_state("SEARCH_GOAL_ACTIVE")
            return

        if self.search_heading_index < len(self.search_headings):
            self._start_direct_scan()
            return

        waypoints = self._active_search_waypoints()

        if not waypoints:
            if bool(self.get_parameter("autonomous_roam_enabled").value):
                self._run_autonomous_roam()
            else:
                self.search_heading_index = 0
                self._publish_state("NO_SEARCH_WAYPOINTS")
            return

        if self.search_waypoint_index >= len(waypoints):
            if self.target_vantage_waypoints:
                self.target_vantage_waypoints = []
                self.target_vantage_completed = True
                self.relocating_after_blocked_target = False
                self.search_waypoint_index = 0
                self.search_heading_index = 0
                self._publish_state("TARGET_VANTAGE_REACHED")
                return
            if bool(self.get_parameter("search_loop").value):
                self.search_waypoint_index = 0
                self.search_heading_index = 0
                self._publish_state("RESTARTING_SEARCH_PATTERN")
            elif (
                bool(self.get_parameter("simple_corridor_mission_enabled").value)
                and not self.simple_corridor_completed
                and self.simple_corridor_waypoints
            ):
                self.simple_corridor_active = True
                self.simple_corridor_index = 0
                self.search_heading_index = 0
                self.get_logger().info(
                    "Search exhausted at corridor entry; driving through corridor "
                    "to get a better view of the target"
                )
                self._drive_simple_corridor_waypoint()
            else:
                self._publish_state("SEARCH_EXHAUSTED")
            return

        if self._safety_blocked_too_long():
            self._start_escape()
            return

        self._drive_to_search_waypoint()

    def _start_direct_scan(self) -> None:
        self.direct_scan_active = True
        self.direct_scan_start_time_s = self._now_s()
        self._publish_state("SCANNING_FOR_TARGET")
        total_scans = max(1, len(self.search_headings))
        scan_number = min(self.search_heading_index + 1, total_scans)
        self.get_logger().info(
            f"Starting camera scan {scan_number}/{total_scans} with direct rotation"
        )
        self._publish_scan_twist()

    def _continue_direct_scan(self) -> None:
        if self.direct_scan_start_time_s is None:
            self._stop_direct_scan()
            return

        elapsed_s = self._now_s() - self.direct_scan_start_time_s
        if elapsed_s < float(self.get_parameter("scan_duration_s").value):
            self._publish_state("SCANNING_FOR_TARGET")
            self._publish_scan_twist()
            return

        self._stop_direct_scan()
        self.search_heading_index += 1
        self._publish_state("SCAN_HEADING_COMPLETE")

    def _publish_scan_twist(self) -> None:
        twist = Twist()
        twist.angular.z = float(self.get_parameter("scan_angular_speed_radps").value)
        self.cmd_vel_pub.publish(twist)

    def _stop_direct_scan(self) -> None:
        self.direct_scan_active = False
        self.direct_scan_start_time_s = None
        self.last_roam_scan_time_s = self._now_s()
        self._publish_stop()

    def _drive_to_search_waypoint(self) -> None:
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self._publish_stop()
            self._publish_state("WAITING_FOR_TF")
            return

        waypoints = self._active_search_waypoints()
        x_m, y_m = waypoints[self.search_waypoint_index]
        dx = x_m - robot_pose.x_m
        dy = y_m - robot_pose.y_m
        distance_m = math.hypot(dx, dy)

        self.goal_pub.publish(self._pose_stamped(x_m, y_m, 0.0))

        if distance_m <= float(self.get_parameter("search_goal_tolerance_m").value):
            self._publish_stop()
            self.relocating_after_blocked_target = False
            self.search_waypoint_index += 1
            self.search_heading_index = 0
            self.latest_target = None
            self.latest_target_time_s = None
            self.confirmed_target_samples = 0
            self._publish_state("SEARCH_POSE_REACHED")
            return

        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - robot_pose.yaw_rad)
        angular_z = clamp(
            float(self.get_parameter("search_angular_kp").value) * heading_error,
            -float(self.get_parameter("search_max_angular_speed_radps").value),
            float(self.get_parameter("search_max_angular_speed_radps").value),
        )

        slowdown_angle = max(
            1e-6,
            float(self.get_parameter("search_heading_slowdown_angle_rad").value),
        )
        alignment_scale = 1.0 - abs(heading_error) / slowdown_angle
        alignment_scale = clamp(
            alignment_scale,
            float(self.get_parameter("search_min_alignment_scale").value),
            1.0,
        )
        linear_x = clamp(
            float(self.get_parameter("search_linear_kp").value) * distance_m,
            0.0,
            float(self.get_parameter("search_max_linear_speed_mps").value),
        )
        linear_x *= alignment_scale
        if abs(heading_error) > 0.45:
            linear_x = 0.0
        linear_x, angular_z = self._apply_simple_corridor_control(linear_x, angular_z)

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self._publish_state("MOVING_TO_SEARCH_POSE")

    def _should_drive_simple_corridor_before_approach(self) -> bool:
        if not bool(self.get_parameter("simple_corridor_mission_enabled").value):
            return False
        if self.simple_corridor_completed:
            return False
        if not self.simple_corridor_waypoints:
            return False
        if not self.simple_corridor_active:
            self.simple_corridor_active = True
            self.simple_corridor_index = 0
            self.get_logger().info(
                "Target found; driving deterministic corridor route before final approach"
            )
        return True

    def _drive_simple_corridor_waypoint(self) -> None:
        self._pause_explorer(True)
        if self.direct_scan_active:
            self._stop_direct_scan()

        if not self.simple_corridor_waypoints:
            self.simple_corridor_active = False
            self.simple_corridor_completed = True
            self._publish_state("CORRIDOR_COMPLETE")
            return

        if self.simple_corridor_index >= len(self.simple_corridor_waypoints):
            self._publish_stop()
            self.simple_corridor_active = False
            self.simple_corridor_completed = True
            self.final_approach_active = False
            self.search_heading_index = 0
            self._publish_state("CORRIDOR_COMPLETE")
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self._publish_stop()
            self._publish_state("WAITING_FOR_TF")
            return

        x_m, y_m = self.simple_corridor_waypoints[self.simple_corridor_index]
        dx = x_m - robot_pose.x_m
        dy = y_m - robot_pose.y_m
        distance_m = math.hypot(dx, dy)
        self.goal_pub.publish(self._pose_stamped(x_m, y_m, 0.0))

        if distance_m <= float(
            self.get_parameter("simple_corridor_goal_tolerance_m").value
        ):
            self._publish_stop()
            self.simple_corridor_index += 1
            if self.simple_corridor_index >= len(self.simple_corridor_waypoints):
                self.simple_corridor_active = False
                self.simple_corridor_completed = True
                self.final_approach_active = False
                self.search_heading_index = 0
                self._publish_state("CORRIDOR_COMPLETE")
            else:
                self._publish_state("CORRIDOR_WAYPOINT_REACHED")
            return

        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - robot_pose.yaw_rad)
        max_angular = float(self.get_parameter("search_max_angular_speed_radps").value)
        angular_z = clamp(
            float(self.get_parameter("search_angular_kp").value) * heading_error,
            -max_angular,
            max_angular,
        )

        linear_x = clamp(
            float(self.get_parameter("search_linear_kp").value) * distance_m,
            0.0,
            min(
                float(self.get_parameter("search_max_linear_speed_mps").value),
                float(self.get_parameter("corridor_max_linear_speed_mps").value),
            ),
        )
        alignment_scale = 1.0 - abs(heading_error) / max(
            1e-6,
            float(self.get_parameter("search_heading_slowdown_angle_rad").value),
        )
        linear_x *= clamp(
            alignment_scale,
            float(self.get_parameter("search_min_alignment_scale").value),
            1.0,
        )
        linear_x, angular_z = self._apply_simple_corridor_control(linear_x, angular_z)

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self._publish_state("MOVING_THROUGH_CORRIDOR")

    def _apply_simple_corridor_control(
        self,
        linear_x: float,
        angular_z: float,
    ) -> tuple[float, float]:
        linear_x = min(
            linear_x,
            float(self.get_parameter("corridor_max_linear_speed_mps").value),
        )
        if not self._scan_is_fresh():
            return linear_x, angular_z

        clearance_m = float(self.get_parameter("corridor_side_clearance_m").value)
        avoid_kp = float(self.get_parameter("corridor_wall_avoid_kp").value)
        left = self.left_range_m
        right = self.right_range_m

        if left is not None and left < clearance_m:
            angular_z -= avoid_kp * (clearance_m - left)
            linear_x *= 0.75
        if right is not None and right < clearance_m:
            angular_z += avoid_kp * (clearance_m - right)
            linear_x *= 0.75

        if self.front_range_m is not None:
            front_slow_m = float(
                self.get_parameter("corridor_front_slow_distance_m").value
            )
            if self.front_range_m < front_slow_m:
                scale = self.front_range_m / max(1e-6, front_slow_m)
                linear_x *= clamp(scale, 0.25, 1.0)

        angular_z = clamp(
            angular_z,
            -float(self.get_parameter("approach_max_angular_speed_radps").value),
            float(self.get_parameter("approach_max_angular_speed_radps").value),
        )
        return linear_x, angular_z

    def _run_post_corridor_scan(self) -> None:
        self._pause_explorer(True)
        self.confirmed_target_samples = 0
        if self.final_approach_active:
            self._publish_final_reacquire_scan()
            return
        if self.direct_scan_active:
            self._continue_direct_scan()
            return
        if self.search_heading_index >= len(self.search_headings):
            self.search_heading_index = 0
        self._start_direct_scan()

    def _publish_final_reacquire_scan(self) -> None:
        if self.direct_scan_active:
            self._stop_direct_scan()
        twist = Twist()
        direction = 1.0
        if self.final_approach_last_lateral_m is not None:
            direction = -1.0 if self.final_approach_last_lateral_m > 0.0 else 1.0
        twist.angular.z = direction * float(
            self.get_parameter("final_approach_reacquire_angular_speed_radps").value
        )
        self.cmd_vel_pub.publish(twist)
        self._publish_state("FINAL_APPROACH_REACQUIRE_TARGET")

    def _run_autonomous_roam(self) -> None:
        self._pause_explorer(True)

        if self.direct_scan_active:
            self._continue_direct_scan()
            return

        if self.escape_phase != "IDLE":
            self._continue_escape()
            return

        now_s = self._now_s()
        scan_interval_s = float(self.get_parameter("roam_scan_interval_s").value)
        if now_s - self.last_roam_scan_time_s >= scan_interval_s:
            self.search_heading_index = 0
            self.last_roam_scan_time_s = now_s
            self._start_direct_scan()
            return

        if not self._scan_is_fresh():
            twist = Twist()
            twist.angular.z = 0.5 * float(
                self.get_parameter("roam_turn_speed_radps").value
            )
            self.cmd_vel_pub.publish(twist)
            self._publish_state("WAITING_FOR_SCAN")
            return

        if self.roam_phase == "TURN":
            elapsed_s = now_s - self.roam_phase_start_time_s
            if elapsed_s < float(self.get_parameter("roam_turn_duration_s").value):
                self._publish_roam_turn()
                return
            self.roam_phase = "FORWARD"
            self.roam_phase_start_time_s = now_s

        front = self.front_range_m
        front_stop_m = float(self.get_parameter("roam_front_stop_distance_m").value)
        if front is not None and front <= front_stop_m:
            self._start_roam_turn()
            self._publish_roam_turn()
            return

        linear_x = float(self.get_parameter("roam_forward_speed_mps").value)
        angular_z = 0.0
        left = self.left_range_m
        right = self.right_range_m

        if left is not None and right is not None:
            angular_z += float(self.get_parameter("roam_side_kp").value) * (left - right)

        side_clearance_m = float(self.get_parameter("roam_side_clearance_m").value)
        side_kp = float(self.get_parameter("roam_side_kp").value)
        if left is not None and left < side_clearance_m:
            angular_z -= side_kp * (side_clearance_m - left)
        if right is not None and right < side_clearance_m:
            angular_z += side_kp * (side_clearance_m - right)

        front_slow_m = float(self.get_parameter("roam_front_slow_distance_m").value)
        if front is not None and front < front_slow_m:
            scale = (front - front_stop_m) / max(1e-6, front_slow_m - front_stop_m)
            linear_x *= clamp(scale, 0.25, 1.0)

        side_slow_m = float(self.get_parameter("roam_side_slow_distance_m").value)
        side_ranges = [value for value in (left, right) if value is not None]
        if side_ranges:
            nearest_side_m = min(side_ranges)
            if nearest_side_m < side_slow_m:
                linear_x *= clamp(nearest_side_m / max(1e-6, side_slow_m), 0.25, 1.0)

        max_angular_z = float(self.get_parameter("roam_turn_speed_radps").value)
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = clamp(angular_z, -max_angular_z, max_angular_z)
        self.cmd_vel_pub.publish(twist)
        self._publish_state("AUTONOMOUS_SEARCH")

    def _start_roam_turn(self) -> None:
        self.roam_phase = "TURN"
        self.roam_phase_start_time_s = self._now_s()
        self.roam_turn_direction = self._open_side_turn_direction()

    def _publish_roam_turn(self) -> None:
        twist = Twist()
        twist.angular.z = (
            self.roam_turn_direction
            * float(self.get_parameter("roam_turn_speed_radps").value)
        )
        self.cmd_vel_pub.publish(twist)
        self._publish_state("ROAM_TURN")

    def _safety_blocked_too_long(self) -> bool:
        if self.latest_safety_state not in {"BLOCKED", "AVOID"}:
            return False
        age_s = self._now_s() - self.safety_state_start_time_s
        return age_s >= float(self.get_parameter("escape_trigger_s").value)

    def _target_approach_blocked_too_long(self) -> bool:
        if self.latest_safety_state not in {"SLOW", "BLOCKED", "AVOID"}:
            return False
        age_s = self._now_s() - self.safety_state_start_time_s
        return age_s >= 0.35

    def _should_route_to_target_vantage_before_approach(self) -> bool:
        if not bool(self.get_parameter("target_memory_route_before_approach").value):
            return False
        if self.target_vantage_completed:
            return False
        if self.target_vantage_waypoints:
            return False
        if self.remembered_target_xy is None:
            return False
        return True

    def _begin_target_vantage_route(self) -> None:
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self._publish_state("WAITING_FOR_TF")
            return

        waypoints = self._target_vantage_route(robot_pose)
        if not waypoints:
            self.target_vantage_completed = True
            return

        self.target_vantage_waypoints = waypoints
        self.search_waypoint_index = 0
        self.search_heading_index = len(self.search_headings)
        self.relocating_after_blocked_target = True
        self.latest_target = None
        self.latest_target_time_s = None
        self.confirmed_target_samples = 0
        self.get_logger().info(
            "Routing to remembered target vantage via "
            + " -> ".join(f"({x:.2f}, {y:.2f})" for x, y in waypoints)
        )
        self._publish_state("ROUTING_TO_TARGET_VANTAGE")

    def _begin_blocked_target_relocation(self) -> None:
        robot_pose = self._lookup_robot_pose()
        if robot_pose is not None:
            waypoints = self._target_vantage_route(robot_pose)
            if waypoints:
                self.target_vantage_waypoints = waypoints
                self.target_vantage_completed = False
                self.search_waypoint_index = 0
                self.search_heading_index = len(self.search_headings)
                self.get_logger().info(
                    "Remembered blocked target; relocating via "
                    + " -> ".join(f"({x:.2f}, {y:.2f})" for x, y in waypoints)
                )

        self.relocating_after_blocked_target = True
        self.latest_target = None
        self.latest_target_time_s = None
        self.confirmed_target_samples = 0
        self._start_escape(skip_current_pose=False)

    def _start_escape(self, *, skip_current_pose: bool | None = None) -> None:
        self.escape_count += 1
        self.escape_phase = "BACKUP"
        self.escape_phase_start_time_s = self._now_s()
        if skip_current_pose is None:
            skip_current_pose = bool(self.get_parameter("skip_blocked_search_pose").value)
        if skip_current_pose:
            self.search_waypoint_index += 1
            self.search_heading_index = 0
        self.safety_state_start_time_s = self._now_s()
        self._publish_state("ESCAPING_BACKUP")
        self._publish_escape_backup()

    def _continue_escape(self) -> None:
        if self.escape_phase_start_time_s is None:
            self._clear_escape()
            return

        elapsed_s = self._now_s() - self.escape_phase_start_time_s
        if self.escape_phase == "BACKUP":
            if elapsed_s < float(self.get_parameter("escape_backup_time_s").value):
                self._publish_state("ESCAPING_BACKUP")
                self._publish_escape_backup()
                return
            self.escape_phase = "TURN"
            self.escape_phase_start_time_s = self._now_s()
            self._publish_state("ESCAPING_TURN")
            self._publish_escape_turn()
            return

        if self.escape_phase == "TURN":
            if elapsed_s < float(self.get_parameter("escape_turn_time_s").value):
                self._publish_state("ESCAPING_TURN")
                self._publish_escape_turn()
                return
            self._clear_escape()
            self._publish_state("ESCAPE_COMPLETE")

    def _publish_escape_backup(self) -> None:
        twist = Twist()
        twist.linear.x = float(self.get_parameter("escape_backup_speed_mps").value)
        twist.angular.z = 0.25 * self._escape_turn_direction()
        self.cmd_vel_pub.publish(twist)

    def _publish_escape_turn(self) -> None:
        twist = Twist()
        twist.angular.z = (
            float(self.get_parameter("escape_turn_speed_radps").value)
            * self._escape_turn_direction()
        )
        self.cmd_vel_pub.publish(twist)

    def _escape_turn_direction(self) -> float:
        return -1.0 if self.escape_count % 2 else 1.0

    def _clear_escape(self) -> None:
        self.escape_phase = "IDLE"
        self.escape_phase_start_time_s = None
        self.safety_state_start_time_s = self._now_s()
        self._publish_stop()

    def _drive_toward_target(self) -> None:
        if self.latest_target is None:
            self._publish_stop()
            return

        distance_m = max(0.0, self.latest_target.distance_m)
        angle_error = math.atan2(self.latest_target.lateral_m, max(distance_m, 1e-6))
        angular_z = clamp(
            -float(self.get_parameter("approach_angular_kp").value) * angle_error,
            -float(self.get_parameter("approach_max_angular_speed_radps").value),
            float(self.get_parameter("approach_max_angular_speed_radps").value),
        )

        distance_error = max(
            0.0,
            distance_m - float(self.get_parameter("stop_distance_m").value),
        )
        linear_x = clamp(
            float(self.get_parameter("approach_linear_kp").value) * distance_error,
            0.0,
            float(self.get_parameter("approach_max_linear_speed_mps").value),
        )
        slowdown_angle = max(
            1e-6,
            float(self.get_parameter("approach_heading_slowdown_angle_rad").value),
        )
        alignment_scale = 1.0 - abs(angle_error) / slowdown_angle
        alignment_scale = clamp(
            alignment_scale,
            float(self.get_parameter("approach_min_alignment_scale").value),
            1.0,
        )
        linear_x *= alignment_scale
        if self.final_approach_active:
            success_distance_m = float(
                self.get_parameter("mission_success_distance_m").value
            )
            if distance_m > success_distance_m and abs(angle_error) < 1.0:
                linear_x = max(
                    linear_x,
                    float(
                        self.get_parameter(
                            "final_approach_min_linear_speed_mps"
                        ).value
                    ),
                )
            elif abs(angle_error) >= 1.0:
                linear_x = 0.0
        corridor_limited = False
        if (
            self.final_approach_active
            and bool(self.get_parameter("final_approach_wall_avoid_enabled").value)
        ):
            linear_x, angular_z = self._apply_simple_corridor_control(
                linear_x,
                angular_z,
            )
        elif not bool(self.get_parameter("pure_visual_target_approach_enabled").value):
            linear_x, angular_z, corridor_limited = self._apply_corridor_control(
                linear_x,
                angular_z,
                angle_error,
            )

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        if corridor_limited:
            self._publish_state("CORRIDOR_APPROACH")
        else:
            self._publish_state("APPROACHING_TARGET")

    def _apply_corridor_control(
        self,
        linear_x: float,
        angular_z: float,
        target_angle_error_rad: float,
    ) -> tuple[float, float, bool]:
        if not bool(self.get_parameter("corridor_follow_enabled").value):
            return linear_x, angular_z, False
        if not self._scan_is_fresh():
            return linear_x, angular_z, False

        corridor_limited = False
        front = self.front_range_m
        left = self.left_range_m
        right = self.right_range_m

        max_linear = float(self.get_parameter("corridor_max_linear_speed_mps").value)
        linear_x = min(linear_x, max_linear)
        front_stop_m = float(self.get_parameter("corridor_front_stop_distance_m").value)
        front_slow_m = float(self.get_parameter("corridor_front_slow_distance_m").value)

        if bool(self.get_parameter("corridor_gap_follow_enabled").value):
            target_scan_heading_rad = -target_angle_error_rad
            gap = self._best_corridor_gap_heading(target_scan_heading_rad)
            if gap is not None:
                gap_heading_rad, gap_range_m = gap
                angular_z = clamp(
                    float(self.get_parameter("corridor_gap_heading_kp").value)
                    * gap_heading_rad,
                    -float(self.get_parameter("approach_max_angular_speed_radps").value),
                    float(self.get_parameter("approach_max_angular_speed_radps").value),
                )
                if gap_range_m <= front_stop_m:
                    linear_x = 0.0
                    angular_z = self._open_side_turn_direction() * float(
                        self.get_parameter("corridor_front_escape_turn_radps").value
                    )
                else:
                    range_scale = (gap_range_m - front_stop_m) / max(
                        1e-6,
                        front_slow_m - front_stop_m,
                    )
                    heading_scale = 1.0 - abs(gap_heading_rad) / max(
                        1e-6,
                        float(self.get_parameter("corridor_gap_half_angle_rad").value),
                    )
                    linear_x *= clamp(range_scale, 0.25, 1.0)
                    linear_x *= clamp(heading_scale, 0.35, 1.0)
                corridor_limited = True

        if left is not None and right is not None:
            center_error_m = left - right
            angular_z += float(self.get_parameter("corridor_center_kp").value) * center_error_m
            corridor_limited = True

        side_clearance_m = float(self.get_parameter("corridor_side_clearance_m").value)
        wall_avoid_kp = float(self.get_parameter("corridor_wall_avoid_kp").value)
        if left is not None and left < side_clearance_m:
            angular_z -= wall_avoid_kp * (side_clearance_m - left)
            corridor_limited = True
        if right is not None and right < side_clearance_m:
            angular_z += wall_avoid_kp * (side_clearance_m - right)
            corridor_limited = True

        if front is not None:
            if front <= front_stop_m:
                linear_x = 0.0
                angular_z += self._open_side_turn_direction() * float(
                    self.get_parameter("corridor_front_escape_turn_radps").value
                )
                corridor_limited = True
            elif front < front_slow_m:
                scale = (front - front_stop_m) / max(1e-6, front_slow_m - front_stop_m)
                linear_x *= clamp(scale, 0.15, 1.0)
                corridor_limited = True

        side_slow_m = float(self.get_parameter("corridor_side_slow_distance_m").value)
        side_ranges = [value for value in (left, right) if value is not None]
        if side_ranges:
            nearest_side_m = min(side_ranges)
            if nearest_side_m < side_slow_m:
                scale = nearest_side_m / max(1e-6, side_slow_m)
                linear_x *= clamp(scale, 0.25, 1.0)
                corridor_limited = True

        angular_z = clamp(
            angular_z,
            -float(self.get_parameter("approach_max_angular_speed_radps").value),
            float(self.get_parameter("approach_max_angular_speed_radps").value),
        )
        return linear_x, angular_z, corridor_limited

    def _best_corridor_gap_heading(
        self,
        target_heading_rad: float,
    ) -> tuple[float, float] | None:
        scan = self.latest_scan
        if scan is None:
            return None

        half_angle_rad = float(self.get_parameter("corridor_gap_half_angle_rad").value)
        max_score_range_m = float(
            self.get_parameter("corridor_gap_max_score_range_m").value
        )
        target_weight = float(self.get_parameter("corridor_gap_target_weight").value)
        center_weight = float(self.get_parameter("corridor_gap_center_weight").value)

        best_score: float | None = None
        best_heading_rad = 0.0
        best_range_m = 0.0
        angle_rad = float(scan.angle_min)
        for range_m in scan.ranges:
            if math.isfinite(range_m) and scan.range_min <= range_m <= scan.range_max:
                heading_rad = normalize_angle(angle_rad)
                if abs(heading_rad) <= half_angle_rad:
                    capped_range_m = min(float(range_m), max_score_range_m)
                    target_error_rad = abs(
                        normalize_angle(heading_rad - target_heading_rad)
                    )
                    score = (
                        capped_range_m
                        - target_weight * target_error_rad
                        - center_weight * abs(heading_rad)
                    )
                    if best_score is None or score > best_score:
                        best_score = score
                        best_heading_rad = heading_rad
                        best_range_m = float(range_m)
            angle_rad += float(scan.angle_increment)

        if best_score is None:
            return None
        return best_heading_rad, best_range_m

    def _open_side_turn_direction(self) -> float:
        left = self.left_range_m
        right = self.right_range_m
        if left is None and right is None:
            return 1.0
        if left is None:
            return -1.0
        if right is None:
            return 1.0
        return 1.0 if left >= right else -1.0

    def _scan_is_fresh(self) -> bool:
        if self.latest_scan_time_s is None:
            return False
        age_s = self._now_s() - self.latest_scan_time_s
        return age_s <= float(self.get_parameter("corridor_scan_timeout_s").value)

    def _publish_approach_goal_if_possible(self) -> None:
        if self.latest_target is None:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        approach_goal = compute_approach_goal(
            robot_pose,
            self.latest_target,
            camera_forward_offset_m=float(
                self.get_parameter("camera_forward_offset_m").value
            ),
            stand_off_distance_m=float(self.get_parameter("stand_off_distance_m").value),
        )
        self.goal_pub.publish(
            self._pose_stamped(
                approach_goal.x_m,
                approach_goal.y_m,
                approach_goal.yaw_rad,
            )
        )

    def _update_remembered_target(self) -> None:
        if not bool(self.get_parameter("target_memory_enabled").value):
            return
        if self.latest_target is None:
            return
        if self.target_memory_locked and bool(
            self.get_parameter("target_memory_lock_enabled").value
        ):
            return

        if bool(self.get_parameter("target_memory_use_hint_after_detection").value):
            target_x_m = float(self.get_parameter("target_hint_x_m").value)
            target_y_m = float(self.get_parameter("target_hint_y_m").value)
            self.remembered_target_xy = (target_x_m, target_y_m)
            self.remembered_target_time_s = self._now_s()
            self.target_memory_locked = bool(
                self.get_parameter("target_memory_lock_enabled").value
            )
            self.get_logger().info(
                f"Remembered target from sim hint at x={target_x_m:.2f}, "
                f"y={target_y_m:.2f}"
            )
            return

        max_distance_m = float(
            self.get_parameter("target_memory_max_observed_distance_m").value
        )
        if self.latest_target.distance_m > max_distance_m:
            self.get_logger().info(
                f"Ignoring distant target memory sample at "
                f"{self.latest_target.distance_m:.2f} m"
            )
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        target_x_m, target_y_m = self._target_map_xy(robot_pose, self.latest_target)
        old_target = self.remembered_target_xy
        self.remembered_target_xy = (target_x_m, target_y_m)
        self.remembered_target_time_s = self._now_s()
        self.target_memory_locked = bool(
            self.get_parameter("target_memory_lock_enabled").value
        )

        if old_target is None or math.hypot(
            target_x_m - old_target[0],
            target_y_m - old_target[1],
        ) >= 0.25:
            self.get_logger().info(
                f"Remembered target at map x={target_x_m:.2f}, y={target_y_m:.2f}"
            )

    def _target_map_xy(
        self,
        robot_pose: RobotPose,
        target: TargetEstimate,
    ) -> tuple[float, float]:
        target_base_x = max(
            0.0,
            float(self.get_parameter("camera_forward_offset_m").value)
            + target.distance_m,
        )
        target_base_y = -target.lateral_m
        cos_yaw = math.cos(robot_pose.yaw_rad)
        sin_yaw = math.sin(robot_pose.yaw_rad)
        return (
            robot_pose.x_m + target_base_x * cos_yaw - target_base_y * sin_yaw,
            robot_pose.y_m + target_base_x * sin_yaw + target_base_y * cos_yaw,
        )

    def _target_vantage_route(self, robot_pose: RobotPose) -> list[tuple[float, float]]:
        if self.remembered_target_xy is None:
            return []

        target_x_m, target_y_m = self.remembered_target_xy
        side_clearance_m = float(
            self.get_parameter("target_memory_side_clearance_m").value
        )
        stand_off_m = float(self.get_parameter("target_memory_stand_off_m").value)
        min_spacing_m = float(
            self.get_parameter("target_memory_min_waypoint_spacing_m").value
        )

        if abs(target_y_m - robot_pose.y_m) > 0.15:
            side_sign = 1.0 if target_y_m > robot_pose.y_m else -1.0
        else:
            side_sign = 1.0 if target_y_m >= 0.0 else -1.0

        side_y_m = target_y_m + side_sign * side_clearance_m
        approach_x_sign = -1.0 if target_x_m >= robot_pose.x_m else 1.0
        approach_x_m = target_x_m + approach_x_sign * stand_off_m
        mid_x_m = 0.5 * (robot_pose.x_m + approach_x_m)

        raw_waypoints = [
            (robot_pose.x_m, side_y_m),
            (mid_x_m, side_y_m),
            (approach_x_m, side_y_m),
        ]

        route: list[tuple[float, float]] = []
        last_x, last_y = robot_pose.x_m, robot_pose.y_m
        for x_m, y_m in raw_waypoints:
            if math.hypot(x_m - last_x, y_m - last_y) < min_spacing_m:
                continue
            route.append((x_m, y_m))
            last_x, last_y = x_m, y_m
        return route

    def _active_search_waypoints(self) -> list[tuple[float, float]]:
        return self.target_vantage_waypoints or self.search_waypoints

    def _publish_stop(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _send_search_move_goal(self) -> None:
        x_m, y_m = self.search_waypoints[self.search_waypoint_index]
        yaw = self.search_headings[0] if self.search_headings else 0.0
        pose = self._pose_stamped(x_m, y_m, yaw)
        self._send_nav2_goal(
            pose,
            goal_kind="search_move",
            state="MOVING_TO_SEARCH_POSE",
            log_message=(
                f"Moving to search pose {self.search_waypoint_index + 1}/"
                f"{len(self.search_waypoints)}: x={x_m:.2f}, y={y_m:.2f}"
            ),
        )

    def _send_goal(self, approach_goal) -> None:
        pose = self._pose_stamped(
            approach_goal.x_m,
            approach_goal.y_m,
            approach_goal.yaw_rad,
        )
        self.last_goal_xy = (approach_goal.x_m, approach_goal.y_m)
        self._send_nav2_goal(
            pose,
            goal_kind="target",
            state="SENDING_TARGET_GOAL",
            log_message=(
                f"Sending target approach goal: x={approach_goal.x_m:.2f}, "
                f"y={approach_goal.y_m:.2f}, yaw={approach_goal.yaw_rad:.2f}, "
                f"target_range={approach_goal.target_range_m:.2f}"
            ),
        )

    def _pose_stamped(self, x_m: float, y_m: float, yaw_rad: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter("map_frame").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x_m)
        pose.pose.position.y = float(y_m)
        pose.pose.orientation.z = math.sin(yaw_rad * 0.5)
        pose.pose.orientation.w = math.cos(yaw_rad * 0.5)
        return pose

    def _send_nav2_goal(
        self,
        pose: PoseStamped,
        *,
        goal_kind: str,
        state: str,
        log_message: str,
    ) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.goal_pub.publish(goal_msg.pose)
        self.active_goal = True
        self.active_goal_kind = goal_kind
        self.active_goal_token += 1
        token = self.active_goal_token
        self._publish_state(state)
        self.get_logger().info(log_message)

        future = self.nav2_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda response_future: self._on_goal_response(
                response_future,
                goal_kind,
                token,
            )
        )

    def _on_goal_response(self, future, goal_kind: str, token: int) -> None:
        if token != self.active_goal_token:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_goal = False
            self.active_goal_kind = None
            self.goal_handle = None
            self._publish_state(f"{goal_kind.upper()}_GOAL_REJECTED")
            self.get_logger().warning(f"{goal_kind} goal was rejected")
            if goal_kind == "scan":
                self.search_heading_index += 1
            elif goal_kind == "search_move":
                self.search_waypoint_index += 1
                self.search_heading_index = 0
            return

        self.goal_handle = goal_handle
        if goal_kind == "target":
            self._publish_state("APPROACHING_TARGET")
        elif goal_kind == "scan":
            self._publish_state("SCANNING_FOR_TARGET")
        elif goal_kind == "search_move":
            self._publish_state("MOVING_TO_SEARCH_POSE")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result_future: self._on_goal_result(result_future, goal_kind, token)
        )

    def _on_goal_result(self, future, goal_kind: str, token: int) -> None:
        if token != self.active_goal_token:
            return
        result = future.result()
        self.active_goal = False
        self.active_goal_kind = None
        self.goal_handle = None

        if goal_kind == "target" and result.status == GoalStatus.STATUS_SUCCEEDED:
            self._stop_direct_scan()
            self.target_reached = True
            self._publish_state("TARGET_REACHED")
            self.get_logger().info("Target approach goal succeeded")
            return

        if goal_kind == "search_move":
            self.search_waypoint_index += 1
            self.search_heading_index = 0
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self._publish_state("SEARCH_POSE_REACHED")
            else:
                self._publish_state("SEARCH_POSE_FAILED")
                self.get_logger().warning(
                    f"Search pose goal finished with status {result.status}"
                )
            return

        self._publish_state("TARGET_GOAL_FAILED")
        self.get_logger().warning(
            f"Target approach goal finished with status {result.status}"
        )

    def _cancel_active_goal(self) -> None:
        self.active_goal_token += 1
        self._stop_direct_scan()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.active_goal = False
        self.active_goal_kind = None
        self.goal_handle = None

    def _should_resend_goal(self, x_m: float, y_m: float) -> bool:
        if self.last_goal_xy is None:
            return True
        distance_m = math.hypot(x_m - self.last_goal_xy[0], y_m - self.last_goal_xy[1])
        return distance_m >= float(self.get_parameter("resend_goal_distance_m").value)

    def _target_is_fresh(self) -> bool:
        if self.latest_target_time_s is None:
            return False
        age_s = self._now_s() - self.latest_target_time_s
        timeout_s = float(self.get_parameter("target_timeout_s").value)
        if self.final_approach_active:
            timeout_s = max(
                timeout_s,
                float(self.get_parameter("final_approach_target_timeout_s").value),
            )
        return age_s <= timeout_s

    def _lookup_robot_pose(self) -> RobotPose | None:
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
        rotation = transform.transform.rotation
        return RobotPose(
            x_m=float(translation.x),
            y_m=float(translation.y),
            yaw_rad=yaw_from_quaternion(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )

    def _parse_search_waypoints(self, text: str) -> list[tuple[float, float]]:
        waypoints: list[tuple[float, float]] = []
        for chunk in text.split(";"):
            stripped = chunk.strip()
            if not stripped:
                continue
            parts = [part.strip() for part in stripped.split(",")]
            if len(parts) != 2:
                self.get_logger().warning(f"Ignoring invalid search waypoint: {chunk}")
                continue
            try:
                waypoints.append((float(parts[0]), float(parts[1])))
            except ValueError:
                self.get_logger().warning(f"Ignoring invalid search waypoint: {chunk}")
        return waypoints

    def _parse_search_headings(self, text: str) -> list[float]:
        headings: list[float] = []
        values = text.replace(";", ",").split(",")
        for value in values:
            stripped = value.strip()
            if not stripped:
                continue
            try:
                headings.append(float(stripped))
            except (TypeError, ValueError):
                self.get_logger().warning(f"Ignoring invalid search heading: {value}")
        return headings

    def _pause_explorer(self, paused: bool) -> None:
        msg = Bool()
        msg.data = bool(paused)
        self.pause_pub.publish(msg)

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        if state != self.last_state:
            self.get_logger().info(f"Target search state: {state}")
            self.last_state = state

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def sector_min_range(
    scan: LaserScan,
    center_angle_rad: float,
    half_width_rad: float,
) -> float | None:
    best_range: float | None = None
    angle = float(scan.angle_min)
    for range_m in scan.ranges:
        if math.isfinite(range_m) and scan.range_min <= range_m <= scan.range_max:
            angle_error = normalize_angle(angle - center_angle_rad)
            if abs(angle_error) <= half_width_rad:
                if best_range is None or range_m < best_range:
                    best_range = float(range_m)
        angle += float(scan.angle_increment)
    return best_range


def main(args=None):
    rclpy.init(args=args)
    node = TargetSearchMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
