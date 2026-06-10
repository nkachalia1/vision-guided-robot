from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

from vision_guided_robot.detour_planner import DetourConfig, plan_detour
from vision_guided_robot.mission_state import (
    MissionProgressWatchdog,
    MissionState,
    SAFETY_PAUSE_STATES,
    SafetyOscillationWatchdog,
    choose_mission_state,
)
from vision_guided_robot.waypoint_driver import (
    Pose2D,
    WaypointCommand,
    WaypointConfig,
    WaypointGoal,
    WaypointState,
    compute_waypoint_command,
    parse_waypoints_text,
)
from vision_guided_robot.path_follower import (
    PathFollowerConfig,
    compute_path_following_command,
)
from vision_guided_robot.recovery_behavior import (
    RecoveryBehavior,
    RecoveryConfig,
    RecoveryPhase,
)
from vision_guided_robot.safety_filter import ScanSample, mean_clearance


class WaypointDriverNode(Node):
    def __init__(self):
        super().__init__("waypoint_driver")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("planned_path_topic", "/planned_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_raw")
        self.declare_parameter("state_topic", "/waypoint/state")
        self.declare_parameter("progress_topic", "/waypoint/progress")
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter("safety_state_topic", "/safety/state")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("planner_state_topic", "/planner/state")
        self.declare_parameter("clear_costmap_topic", "/planner/clear_costmap")
        self.declare_parameter("recovery_trigger_topic", "/recovery/trigger")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("accept_direct_goal_pose", True)
        self.declare_parameter("start_with_parameter_goal", True)
        self.declare_parameter("waypoints_text", "")
        self.declare_parameter("loop_waypoints", False)
        self.declare_parameter("waypoint_hold_time_s", 0.5)
        self.declare_parameter("blocked_timeout_s", 8.0)
        self.declare_parameter("stuck_timeout_s", 10.0)
        self.declare_parameter("stuck_min_progress_m", 0.10)
        self.declare_parameter("safety_oscillation_max_interruptions", 2)
        self.declare_parameter("safety_oscillation_window_s", 8.0)
        self.declare_parameter("enable_rerouting", True)
        self.declare_parameter("max_detour_attempts_per_goal", 2)
        self.declare_parameter("detour_forward_offset_m", 0.60)
        self.declare_parameter("detour_lateral_offset_m", 1.00)
        self.declare_parameter("detour_scan_sector_angle_rad", 1.20)
        self.declare_parameter("goal_x_m", 2.0)
        self.declare_parameter("goal_y_m", 0.0)
        self.declare_parameter("goal_yaw_rad", 0.0)
        self.declare_parameter("use_final_yaw", False)
        self.declare_parameter("goal_tolerance_m", 0.10)
        self.declare_parameter("heading_tolerance_rad", 0.12)
        self.declare_parameter("final_yaw_tolerance_rad", 0.12)
        self.declare_parameter("linear_kp", 0.8)
        self.declare_parameter("angular_kp", 1.8)
        self.declare_parameter("max_linear_speed_mps", 0.6)
        self.declare_parameter("max_angular_speed_radps", 1.2)
        self.declare_parameter("path_following_mode", "waypoint")
        self.declare_parameter("path_lookahead_distance_m", 0.45)
        self.declare_parameter("path_heading_slowdown_angle_rad", 1.0)
        self.declare_parameter("path_stop_heading_error_rad", 1.45)
        self.declare_parameter("enable_recovery_behavior", True)
        self.declare_parameter("recovery_max_attempts", 2)
        self.declare_parameter("recovery_backup_time_s", 0.8)
        self.declare_parameter("recovery_backup_speed_mps", 0.20)
        self.declare_parameter("recovery_rotate_time_s", 1.2)
        self.declare_parameter("recovery_rotate_speed_radps", 0.85)
        self.declare_parameter("recovery_replan_wait_time_s", 1.0)

        self.latest_pose: Pose2D | None = None
        self.active_goal: WaypointGoal | None = None
        self.waypoints: list[WaypointGoal] = []
        self.active_path: list[WaypointGoal] = []
        self.current_waypoint_index = 0
        self.done_since_time_s: float | None = None
        self.latest_safety_state = "CLEAR"
        self.latest_planner_state = ""
        self.manual_recovery_requested = False
        self.latest_scan_samples: list[ScanSample] | None = None
        self.safety_pause_started_time_s: float | None = None
        self.progress_watchdog = self._new_progress_watchdog()
        self.safety_oscillation_watchdog = self._new_safety_oscillation_watchdog()
        self.detour_return_goal: WaypointGoal | None = None
        self.detour_attempts_for_goal = 0
        self.last_detour_side: float | None = None
        self.last_state_name: str | None = None
        self.last_mission_state_name: str | None = None
        self.last_progress_text: str | None = None
        self.last_planned_path_signature: tuple[tuple[float, float], ...] | None = None
        self.recovery_behavior = self._new_recovery_behavior()
        self._load_initial_goal()

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.clear_costmap_pub = self.create_publisher(
            Empty,
            self.get_parameter("clear_costmap_topic").value,
            10,
        )
        self.state_pub = self.create_publisher(String, self.get_parameter("state_topic").value, 10)
        self.progress_pub = self.create_publisher(
            String,
            self.get_parameter("progress_topic").value,
            10,
        )
        self.mission_state_pub = self.create_publisher(
            String,
            self.get_parameter("mission_state_topic").value,
            10,
        )
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
        self.planned_path_sub = self.create_subscription(
            Path,
            self.get_parameter("planned_path_topic").value,
            self.on_planned_path,
            10,
        )
        self.safety_state_sub = self.create_subscription(
            String,
            self.get_parameter("safety_state_topic").value,
            self.on_safety_state,
            10,
        )
        self.planner_state_sub = self.create_subscription(
            String,
            self.get_parameter("planner_state_topic").value,
            self.on_planner_state,
            10,
        )
        self.recovery_trigger_sub = self.create_subscription(
            Empty,
            self.get_parameter("recovery_trigger_topic").value,
            self.on_recovery_trigger,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
            qos_profile_sensor_data,
        )

        control_rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / control_rate_hz, self.on_timer)

        self.get_logger().info(
            f"Waypoint driver publishing to {self.get_parameter('cmd_vel_topic').value}"
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

    def on_goal_pose(self, msg: PoseStamped) -> None:
        if not bool(self.get_parameter("accept_direct_goal_pose").value):
            return

        if msg.header.frame_id and msg.header.frame_id != "odom":
            self.get_logger().warning(
                f"Received goal in frame '{msg.header.frame_id}', treating it as odom."
            )

        position = msg.pose.position
        orientation = msg.pose.orientation
        self.active_goal = WaypointGoal(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(
                x=float(orientation.x),
                y=float(orientation.y),
                z=float(orientation.z),
                w=float(orientation.w),
            ),
            use_final_yaw=True,
        )
        self.waypoints = []
        self.active_path = []
        self.current_waypoint_index = 0
        self.done_since_time_s = None
        self.safety_pause_started_time_s = None
        self._reset_detour_state(reset_attempts=True)
        self._reset_progress_watchdog()
        self._reset_safety_oscillation_watchdog()
        self.recovery_behavior.reset()
        self.last_state_name = None
        self.last_mission_state_name = None
        self.last_progress_text = None
        self.get_logger().info(
            f"Received waypoint goal: x={self.active_goal.x:.2f}, "
            f"y={self.active_goal.y:.2f}, yaw={self.active_goal.yaw:.2f}"
        )

    def on_planned_path(self, msg: Path) -> None:
        if not msg.poses:
            return
        if self.recovery_behavior.active and self.recovery_behavior.phase in {
            RecoveryPhase.BACK_UP,
            RecoveryPhase.ROTATE,
            RecoveryPhase.CLEAR_COSTMAP,
        }:
            return

        signature = tuple(
            (
                round(float(pose.pose.position.x), 2),
                round(float(pose.pose.position.y), 2),
            )
            for pose in msg.poses
        )
        if signature == self.last_planned_path_signature:
            return
        self.last_planned_path_signature = signature

        full_path = [
            WaypointGoal(
                x=float(pose.pose.position.x),
                y=float(pose.pose.position.y),
                yaw=yaw_from_quaternion(
                    x=float(pose.pose.orientation.x),
                    y=float(pose.pose.orientation.y),
                    z=float(pose.pose.orientation.z),
                    w=float(pose.pose.orientation.w),
                ),
                use_final_yaw=False,
            )
            for pose in msg.poses
        ]
        if not full_path:
            return

        path_poses = full_path[1:] if len(full_path) > 1 else full_path
        if self._use_path_follower_for_planned_path():
            self.active_path = full_path
            self.waypoints = [full_path[-1]]
        else:
            self.active_path = []
            self.waypoints = path_poses

        if not self.waypoints:
            return

        self.current_waypoint_index = 0
        self.active_goal = self.waypoints[0]
        self.done_since_time_s = None
        self.safety_pause_started_time_s = None
        self._reset_detour_state(reset_attempts=True)
        self._reset_progress_watchdog()
        self._reset_safety_oscillation_watchdog()
        if not self.recovery_behavior.active:
            self.recovery_behavior.reset()
        self.last_state_name = None
        self.last_mission_state_name = None
        self.last_progress_text = None
        if self.active_path:
            self.get_logger().info(
                f"Received planned path with {len(self.active_path)} poses; "
                "using pure-pursuit path following"
            )
        else:
            self.get_logger().info(f"Received planned path with {len(self.waypoints)} waypoints")

    def on_safety_state(self, msg: String) -> None:
        self.latest_safety_state = msg.data

        if msg.data in SAFETY_PAUSE_STATES:
            if self.safety_pause_started_time_s is None:
                self.safety_pause_started_time_s = self._now_s()
        else:
            self.safety_pause_started_time_s = None

    def on_planner_state(self, msg: String) -> None:
        self.latest_planner_state = msg.data

    def on_recovery_trigger(self, _msg: Empty) -> None:
        self.manual_recovery_requested = True
        self.get_logger().warning("Manual recovery trigger received")

    def on_scan(self, msg: LaserScan) -> None:
        samples = []
        angle = float(msg.angle_min)
        for range_m in msg.ranges:
            if math.isfinite(range_m) and msg.range_min <= range_m <= msg.range_max:
                samples.append(ScanSample(angle_rad=angle, range_m=float(range_m)))
            angle += float(msg.angle_increment)

        self.latest_scan_samples = samples

    def on_timer(self) -> None:
        output = self._compute_navigation_command()
        stuck_blocked = self._update_progress_watchdog(
            waypoint_state_name=output.state.value,
            distance_to_goal_m=output.distance_to_goal_m,
        )
        safety_oscillating = self._update_safety_oscillation_watchdog(output.state.value)
        mission_state = self._mission_state(
            output.state.value,
            stuck_blocked,
            safety_oscillating,
        )

        if self.recovery_behavior.active:
            output = self._recovery_command()
            mission_state = MissionState.RECOVERING
        elif self._should_start_recovery(mission_state):
            self._start_recovery()
            output = self._recovery_command()
            mission_state = MissionState.RECOVERING
        elif mission_state == MissionState.BLOCKED and self._start_detour_if_possible():
            output = self._compute_navigation_command()
            mission_state = self._mission_state(
                output.state.value,
                stuck_blocked=False,
                safety_oscillating=False,
            )

        twist = Twist()
        if mission_state == MissionState.BLOCKED:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = output.linear_x
            twist.angular.z = output.angular_z
        self.cmd_pub.publish(twist)

        state_msg = String()
        state_msg.data = output.state.value
        self.state_pub.publish(state_msg)

        progress_msg = String()
        progress_msg.data = self._progress_text(output.state.value)
        self.progress_pub.publish(progress_msg)

        mission_state_msg = String()
        mission_state_msg.data = mission_state.value
        self.mission_state_pub.publish(mission_state_msg)

        if state_msg.data != self.last_state_name:
            self.get_logger().info(
                f"Waypoint state: {state_msg.data}, "
                f"distance: {output.distance_to_goal_m:.2f} m, "
                f"heading error: {output.heading_error_rad:.2f} rad"
            )
            self.last_state_name = state_msg.data

        if progress_msg.data != self.last_progress_text:
            self.get_logger().info(f"Waypoint progress: {progress_msg.data}")
            self.last_progress_text = progress_msg.data

        if mission_state_msg.data != self.last_mission_state_name:
            self.get_logger().info(f"Mission state: {mission_state_msg.data}")
            self.last_mission_state_name = mission_state_msg.data

        if mission_state == MissionState.DONE or (
            self.detour_return_goal is not None and output.state.value == "DONE"
        ):
            if not self.recovery_behavior.active:
                self.recovery_behavior.reset()
            self._advance_waypoint_if_ready(output.state.value)
        else:
            self.done_since_time_s = None

    def _load_initial_goal(self) -> None:
        waypoints_text = str(self.get_parameter("waypoints_text").value)
        try:
            self.waypoints = parse_waypoints_text(
                waypoints_text,
                default_use_final_yaw=bool(self.get_parameter("use_final_yaw").value),
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            self.waypoints = []

        if self.waypoints:
            self.current_waypoint_index = 0
            self.active_goal = self.waypoints[0]
            self.active_path = []
            self.recovery_behavior.reset()
            self._reset_detour_state(reset_attempts=True)
            self._reset_safety_oscillation_watchdog()
            self.get_logger().info(f"Loaded {len(self.waypoints)} mission waypoints")
            return

        if bool(self.get_parameter("start_with_parameter_goal").value):
            self.active_goal = self._goal_from_parameters()
            self.active_path = []
            self.recovery_behavior.reset()

    def _advance_waypoint_if_ready(self, state_name: str) -> None:
        if state_name != "DONE":
            self.done_since_time_s = None
            return

        now_s = self._now_s()
        if self.done_since_time_s is None:
            self.done_since_time_s = now_s
            return

        hold_time_s = max(0.0, float(self.get_parameter("waypoint_hold_time_s").value))
        if now_s - self.done_since_time_s < hold_time_s:
            return

        if self.detour_return_goal is not None:
            self.active_goal = self.detour_return_goal
            self._reset_detour_state(reset_attempts=False)
            self.done_since_time_s = None
            self._reset_progress_watchdog()
            self._reset_safety_oscillation_watchdog()
            self.last_state_name = None
            self.last_mission_state_name = None
            self.last_progress_text = None
            self.get_logger().info("Detour complete; resuming original waypoint")
            return

        if not self.waypoints:
            return

        if self.current_waypoint_index + 1 < len(self.waypoints):
            self.current_waypoint_index += 1
        elif bool(self.get_parameter("loop_waypoints").value):
            self.current_waypoint_index = 0
        else:
            return

        self.active_goal = self.waypoints[self.current_waypoint_index]
        self.done_since_time_s = None
        self._reset_detour_state(reset_attempts=True)
        self._reset_progress_watchdog()
        self._reset_safety_oscillation_watchdog()
        self.last_state_name = None
        self.last_mission_state_name = None
        self.last_progress_text = None
        self.get_logger().info(
            f"Advancing to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}"
        )

    def _mission_state(
        self,
        waypoint_state_name: str,
        stuck_blocked: bool,
        safety_oscillating: bool,
    ) -> MissionState:
        return choose_mission_state(
            has_goal=self.active_goal is not None,
            waypoint_state=waypoint_state_name,
            safety_state=self.latest_safety_state,
            safety_pause_duration_s=self._safety_pause_duration_s(),
            blocked_timeout_s=max(
                0.1,
                float(self.get_parameter("blocked_timeout_s").value),
            ),
            stuck_blocked=stuck_blocked,
            safety_oscillating=safety_oscillating,
            rerouting=self.detour_return_goal is not None,
            recovering=self.recovery_behavior.active,
        )

    def _start_detour_if_possible(self) -> bool:
        if not bool(self.get_parameter("enable_rerouting").value):
            return False

        if self.latest_pose is None or self.active_goal is None:
            return False

        max_attempts = max(0, int(self.get_parameter("max_detour_attempts_per_goal").value))
        if self.detour_attempts_for_goal >= max_attempts:
            return False

        resume_goal = self.detour_return_goal or self.active_goal
        side = self._choose_detour_side()
        if self.detour_attempts_for_goal > 0 and self.last_detour_side is not None:
            side = -self.last_detour_side

        plan = plan_detour(
            pose=self.latest_pose,
            resume_goal=resume_goal,
            side_sign=side,
            config=DetourConfig(
                forward_offset_m=max(
                    0.0,
                    float(self.get_parameter("detour_forward_offset_m").value),
                ),
                lateral_offset_m=max(
                    0.0,
                    float(self.get_parameter("detour_lateral_offset_m").value),
                ),
            ),
        )
        if plan is None:
            return False

        self.active_goal = plan.goal
        self.active_path = []
        self.detour_return_goal = plan.resume_goal
        self.detour_attempts_for_goal += 1
        self.last_detour_side = plan.side_sign
        self.done_since_time_s = None
        self._reset_progress_watchdog()
        self._reset_safety_oscillation_watchdog()
        self.last_state_name = None
        self.last_mission_state_name = None
        self.last_progress_text = None

        side_name = "left" if plan.side_sign > 0.0 else "right"
        self.get_logger().info(
            f"Inserted {side_name} detour {self.detour_attempts_for_goal}/"
            f"{max_attempts}: x={plan.goal.x:.2f}, y={plan.goal.y:.2f}"
        )
        return True

    def _choose_detour_side(self) -> float:
        if not self.latest_scan_samples:
            return -1.0

        sector_angle = max(
            0.1,
            float(self.get_parameter("detour_scan_sector_angle_rad").value),
        )
        left_clearance = mean_clearance(self.latest_scan_samples, lower=0.0, upper=sector_angle)
        right_clearance = mean_clearance(
            self.latest_scan_samples,
            lower=-sector_angle,
            upper=0.0,
        )
        return 1.0 if left_clearance >= right_clearance else -1.0

    def _reset_detour_state(self, reset_attempts: bool) -> None:
        self.detour_return_goal = None
        if reset_attempts:
            self.detour_attempts_for_goal = 0
            self.last_detour_side = None

    def _new_progress_watchdog(self) -> MissionProgressWatchdog:
        return MissionProgressWatchdog(
            min_progress_m=max(
                0.0,
                float(self.get_parameter("stuck_min_progress_m").value),
            ),
            stuck_timeout_s=max(
                0.1,
                float(self.get_parameter("stuck_timeout_s").value),
            ),
        )

    def _reset_progress_watchdog(self) -> None:
        self.progress_watchdog = self._new_progress_watchdog()

    def _new_safety_oscillation_watchdog(self) -> SafetyOscillationWatchdog:
        return SafetyOscillationWatchdog(
            max_interruptions=max(
                1,
                int(self.get_parameter("safety_oscillation_max_interruptions").value),
            ),
            window_s=max(
                0.1,
                float(self.get_parameter("safety_oscillation_window_s").value),
            ),
        )

    def _reset_safety_oscillation_watchdog(self) -> None:
        self.safety_oscillation_watchdog = self._new_safety_oscillation_watchdog()

    def _new_recovery_behavior(self) -> RecoveryBehavior:
        return RecoveryBehavior(config=self._recovery_config_from_parameters())

    def _recovery_config_from_parameters(self) -> RecoveryConfig:
        return RecoveryConfig(
            backup_time_s=max(0.0, float(self.get_parameter("recovery_backup_time_s").value)),
            backup_speed_mps=max(
                0.0,
                float(self.get_parameter("recovery_backup_speed_mps").value),
            ),
            rotate_time_s=max(0.0, float(self.get_parameter("recovery_rotate_time_s").value)),
            rotate_speed_radps=max(
                0.0,
                float(self.get_parameter("recovery_rotate_speed_radps").value),
            ),
            replan_wait_time_s=max(
                0.0,
                float(self.get_parameter("recovery_replan_wait_time_s").value),
            ),
            max_attempts=max(0, int(self.get_parameter("recovery_max_attempts").value)),
        )

    def _update_progress_watchdog(
        self,
        waypoint_state_name: str,
        distance_to_goal_m: float,
    ) -> bool:
        return self.progress_watchdog.update(
            now_s=self._now_s(),
            has_goal=self.active_goal is not None,
            waypoint_state=waypoint_state_name,
            safety_state=self.latest_safety_state,
            distance_to_goal_m=distance_to_goal_m,
        )

    def _update_safety_oscillation_watchdog(self, waypoint_state_name: str) -> bool:
        return self.safety_oscillation_watchdog.update(
            now_s=self._now_s(),
            has_goal=self.active_goal is not None,
            waypoint_state=waypoint_state_name,
            safety_state=self.latest_safety_state,
        )

    def _safety_pause_duration_s(self) -> float:
        if self.safety_pause_started_time_s is None:
            return 0.0

        return max(0.0, self._now_s() - self.safety_pause_started_time_s)

    def _progress_text(self, state_name: str) -> str:
        if state_name == WaypointState.RECOVERING.value:
            return f"{state_name}: {self.recovery_behavior.phase.value}"

        if self.active_goal is None:
            return f"{state_name}: no active goal"

        if self.active_path:
            return (
                f"{state_name}: following path to "
                f"({self.active_goal.x:.2f}, {self.active_goal.y:.2f}) "
                f"with {len(self.active_path)} path poses"
            )

        if self.detour_return_goal is not None:
            return (
                f"{state_name}: detour {self.detour_attempts_for_goal} "
                f"({self.active_goal.x:.2f}, {self.active_goal.y:.2f}) "
                f"then resume ({self.detour_return_goal.x:.2f}, "
                f"{self.detour_return_goal.y:.2f})"
            )

        if self.waypoints:
            return (
                f"{state_name}: waypoint {self.current_waypoint_index + 1}/"
                f"{len(self.waypoints)} "
                f"({self.active_goal.x:.2f}, {self.active_goal.y:.2f}, "
                f"{self.active_goal.yaw:.2f})"
            )

        return (
            f"{state_name}: goal "
            f"({self.active_goal.x:.2f}, {self.active_goal.y:.2f}, "
            f"{self.active_goal.yaw:.2f})"
        )

    def _goal_from_parameters(self) -> WaypointGoal:
        return WaypointGoal(
            x=float(self.get_parameter("goal_x_m").value),
            y=float(self.get_parameter("goal_y_m").value),
            yaw=float(self.get_parameter("goal_yaw_rad").value),
            use_final_yaw=bool(self.get_parameter("use_final_yaw").value),
        )

    def _config_from_parameters(self) -> WaypointConfig:
        return WaypointConfig(
            goal_tolerance_m=float(self.get_parameter("goal_tolerance_m").value),
            heading_tolerance_rad=float(self.get_parameter("heading_tolerance_rad").value),
            final_yaw_tolerance_rad=float(
                self.get_parameter("final_yaw_tolerance_rad").value
            ),
            linear_kp=float(self.get_parameter("linear_kp").value),
            angular_kp=float(self.get_parameter("angular_kp").value),
            max_linear_speed_mps=float(self.get_parameter("max_linear_speed_mps").value),
            max_angular_speed_radps=float(
                self.get_parameter("max_angular_speed_radps").value
            ),
        )

    def _path_config_from_parameters(self) -> PathFollowerConfig:
        return PathFollowerConfig(
            goal_tolerance_m=float(self.get_parameter("goal_tolerance_m").value),
            lookahead_distance_m=float(
                self.get_parameter("path_lookahead_distance_m").value
            ),
            linear_kp=float(self.get_parameter("linear_kp").value),
            angular_kp=float(self.get_parameter("angular_kp").value),
            max_linear_speed_mps=float(self.get_parameter("max_linear_speed_mps").value),
            max_angular_speed_radps=float(
                self.get_parameter("max_angular_speed_radps").value
            ),
            heading_slowdown_angle_rad=float(
                self.get_parameter("path_heading_slowdown_angle_rad").value
            ),
            stop_heading_error_rad=float(
                self.get_parameter("path_stop_heading_error_rad").value
            ),
        )

    def _compute_navigation_command(self):
        if self.active_path and self._use_path_follower_for_planned_path():
            return compute_path_following_command(
                pose=self.latest_pose,
                path=self.active_path,
                config=self._path_config_from_parameters(),
            )

        return compute_waypoint_command(
            pose=self.latest_pose,
            goal=self.active_goal,
            config=self._config_from_parameters(),
        )

    def _use_path_follower_for_planned_path(self) -> bool:
        mode = str(self.get_parameter("path_following_mode").value).strip().lower()
        return mode in {"pure_pursuit", "lookahead", "path"}

    def _should_start_recovery(self, mission_state: MissionState) -> bool:
        if not bool(self.get_parameter("enable_recovery_behavior").value):
            return False
        if not self.recovery_behavior.can_start():
            return False

        if self.manual_recovery_requested:
            return True

        planner_failed = self.latest_planner_state == "NO_PATH"
        planned_navigation_blocked = mission_state == MissionState.BLOCKED and bool(
            self.active_path
        )
        return planner_failed or planned_navigation_blocked

    def _start_recovery(self) -> None:
        self.recovery_behavior.config = self._recovery_config_from_parameters()
        if not self.recovery_behavior.start(self._now_s()):
            return

        self.manual_recovery_requested = False
        self.active_path = []
        self.waypoints = []
        self.active_goal = None
        self.current_waypoint_index = 0
        self.done_since_time_s = None
        self.safety_pause_started_time_s = None
        self.last_planned_path_signature = None
        self._reset_detour_state(reset_attempts=True)
        self._reset_progress_watchdog()
        self._reset_safety_oscillation_watchdog()
        self.last_state_name = None
        self.last_mission_state_name = None
        self.last_progress_text = None
        self.get_logger().warning("Starting planned-navigation recovery behavior")

    def _recovery_command(self) -> WaypointCommand:
        command = self.recovery_behavior.update(self._now_s())
        if command.clear_costmap:
            self.clear_costmap_pub.publish(Empty())
            self.active_path = []
            self.waypoints = []
            self.active_goal = None
            self.last_planned_path_signature = None
            self.get_logger().info("Recovery requested planner costmap clear")

        if command.finished:
            self._reset_progress_watchdog()
            self._reset_safety_oscillation_watchdog()
            self.last_state_name = None
            self.last_mission_state_name = None
            self.last_progress_text = None

        return WaypointCommand(
            linear_x=command.linear_x,
            angular_z=command.angular_z,
            state=WaypointState.RECOVERING,
            distance_to_goal_m=math.inf,
            heading_error_rad=0.0,
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointDriverNode()
    try:
        rclpy.spin(node)
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
