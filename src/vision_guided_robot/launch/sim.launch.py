from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")
    world_path = PathJoinSubstitution([package_share, "worlds", "red_ball_world.sdf"])
    bridge_config = PathJoinSubstitution([package_share, "config", "bridge.yaml"])
    detector_config = PathJoinSubstitution([package_share, "config", "detector.yaml"])
    controller_config = PathJoinSubstitution([package_share, "config", "controller.yaml"])
    waypoint_config = PathJoinSubstitution([package_share, "config", "waypoint.yaml"])
    planner_config = PathJoinSubstitution([package_share, "config", "planner.yaml"])
    safety_config = PathJoinSubstitution([package_share, "config", "safety.yaml"])
    visualization_config = PathJoinSubstitution(
        [package_share, "config", "visualization.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "vision_guided_robot.rviz"]
    )
    model_path = PathJoinSubstitution([package_share, "models"])
    target_sdf = PathJoinSubstitution(
        [package_share, "models", LaunchConfiguration("target_model"), "model.sdf"]
    )
    occluder_sdf = PathJoinSubstitution([package_share, "models", "occluder_wall", "model.sdf"])
    second_occluder_sdf = PathJoinSubstitution(
        [package_share, "models", "occluder_wall_blue", "model.sdf"]
    )

    gz_sim_launch = PythonLaunchDescriptionSource(
        [get_package_share_directory("ros_gz_sim"), "/launch/gz_sim.launch.py"]
    )
    vision_mode = IfCondition(
        PythonExpression(["'", LaunchConfiguration("control_mode"), "' == 'vision'"])
    )
    waypoint_mode = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("control_mode"),
                "' == 'waypoint' or '",
                LaunchConfiguration("control_mode"),
                "' == 'planned'",
            ]
        )
    )
    planned_mode = IfCondition(
        PythonExpression(["'", LaunchConfiguration("control_mode"), "' == 'planned'"])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=world_path,
                description="Path to the Gazebo world SDF file.",
            ),
            DeclareLaunchArgument("ball_x", default_value="1.0", description="Red ball x pose."),
            DeclareLaunchArgument("ball_y", default_value="0.0", description="Red ball y pose."),
            DeclareLaunchArgument("ball_z", default_value="0.1", description="Red ball z pose."),
            DeclareLaunchArgument(
                "target_model",
                default_value="red_ball",
                description="Model folder to spawn as the visual target.",
            ),
            DeclareLaunchArgument(
                "spawn_target",
                default_value="true",
                description="Spawn the visual target model.",
            ),
            DeclareLaunchArgument(
                "control_mode",
                default_value="vision",
                description="Controller to run: 'vision', 'waypoint', 'planned', or external.",
            ),
            DeclareLaunchArgument(
                "enable_perception",
                default_value="true",
                description="Start the ball_tracker perception node.",
            ),
            DeclareLaunchArgument(
                "enable_safety_filter",
                default_value="true",
                description="Start the custom safety filter that publishes /cmd_vel.",
            ),
            DeclareLaunchArgument(
                "detector_real_diameter_m",
                default_value="0.20",
                description="Diameter assumed by the distance estimator.",
            ),
            DeclareLaunchArgument(
                "detector_backend",
                default_value="hsv",
                description="Detector backend to use. Current baseline: 'hsv'.",
            ),
            DeclareLaunchArgument(
                "detector_model_path",
                default_value="",
                description="ONNX model path for detector_backend:=onnx/ml/yolo.",
            ),
            DeclareLaunchArgument(
                "detector_class_names_path",
                default_value="",
                description="Optional class-name file for ONNX detector backends.",
            ),
            DeclareLaunchArgument(
                "detector_target_class_names",
                default_value="sports ball",
                description="Comma-separated target classes for ONNX detector backends.",
            ),
            DeclareLaunchArgument(
                "detector_confidence_threshold",
                default_value="0.25",
                description="Minimum confidence for ONNX detector backends.",
            ),
            DeclareLaunchArgument(
                "detector_nms_threshold",
                default_value="0.45",
                description="Non-maximum suppression threshold for ONNX detector backends.",
            ),
            DeclareLaunchArgument(
                "detector_input_size_px",
                default_value="640",
                description="Square ONNX model input size in pixels.",
            ),
            DeclareLaunchArgument(
                "detector_min_area_px",
                default_value="150.0",
                description="Minimum contour area accepted by the detector.",
            ),
            DeclareLaunchArgument(
                "detector_min_circularity",
                default_value="0.55",
                description="Minimum contour circularity accepted by the detector.",
            ),
            DeclareLaunchArgument(
                "spawn_occluder",
                default_value="false",
                description="Spawn a partial occluder between robot and target.",
            ),
            DeclareLaunchArgument("occluder_x", default_value="1.6", description="Occluder x pose."),
            DeclareLaunchArgument("occluder_y", default_value="0.7", description="Occluder y pose."),
            DeclareLaunchArgument("occluder_z", default_value="0.3", description="Occluder z pose."),
            DeclareLaunchArgument(
                "spawn_second_occluder",
                default_value="false",
                description="Spawn a second wall obstacle for harder navigation tests.",
            ),
            DeclareLaunchArgument(
                "second_occluder_x",
                default_value="1.6",
                description="Second occluder x pose.",
            ),
            DeclareLaunchArgument(
                "second_occluder_y",
                default_value="-0.45",
                description="Second occluder y pose.",
            ),
            DeclareLaunchArgument(
                "second_occluder_z",
                default_value="0.3",
                description="Second occluder z pose.",
            ),
            DeclareLaunchArgument(
                "linear_kp",
                default_value="1.0",
                description="Proportional gain for forward approach speed.",
            ),
            DeclareLaunchArgument(
                "angular_kp",
                default_value="2.2",
                description="Proportional gain for yaw correction.",
            ),
            DeclareLaunchArgument(
                "max_linear_speed_mps",
                default_value="1.2",
                description="Maximum forward speed in meters per second.",
            ),
            DeclareLaunchArgument(
                "max_angular_speed_radps",
                default_value="1.8",
                description="Maximum yaw rate in radians per second.",
            ),
            DeclareLaunchArgument(
                "waypoint_linear_kp",
                default_value="1.1",
                description="Waypoint proportional gain for forward speed.",
            ),
            DeclareLaunchArgument(
                "waypoint_angular_kp",
                default_value="2.2",
                description="Waypoint proportional gain for yaw correction.",
            ),
            DeclareLaunchArgument(
                "waypoint_max_linear_speed_mps",
                default_value="0.9",
                description="Waypoint maximum forward speed in meters per second.",
            ),
            DeclareLaunchArgument(
                "waypoint_max_angular_speed_radps",
                default_value="1.8",
                description="Waypoint maximum yaw rate in radians per second.",
            ),
            DeclareLaunchArgument(
                "path_following_mode",
                default_value="waypoint",
                description="Planned-path follower: 'waypoint' or 'pure_pursuit'.",
            ),
            DeclareLaunchArgument(
                "path_lookahead_distance_m",
                default_value="0.45",
                description="Lookahead distance for pure-pursuit path following.",
            ),
            DeclareLaunchArgument(
                "path_heading_slowdown_angle_rad",
                default_value="1.0",
                description="Heading error where pure pursuit begins slowing forward speed.",
            ),
            DeclareLaunchArgument(
                "path_stop_heading_error_rad",
                default_value="1.45",
                description="Heading error where pure pursuit stops forward motion and pivots.",
            ),
            DeclareLaunchArgument(
                "enable_recovery_behavior",
                default_value="true",
                description="Enable backup/rotate/clear-costmap recovery in planned mode.",
            ),
            DeclareLaunchArgument(
                "recovery_max_attempts",
                default_value="2",
                description="Maximum recovery attempts before staying blocked.",
            ),
            DeclareLaunchArgument(
                "recovery_backup_time_s",
                default_value="0.8",
                description="Seconds to back up during planned-navigation recovery.",
            ),
            DeclareLaunchArgument(
                "recovery_backup_speed_mps",
                default_value="0.20",
                description="Reverse speed during planned-navigation recovery.",
            ),
            DeclareLaunchArgument(
                "recovery_rotate_time_s",
                default_value="1.2",
                description="Seconds to rotate during planned-navigation recovery.",
            ),
            DeclareLaunchArgument(
                "recovery_rotate_speed_radps",
                default_value="0.85",
                description="Yaw speed during planned-navigation recovery.",
            ),
            DeclareLaunchArgument(
                "recovery_replan_wait_time_s",
                default_value="1.0",
                description="Seconds to wait after clearing costmap before resuming.",
            ),
            DeclareLaunchArgument(
                "goal_x_m",
                default_value="2.0",
                description="Waypoint goal x position in odom frame.",
            ),
            DeclareLaunchArgument(
                "goal_y_m",
                default_value="0.0",
                description="Waypoint goal y position in odom frame.",
            ),
            DeclareLaunchArgument(
                "goal_yaw_rad",
                default_value="0.0",
                description="Final waypoint yaw in odom frame.",
            ),
            DeclareLaunchArgument(
                "use_final_yaw",
                default_value="false",
                description="Rotate to goal_yaw_rad after reaching the waypoint.",
            ),
            DeclareLaunchArgument(
                "start_with_parameter_goal",
                default_value="true",
                description="Start waypoint mode with the launch-provided goal.",
            ),
            DeclareLaunchArgument(
                "accept_direct_goal_pose",
                default_value="true",
                description="Let waypoint mode consume /goal_pose directly.",
            ),
            DeclareLaunchArgument(
                "planner_start_with_parameter_goal",
                default_value="true",
                description="Start planned mode with the launch-provided goal.",
            ),
            DeclareLaunchArgument(
                "planner_obstacles_text",
                default_value="1.2,0.4,0.10,0.80",
                description="Static rectangular planner obstacles: 'x,y,width,height;...'.",
            ),
            DeclareLaunchArgument(
                "planner_inflation_radius_m",
                default_value="0.25",
                description="Obstacle inflation radius for the grid planner.",
            ),
            DeclareLaunchArgument(
                "planner_use_scan_obstacles",
                default_value="false",
                description="Add live /scan obstacle cells to the planning grid.",
            ),
            DeclareLaunchArgument(
                "planner_require_scan_for_planning",
                default_value="false",
                description="Wait for a lidar scan before publishing a planned path.",
            ),
            DeclareLaunchArgument(
                "planner_replan_on_scan_change",
                default_value="false",
                description="Continuously replan when scan obstacle cells change.",
            ),
            DeclareLaunchArgument(
                "planner_replan_when_path_blocked",
                default_value="true",
                description="Replan only when live scan cells intersect the current path.",
            ),
            DeclareLaunchArgument(
                "planner_replan_cooldown_s",
                default_value="2.0",
                description="Minimum seconds between blocked-path replans.",
            ),
            DeclareLaunchArgument(
                "planner_keep_last_scan_map",
                default_value="true",
                description="Keep the last non-empty live scan map if later scans are empty.",
            ),
            DeclareLaunchArgument(
                "planner_persistent_scan_map",
                default_value="false",
                description="Accumulate scan obstacle cells into a short-lived costmap.",
            ),
            DeclareLaunchArgument(
                "planner_scan_memory_time_s",
                default_value="12.0",
                description="Seconds that scan-observed obstacle cells remain in the costmap.",
            ),
            DeclareLaunchArgument(
                "planner_scan_obstacle_inflation_radius_m",
                default_value="0.25",
                description="Inflation radius for live scan obstacle points.",
            ),
            DeclareLaunchArgument(
                "waypoints_text",
                default_value="",
                description="Semicolon-separated mission waypoints: 'x,y;x,y,yaw'.",
            ),
            DeclareLaunchArgument(
                "loop_waypoints",
                default_value="false",
                description="Repeat the waypoint mission after the final waypoint.",
            ),
            DeclareLaunchArgument(
                "stuck_timeout_s",
                default_value="10.0",
                description="Seconds without waypoint progress before safety-interrupted mission is blocked.",
            ),
            DeclareLaunchArgument(
                "stuck_min_progress_m",
                default_value="0.10",
                description="Minimum distance improvement that resets waypoint stuck detection.",
            ),
            DeclareLaunchArgument(
                "safety_oscillation_max_interruptions",
                default_value="2",
                description="Repeated safety pauses before waypoint mode attempts a detour.",
            ),
            DeclareLaunchArgument(
                "safety_oscillation_window_s",
                default_value="8.0",
                description="Time window for detecting repeated safety pause oscillation.",
            ),
            DeclareLaunchArgument(
                "enable_rerouting",
                default_value="true",
                description="Insert temporary detour waypoints when waypoint progress is blocked.",
            ),
            DeclareLaunchArgument(
                "max_detour_attempts_per_goal",
                default_value="2",
                description="Maximum temporary detours to try before reporting BLOCKED.",
            ),
            DeclareLaunchArgument(
                "detour_forward_offset_m",
                default_value="0.60",
                description="Forward offset used when generating a temporary detour waypoint.",
            ),
            DeclareLaunchArgument(
                "detour_lateral_offset_m",
                default_value="1.00",
                description="Side offset used when generating a temporary detour waypoint.",
            ),
            DeclareLaunchArgument(
                "detour_scan_sector_angle_rad",
                default_value="1.20",
                description="Left/right scan sector used to pick the clearer detour side.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz with the project visualization layout.",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path),
            IncludeLaunchDescription(
                gz_sim_launch,
                launch_arguments={
                    "gz_args": ["-r ", LaunchConfiguration("world")],
                    "on_exit_shutdown": "true",
                }.items(),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-world",
                            "red_ball_world",
                            "-file",
                            target_sdf,
                            "-name",
                            "target_ball",
                            "-x",
                            LaunchConfiguration("ball_x"),
                            "-y",
                            LaunchConfiguration("ball_y"),
                            "-z",
                            LaunchConfiguration("ball_z"),
                        ],
                        condition=IfCondition(LaunchConfiguration("spawn_target")),
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=2.5,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-world",
                            "red_ball_world",
                            "-file",
                            occluder_sdf,
                            "-name",
                            "primary_occluder_wall",
                            "-x",
                            LaunchConfiguration("occluder_x"),
                            "-y",
                            LaunchConfiguration("occluder_y"),
                            "-z",
                            LaunchConfiguration("occluder_z"),
                        ],
                        condition=IfCondition(LaunchConfiguration("spawn_occluder")),
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=2.8,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-world",
                            "red_ball_world",
                            "-file",
                            second_occluder_sdf,
                            "-name",
                            "secondary_occluder_wall",
                            "-x",
                            LaunchConfiguration("second_occluder_x"),
                            "-y",
                            LaunchConfiguration("second_occluder_y"),
                            "-z",
                            LaunchConfiguration("second_occluder_z"),
                        ],
                        condition=IfCondition(LaunchConfiguration("spawn_second_occluder")),
                        output="screen",
                    )
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{"config_file": bridge_config}],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0.16",
                    "--y",
                    "0.0",
                    "--z",
                    "0.13",
                    "--yaw",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "lidar_link",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0.19",
                    "--y",
                    "0.0",
                    "--z",
                    "0.07",
                    "--yaw",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "camera_link",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--yaw",
                    "-1.57079632679",
                    "--pitch",
                    "0.0",
                    "--roll",
                    "-1.57079632679",
                    "--frame-id",
                    "camera_link",
                    "--child-frame-id",
                    "camera_optical_frame",
                ],
                output="screen",
            ),
            Node(
                package="vision_guided_robot",
                executable="ball_tracker",
                condition=IfCondition(LaunchConfiguration("enable_perception")),
                parameters=[
                    detector_config,
                    {
                        "detector_backend": ParameterValue(
                            LaunchConfiguration("detector_backend"),
                            value_type=str,
                        ),
                        "detector_model_path": ParameterValue(
                            LaunchConfiguration("detector_model_path"),
                            value_type=str,
                        ),
                        "detector_class_names_path": ParameterValue(
                            LaunchConfiguration("detector_class_names_path"),
                            value_type=str,
                        ),
                        "detector_target_class_names": ParameterValue(
                            LaunchConfiguration("detector_target_class_names"),
                            value_type=str,
                        ),
                        "detector_confidence_threshold": ParameterValue(
                            LaunchConfiguration("detector_confidence_threshold"),
                            value_type=float,
                        ),
                        "detector_nms_threshold": ParameterValue(
                            LaunchConfiguration("detector_nms_threshold"),
                            value_type=float,
                        ),
                        "detector_input_size_px": ParameterValue(
                            LaunchConfiguration("detector_input_size_px"),
                            value_type=int,
                        ),
                        "real_diameter_m": ParameterValue(
                            LaunchConfiguration("detector_real_diameter_m"),
                            value_type=float,
                        ),
                        "min_area_px": ParameterValue(
                            LaunchConfiguration("detector_min_area_px"),
                            value_type=float,
                        ),
                        "min_circularity": ParameterValue(
                            LaunchConfiguration("detector_min_circularity"),
                            value_type=float,
                        ),
                    },
                ],
                output="screen",
            ),
            Node(
                package="vision_guided_robot",
                executable="visual_servo",
                condition=vision_mode,
                parameters=[
                    controller_config,
                    {
                        "linear_kp": ParameterValue(
                            LaunchConfiguration("linear_kp"),
                            value_type=float,
                        ),
                        "angular_kp": ParameterValue(
                            LaunchConfiguration("angular_kp"),
                            value_type=float,
                        ),
                        "max_linear_speed_mps": ParameterValue(
                            LaunchConfiguration("max_linear_speed_mps"),
                            value_type=float,
                        ),
                        "max_angular_speed_radps": ParameterValue(
                            LaunchConfiguration("max_angular_speed_radps"),
                            value_type=float,
                        ),
                    },
                ],
                output="screen",
            ),
            Node(
                package="vision_guided_robot",
                executable="waypoint_driver",
                condition=waypoint_mode,
                parameters=[
                    waypoint_config,
                    {
                        "goal_x_m": ParameterValue(
                            LaunchConfiguration("goal_x_m"),
                            value_type=float,
                        ),
                        "goal_y_m": ParameterValue(
                            LaunchConfiguration("goal_y_m"),
                            value_type=float,
                        ),
                        "goal_yaw_rad": ParameterValue(
                            LaunchConfiguration("goal_yaw_rad"),
                            value_type=float,
                        ),
                        "use_final_yaw": ParameterValue(
                            LaunchConfiguration("use_final_yaw"),
                            value_type=bool,
                        ),
                        "start_with_parameter_goal": ParameterValue(
                            LaunchConfiguration("start_with_parameter_goal"),
                            value_type=bool,
                        ),
                        "accept_direct_goal_pose": ParameterValue(
                            LaunchConfiguration("accept_direct_goal_pose"),
                            value_type=bool,
                        ),
                        "waypoints_text": ParameterValue(
                            LaunchConfiguration("waypoints_text"),
                            value_type=str,
                        ),
                        "loop_waypoints": ParameterValue(
                            LaunchConfiguration("loop_waypoints"),
                            value_type=bool,
                        ),
                        "stuck_timeout_s": ParameterValue(
                            LaunchConfiguration("stuck_timeout_s"),
                            value_type=float,
                        ),
                        "stuck_min_progress_m": ParameterValue(
                            LaunchConfiguration("stuck_min_progress_m"),
                            value_type=float,
                        ),
                        "safety_oscillation_max_interruptions": ParameterValue(
                            LaunchConfiguration("safety_oscillation_max_interruptions"),
                            value_type=int,
                        ),
                        "safety_oscillation_window_s": ParameterValue(
                            LaunchConfiguration("safety_oscillation_window_s"),
                            value_type=float,
                        ),
                        "enable_rerouting": ParameterValue(
                            LaunchConfiguration("enable_rerouting"),
                            value_type=bool,
                        ),
                        "max_detour_attempts_per_goal": ParameterValue(
                            LaunchConfiguration("max_detour_attempts_per_goal"),
                            value_type=int,
                        ),
                        "detour_forward_offset_m": ParameterValue(
                            LaunchConfiguration("detour_forward_offset_m"),
                            value_type=float,
                        ),
                        "detour_lateral_offset_m": ParameterValue(
                            LaunchConfiguration("detour_lateral_offset_m"),
                            value_type=float,
                        ),
                        "detour_scan_sector_angle_rad": ParameterValue(
                            LaunchConfiguration("detour_scan_sector_angle_rad"),
                            value_type=float,
                        ),
                        "linear_kp": ParameterValue(
                            LaunchConfiguration("waypoint_linear_kp"),
                            value_type=float,
                        ),
                        "angular_kp": ParameterValue(
                            LaunchConfiguration("waypoint_angular_kp"),
                            value_type=float,
                        ),
                        "max_linear_speed_mps": ParameterValue(
                            LaunchConfiguration("waypoint_max_linear_speed_mps"),
                            value_type=float,
                        ),
                        "max_angular_speed_radps": ParameterValue(
                            LaunchConfiguration("waypoint_max_angular_speed_radps"),
                            value_type=float,
                        ),
                        "path_following_mode": ParameterValue(
                            LaunchConfiguration("path_following_mode"),
                            value_type=str,
                        ),
                        "path_lookahead_distance_m": ParameterValue(
                            LaunchConfiguration("path_lookahead_distance_m"),
                            value_type=float,
                        ),
                        "path_heading_slowdown_angle_rad": ParameterValue(
                            LaunchConfiguration("path_heading_slowdown_angle_rad"),
                            value_type=float,
                        ),
                        "path_stop_heading_error_rad": ParameterValue(
                            LaunchConfiguration("path_stop_heading_error_rad"),
                            value_type=float,
                        ),
                        "enable_recovery_behavior": ParameterValue(
                            LaunchConfiguration("enable_recovery_behavior"),
                            value_type=bool,
                        ),
                        "recovery_max_attempts": ParameterValue(
                            LaunchConfiguration("recovery_max_attempts"),
                            value_type=int,
                        ),
                        "recovery_backup_time_s": ParameterValue(
                            LaunchConfiguration("recovery_backup_time_s"),
                            value_type=float,
                        ),
                        "recovery_backup_speed_mps": ParameterValue(
                            LaunchConfiguration("recovery_backup_speed_mps"),
                            value_type=float,
                        ),
                        "recovery_rotate_time_s": ParameterValue(
                            LaunchConfiguration("recovery_rotate_time_s"),
                            value_type=float,
                        ),
                        "recovery_rotate_speed_radps": ParameterValue(
                            LaunchConfiguration("recovery_rotate_speed_radps"),
                            value_type=float,
                        ),
                        "recovery_replan_wait_time_s": ParameterValue(
                            LaunchConfiguration("recovery_replan_wait_time_s"),
                            value_type=float,
                        ),
                    },
                ],
                output="screen",
            ),
            TimerAction(
                period=3.5,
                actions=[
                    Node(
                        package="vision_guided_robot",
                        executable="grid_planner",
                        condition=planned_mode,
                        parameters=[
                            planner_config,
                            {
                                "goal_x_m": ParameterValue(
                                    LaunchConfiguration("goal_x_m"),
                                    value_type=float,
                                ),
                                "goal_y_m": ParameterValue(
                                    LaunchConfiguration("goal_y_m"),
                                    value_type=float,
                                ),
                                "start_with_parameter_goal": ParameterValue(
                                    LaunchConfiguration("planner_start_with_parameter_goal"),
                                    value_type=bool,
                                ),
                                "obstacle_rectangles_text": ParameterValue(
                                    LaunchConfiguration("planner_obstacles_text"),
                                    value_type=str,
                                ),
                                "inflation_radius_m": ParameterValue(
                                    LaunchConfiguration("planner_inflation_radius_m"),
                                    value_type=float,
                                ),
                                "use_scan_obstacles": ParameterValue(
                                    LaunchConfiguration("planner_use_scan_obstacles"),
                                    value_type=bool,
                                ),
                                "require_scan_for_planning": ParameterValue(
                                    LaunchConfiguration("planner_require_scan_for_planning"),
                                    value_type=bool,
                                ),
                                "replan_on_scan_change": ParameterValue(
                                    LaunchConfiguration("planner_replan_on_scan_change"),
                                    value_type=bool,
                                ),
                                "replan_when_path_blocked": ParameterValue(
                                    LaunchConfiguration("planner_replan_when_path_blocked"),
                                    value_type=bool,
                                ),
                                "replan_cooldown_s": ParameterValue(
                                    LaunchConfiguration("planner_replan_cooldown_s"),
                                    value_type=float,
                                ),
                                "keep_last_scan_map": ParameterValue(
                                    LaunchConfiguration("planner_keep_last_scan_map"),
                                    value_type=bool,
                                ),
                                "persistent_scan_map": ParameterValue(
                                    LaunchConfiguration("planner_persistent_scan_map"),
                                    value_type=bool,
                                ),
                                "scan_memory_time_s": ParameterValue(
                                    LaunchConfiguration("planner_scan_memory_time_s"),
                                    value_type=float,
                                ),
                                "scan_obstacle_inflation_radius_m": ParameterValue(
                                    LaunchConfiguration(
                                        "planner_scan_obstacle_inflation_radius_m"
                                    ),
                                    value_type=float,
                                ),
                            },
                        ],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="vision_guided_robot",
                executable="safety_filter",
                condition=IfCondition(LaunchConfiguration("enable_safety_filter")),
                parameters=[safety_config],
                output="screen",
            ),
            Node(
                package="vision_guided_robot",
                executable="robot_visualization",
                parameters=[visualization_config],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
