from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")
    nav2_share = FindPackageShare("nav2_bringup")
    slam_share = FindPackageShare("slam_toolbox")

    sim_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, "launch", "sim.launch.py"])
    )
    slam_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([slam_share, "launch", "online_async_launch.py"])
    )
    navigation_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([nav2_share, "launch", "navigation_launch.py"])
    )

    slam_params = PathJoinSubstitution([package_share, "config", "slam_toolbox.yaml"])
    nav2_params = PathJoinSubstitution([package_share, "config", "nav2_slam_params.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "rviz", "nav2_map.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("ball_x", default_value="2.45"),
            DeclareLaunchArgument("ball_y", default_value="0.85"),
            DeclareLaunchArgument("ball_z", default_value="0.14"),
            DeclareLaunchArgument("occluder_x", default_value="1.15"),
            DeclareLaunchArgument("occluder_y", default_value="0.50"),
            DeclareLaunchArgument("second_occluder_x", default_value="1.65"),
            DeclareLaunchArgument("second_occluder_y", default_value="-0.65"),
            DeclareLaunchArgument("slam_start_delay_s", default_value="1.0"),
            DeclareLaunchArgument("use_nav2", default_value="false"),
            DeclareLaunchArgument("nav2_start_delay_s", default_value="5.0"),
            DeclareLaunchArgument("target_search_start_delay_s", default_value="3.0"),
            DeclareLaunchArgument(
                "target_frame",
                default_value="odom",
                description=(
                    "Frame used by the direct target search controller. odom keeps "
                    "the search points aligned with Gazebo spawn coordinates."
                ),
            ),
            DeclareLaunchArgument("stand_off_distance_m", default_value="0.28"),
            DeclareLaunchArgument("mission_success_distance_m", default_value="0.30"),
            DeclareLaunchArgument("final_approach_target_timeout_s", default_value="2.5"),
            DeclareLaunchArgument("active_search_enabled", default_value="true"),
            DeclareLaunchArgument(
                "require_search_pose_before_final_approach",
                default_value="true",
                description=(
                    "Force the robot to reach the corridor/search pose before "
                    "committing to the final ball approach."
                ),
            ),
            DeclareLaunchArgument(
                "use_safety_filter",
                default_value="false",
                description=(
                    "Use /cmd_vel_raw -> safety_filter -> /cmd_vel. Set false with "
                    "target_cmd_vel_topic:=/cmd_vel only for motion debugging."
                ),
            ),
            DeclareLaunchArgument(
                "target_cmd_vel_topic",
                default_value="/cmd_vel",
                description="Velocity topic used by target_search_mission.",
            ),
            DeclareLaunchArgument(
                "search_waypoints_text",
                default_value="0.85,-0.15",
                description=(
                    "Safe corridor-entry search point before the first wall. The "
                    "robot scans at the start, moves here, then scans again."
                ),
            ),
            DeclareLaunchArgument(
                "search_headings_text",
                default_value="0.0",
                description=(
                    "Number of comma-separated scan passes to perform at each search "
                    "pose. Each pass is a timed direct /cmd_vel rotation."
                ),
            ),
            DeclareLaunchArgument("search_loop", default_value="false"),
            DeclareLaunchArgument("scan_duration_s", default_value="2.25"),
            DeclareLaunchArgument("scan_angular_speed_radps", default_value="2.8"),
            DeclareLaunchArgument("search_max_linear_speed_mps", default_value="2.8"),
            DeclareLaunchArgument("search_max_angular_speed_radps", default_value="4.5"),
            DeclareLaunchArgument("approach_max_linear_speed_mps", default_value="2.5"),
            DeclareLaunchArgument("approach_max_angular_speed_radps", default_value="2.2"),
            DeclareLaunchArgument("final_approach_min_linear_speed_mps", default_value="0.90"),
            DeclareLaunchArgument(
                "final_approach_reacquire_angular_speed_radps",
                default_value="0.75",
            ),
            DeclareLaunchArgument("final_approach_wall_avoid_enabled", default_value="true"),
            DeclareLaunchArgument("pure_visual_target_approach_enabled", default_value="true"),
            DeclareLaunchArgument("escape_trigger_s", default_value="0.7"),
            DeclareLaunchArgument("escape_backup_speed_mps", default_value="-1.2"),
            DeclareLaunchArgument("escape_turn_speed_radps", default_value="3.0"),
            DeclareLaunchArgument("relocate_on_blocked_target", default_value="false"),
            DeclareLaunchArgument("target_memory_enabled", default_value="true"),
            DeclareLaunchArgument("target_memory_use_hint_after_detection", default_value="false"),
            DeclareLaunchArgument("target_memory_route_before_approach", default_value="false"),
            DeclareLaunchArgument("target_memory_side_clearance_m", default_value="0.95"),
            DeclareLaunchArgument("target_memory_stand_off_m", default_value="0.95"),
            DeclareLaunchArgument("corridor_follow_enabled", default_value="true"),
            DeclareLaunchArgument("corridor_front_sector_angle_rad", default_value="0.18"),
            DeclareLaunchArgument("corridor_side_clearance_m", default_value="0.22"),
            DeclareLaunchArgument("corridor_front_stop_distance_m", default_value="0.22"),
            DeclareLaunchArgument("corridor_front_slow_distance_m", default_value="0.40"),
            DeclareLaunchArgument("corridor_side_slow_distance_m", default_value="0.24"),
            DeclareLaunchArgument("corridor_center_kp", default_value="0.45"),
            DeclareLaunchArgument("corridor_wall_avoid_kp", default_value="1.10"),
            DeclareLaunchArgument("corridor_max_linear_speed_mps", default_value="1.40"),
            DeclareLaunchArgument("corridor_gap_follow_enabled", default_value="false"),
            DeclareLaunchArgument("corridor_gap_half_angle_rad", default_value="0.95"),
            DeclareLaunchArgument("corridor_gap_heading_kp", default_value="2.2"),
            DeclareLaunchArgument("corridor_gap_target_weight", default_value="0.45"),
            DeclareLaunchArgument("simple_corridor_mission_enabled", default_value="true"),
            DeclareLaunchArgument(
                "simple_corridor_waypoints_text",
                default_value="1.25,-0.15;1.65,0.02;2.05,0.35",
            ),
            DeclareLaunchArgument("simple_corridor_goal_tolerance_m", default_value="0.10"),
            DeclareLaunchArgument("autonomous_roam_enabled", default_value="false"),
            DeclareLaunchArgument("roam_forward_speed_mps", default_value="0.90"),
            DeclareLaunchArgument("roam_turn_speed_radps", default_value="1.8"),
            DeclareLaunchArgument("roam_front_stop_distance_m", default_value="0.55"),
            DeclareLaunchArgument("roam_front_slow_distance_m", default_value="1.15"),
            DeclareLaunchArgument("roam_side_clearance_m", default_value="0.45"),
            DeclareLaunchArgument("roam_scan_interval_s", default_value="3.5"),
            DeclareLaunchArgument("detector_backend", default_value="hsv"),
            DeclareLaunchArgument("detector_model_path", default_value=""),
            DeclareLaunchArgument("detector_class_names_path", default_value=""),
            DeclareLaunchArgument("detector_target_class_names", default_value="red_ball"),
            DeclareLaunchArgument("detector_confidence_threshold", default_value="0.10"),
            DeclareLaunchArgument("detector_real_diameter_m", default_value="0.20"),
            IncludeLaunchDescription(
                sim_launch,
                launch_arguments={
                    "control_mode": "external",
                    "spawn_target": "true",
                    "ball_x": LaunchConfiguration("ball_x"),
                    "ball_y": LaunchConfiguration("ball_y"),
                    "ball_z": LaunchConfiguration("ball_z"),
                    "enable_perception": "true",
                    "enable_safety_filter": LaunchConfiguration("use_safety_filter"),
                    "spawn_occluder": "true",
                    "occluder_x": LaunchConfiguration("occluder_x"),
                    "occluder_y": LaunchConfiguration("occluder_y"),
                    "spawn_second_occluder": "true",
                    "second_occluder_x": LaunchConfiguration("second_occluder_x"),
                    "second_occluder_y": LaunchConfiguration("second_occluder_y"),
                    "rviz": "false",
                    "detector_backend": LaunchConfiguration("detector_backend"),
                    "detector_model_path": LaunchConfiguration("detector_model_path"),
                    "detector_class_names_path": LaunchConfiguration(
                        "detector_class_names_path"
                    ),
                    "detector_target_class_names": LaunchConfiguration(
                        "detector_target_class_names"
                    ),
                    "detector_confidence_threshold": LaunchConfiguration(
                        "detector_confidence_threshold"
                    ),
                    "detector_real_diameter_m": LaunchConfiguration(
                        "detector_real_diameter_m"
                    ),
                    "detector_min_area_px": "50.0",
                    "detector_min_circularity": "0.30",
                }.items(),
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
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "lidar_link",
                    "--child-frame-id",
                    "vision_bot/lidar_link/front_lidar",
                ],
                output="screen",
            ),
            TimerAction(
                period=LaunchConfiguration("slam_start_delay_s"),
                actions=[
                    IncludeLaunchDescription(
                        slam_launch,
                        launch_arguments={
                            "use_sim_time": "True",
                            "slam_params_file": slam_params,
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("nav2_start_delay_s"),
                condition=IfCondition(LaunchConfiguration("use_nav2")),
                actions=[
                    IncludeLaunchDescription(
                        navigation_launch,
                        launch_arguments={
                            "use_sim_time": "True",
                            "params_file": nav2_params,
                            "autostart": "True",
                            "use_composition": "False",
                            "use_respawn": "False",
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("target_search_start_delay_s"),
                actions=[
                    Node(
                        package="vision_guided_robot",
                        executable="target_search_mission",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "stand_off_distance_m": ParameterValue(
                                    LaunchConfiguration("stand_off_distance_m"),
                                    value_type=float,
                                ),
                                "stop_distance_m": ParameterValue(
                                    LaunchConfiguration("stand_off_distance_m"),
                                    value_type=float,
                                ),
                                "mission_success_distance_m": ParameterValue(
                                    LaunchConfiguration("mission_success_distance_m"),
                                    value_type=float,
                                ),
                                "final_approach_target_timeout_s": ParameterValue(
                                    LaunchConfiguration(
                                        "final_approach_target_timeout_s"
                                    ),
                                    value_type=float,
                                ),
                                "cmd_vel_topic": ParameterValue(
                                    LaunchConfiguration("target_cmd_vel_topic"),
                                    value_type=str,
                                ),
                                "map_frame": ParameterValue(
                                    LaunchConfiguration("target_frame"),
                                    value_type=str,
                                ),
                                "active_search_enabled": ParameterValue(
                                    LaunchConfiguration("active_search_enabled"),
                                    value_type=bool,
                                ),
                                "require_search_pose_before_final_approach": ParameterValue(
                                    LaunchConfiguration(
                                        "require_search_pose_before_final_approach"
                                    ),
                                    value_type=bool,
                                ),
                                "search_waypoints_text": ParameterValue(
                                    LaunchConfiguration("search_waypoints_text"),
                                    value_type=str,
                                ),
                                "search_headings_text": ParameterValue(
                                    LaunchConfiguration("search_headings_text"),
                                    value_type=str,
                                ),
                                "scan_duration_s": ParameterValue(
                                    LaunchConfiguration("scan_duration_s"),
                                    value_type=float,
                                ),
                                "search_loop": ParameterValue(
                                    LaunchConfiguration("search_loop"),
                                    value_type=bool,
                                ),
                                "scan_angular_speed_radps": ParameterValue(
                                    LaunchConfiguration("scan_angular_speed_radps"),
                                    value_type=float,
                                ),
                                "search_max_linear_speed_mps": ParameterValue(
                                    LaunchConfiguration("search_max_linear_speed_mps"),
                                    value_type=float,
                                ),
                                "search_max_angular_speed_radps": ParameterValue(
                                    LaunchConfiguration("search_max_angular_speed_radps"),
                                    value_type=float,
                                ),
                                "approach_max_linear_speed_mps": ParameterValue(
                                    LaunchConfiguration("approach_max_linear_speed_mps"),
                                    value_type=float,
                                ),
                                "approach_max_angular_speed_radps": ParameterValue(
                                    LaunchConfiguration("approach_max_angular_speed_radps"),
                                    value_type=float,
                                ),
                                "final_approach_min_linear_speed_mps": ParameterValue(
                                    LaunchConfiguration(
                                        "final_approach_min_linear_speed_mps"
                                    ),
                                    value_type=float,
                                ),
                                "final_approach_reacquire_angular_speed_radps": ParameterValue(
                                    LaunchConfiguration(
                                        "final_approach_reacquire_angular_speed_radps"
                                    ),
                                    value_type=float,
                                ),
                                "final_approach_wall_avoid_enabled": ParameterValue(
                                    LaunchConfiguration(
                                        "final_approach_wall_avoid_enabled"
                                    ),
                                    value_type=bool,
                                ),
                                "pure_visual_target_approach_enabled": ParameterValue(
                                    LaunchConfiguration(
                                        "pure_visual_target_approach_enabled"
                                    ),
                                    value_type=bool,
                                ),
                                "escape_trigger_s": ParameterValue(
                                    LaunchConfiguration("escape_trigger_s"),
                                    value_type=float,
                                ),
                                "escape_backup_speed_mps": ParameterValue(
                                    LaunchConfiguration("escape_backup_speed_mps"),
                                    value_type=float,
                                ),
                                "escape_turn_speed_radps": ParameterValue(
                                    LaunchConfiguration("escape_turn_speed_radps"),
                                    value_type=float,
                                ),
                                "relocate_on_blocked_target": ParameterValue(
                                    LaunchConfiguration("relocate_on_blocked_target"),
                                    value_type=bool,
                                ),
                                "target_memory_enabled": ParameterValue(
                                    LaunchConfiguration("target_memory_enabled"),
                                    value_type=bool,
                                ),
                                "target_memory_use_hint_after_detection": ParameterValue(
                                    LaunchConfiguration(
                                        "target_memory_use_hint_after_detection"
                                    ),
                                    value_type=bool,
                                ),
                                "target_memory_route_before_approach": ParameterValue(
                                    LaunchConfiguration(
                                        "target_memory_route_before_approach"
                                    ),
                                    value_type=bool,
                                ),
                                "target_memory_side_clearance_m": ParameterValue(
                                    LaunchConfiguration("target_memory_side_clearance_m"),
                                    value_type=float,
                                ),
                                "target_memory_stand_off_m": ParameterValue(
                                    LaunchConfiguration("target_memory_stand_off_m"),
                                    value_type=float,
                                ),
                                "corridor_follow_enabled": ParameterValue(
                                    LaunchConfiguration("corridor_follow_enabled"),
                                    value_type=bool,
                                ),
                                "corridor_front_sector_angle_rad": ParameterValue(
                                    LaunchConfiguration("corridor_front_sector_angle_rad"),
                                    value_type=float,
                                ),
                                "corridor_side_clearance_m": ParameterValue(
                                    LaunchConfiguration("corridor_side_clearance_m"),
                                    value_type=float,
                                ),
                                "corridor_front_stop_distance_m": ParameterValue(
                                    LaunchConfiguration("corridor_front_stop_distance_m"),
                                    value_type=float,
                                ),
                                "corridor_front_slow_distance_m": ParameterValue(
                                    LaunchConfiguration("corridor_front_slow_distance_m"),
                                    value_type=float,
                                ),
                                "corridor_side_slow_distance_m": ParameterValue(
                                    LaunchConfiguration("corridor_side_slow_distance_m"),
                                    value_type=float,
                                ),
                                "corridor_center_kp": ParameterValue(
                                    LaunchConfiguration("corridor_center_kp"),
                                    value_type=float,
                                ),
                                "corridor_wall_avoid_kp": ParameterValue(
                                    LaunchConfiguration("corridor_wall_avoid_kp"),
                                    value_type=float,
                                ),
                                "corridor_max_linear_speed_mps": ParameterValue(
                                    LaunchConfiguration("corridor_max_linear_speed_mps"),
                                    value_type=float,
                                ),
                                "corridor_gap_follow_enabled": ParameterValue(
                                    LaunchConfiguration("corridor_gap_follow_enabled"),
                                    value_type=bool,
                                ),
                                "corridor_gap_half_angle_rad": ParameterValue(
                                    LaunchConfiguration("corridor_gap_half_angle_rad"),
                                    value_type=float,
                                ),
                                "corridor_gap_heading_kp": ParameterValue(
                                    LaunchConfiguration("corridor_gap_heading_kp"),
                                    value_type=float,
                                ),
                                "corridor_gap_target_weight": ParameterValue(
                                    LaunchConfiguration("corridor_gap_target_weight"),
                                    value_type=float,
                                ),
                                "simple_corridor_mission_enabled": ParameterValue(
                                    LaunchConfiguration(
                                        "simple_corridor_mission_enabled"
                                    ),
                                    value_type=bool,
                                ),
                                "simple_corridor_waypoints_text": ParameterValue(
                                    LaunchConfiguration(
                                        "simple_corridor_waypoints_text"
                                    ),
                                    value_type=str,
                                ),
                                "simple_corridor_goal_tolerance_m": ParameterValue(
                                    LaunchConfiguration(
                                        "simple_corridor_goal_tolerance_m"
                                    ),
                                    value_type=float,
                                ),
                                "autonomous_roam_enabled": ParameterValue(
                                    LaunchConfiguration("autonomous_roam_enabled"),
                                    value_type=bool,
                                ),
                                "roam_forward_speed_mps": ParameterValue(
                                    LaunchConfiguration("roam_forward_speed_mps"),
                                    value_type=float,
                                ),
                                "roam_turn_speed_radps": ParameterValue(
                                    LaunchConfiguration("roam_turn_speed_radps"),
                                    value_type=float,
                                ),
                                "roam_front_stop_distance_m": ParameterValue(
                                    LaunchConfiguration("roam_front_stop_distance_m"),
                                    value_type=float,
                                ),
                                "roam_front_slow_distance_m": ParameterValue(
                                    LaunchConfiguration("roam_front_slow_distance_m"),
                                    value_type=float,
                                ),
                                "roam_side_clearance_m": ParameterValue(
                                    LaunchConfiguration("roam_side_clearance_m"),
                                    value_type=float,
                                ),
                                "roam_scan_interval_s": ParameterValue(
                                    LaunchConfiguration("roam_scan_interval_s"),
                                    value_type=float,
                                ),
                            }
                        ],
                    ),
                ],
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
