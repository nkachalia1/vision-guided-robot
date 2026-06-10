from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")
    sim_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, "launch", "sim.launch.py"])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "goal_x_m",
                default_value="2.0",
                description="Live-planned navigation goal x in odom frame.",
            ),
            DeclareLaunchArgument(
                "goal_y_m",
                default_value="0.8",
                description="Live-planned navigation goal y in odom frame.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the project visualization layout.",
            ),
            DeclareLaunchArgument(
                "path_following_mode",
                default_value="pure_pursuit",
                description="Path follower for planned navigation: 'pure_pursuit' or 'waypoint'.",
            ),
            DeclareLaunchArgument(
                "path_lookahead_distance_m",
                default_value="0.45",
                description="Lookahead distance for pure-pursuit path following.",
            ),
            DeclareLaunchArgument(
                "planner_persistent_scan_map",
                default_value="true",
                description="Accumulate live scan obstacle cells in a short-lived costmap.",
            ),
            DeclareLaunchArgument(
                "planner_scan_memory_time_s",
                default_value="12.0",
                description="Seconds that observed scan cells remain in the costmap.",
            ),
            DeclareLaunchArgument(
                "enable_recovery_behavior",
                default_value="true",
                description="Enable backup/rotate/clear-costmap recovery behavior.",
            ),
            DeclareLaunchArgument(
                "spawn_second_occluder",
                default_value="false",
                description="Spawn a second wall obstacle for harder planned-navigation tests.",
            ),
            DeclareLaunchArgument(
                "second_occluder_x",
                default_value="1.6",
                description="Second wall obstacle x pose.",
            ),
            DeclareLaunchArgument(
                "second_occluder_y",
                default_value="-0.45",
                description="Second wall obstacle y pose.",
            ),
            DeclareLaunchArgument(
                "recovery_max_attempts",
                default_value="2",
                description="Maximum recovery attempts before giving up.",
            ),
            DeclareLaunchArgument(
                "recovery_backup_time_s",
                default_value="0.8",
                description="Seconds to back up during recovery.",
            ),
            DeclareLaunchArgument(
                "recovery_rotate_time_s",
                default_value="1.2",
                description="Seconds to rotate during recovery.",
            ),
            DeclareLaunchArgument(
                "recovery_replan_wait_time_s",
                default_value="1.0",
                description="Seconds to wait after clearing the costmap.",
            ),
            IncludeLaunchDescription(
                sim_launch,
                launch_arguments={
                    "control_mode": "planned",
                    "goal_x_m": LaunchConfiguration("goal_x_m"),
                    "goal_y_m": LaunchConfiguration("goal_y_m"),
                    "start_with_parameter_goal": "false",
                    "accept_direct_goal_pose": "false",
                    "planner_start_with_parameter_goal": "true",
                    "path_following_mode": LaunchConfiguration("path_following_mode"),
                    "path_lookahead_distance_m": LaunchConfiguration(
                        "path_lookahead_distance_m"
                    ),
                    "spawn_occluder": "true",
                    "occluder_x": "1.2",
                    "occluder_y": "0.4",
                    "spawn_second_occluder": LaunchConfiguration("spawn_second_occluder"),
                    "second_occluder_x": LaunchConfiguration("second_occluder_x"),
                    "second_occluder_y": LaunchConfiguration("second_occluder_y"),
                    "planner_obstacles_text": "",
                    "planner_use_scan_obstacles": "true",
                    "planner_require_scan_for_planning": "true",
                    "planner_replan_on_scan_change": "false",
                    "planner_replan_when_path_blocked": "true",
                    "planner_replan_cooldown_s": "2.0",
                    "planner_keep_last_scan_map": "true",
                    "planner_persistent_scan_map": LaunchConfiguration(
                        "planner_persistent_scan_map"
                    ),
                    "planner_scan_memory_time_s": LaunchConfiguration(
                        "planner_scan_memory_time_s"
                    ),
                    "planner_scan_obstacle_inflation_radius_m": "0.25",
                    "enable_recovery_behavior": LaunchConfiguration(
                        "enable_recovery_behavior"
                    ),
                    "recovery_max_attempts": LaunchConfiguration("recovery_max_attempts"),
                    "recovery_backup_time_s": LaunchConfiguration(
                        "recovery_backup_time_s"
                    ),
                    "recovery_rotate_time_s": LaunchConfiguration(
                        "recovery_rotate_time_s"
                    ),
                    "recovery_replan_wait_time_s": LaunchConfiguration(
                        "recovery_replan_wait_time_s"
                    ),
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
        ]
    )
