from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")
    sim_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, "launch", "sim.launch.py"])
    )
    blocker_sdf = PathJoinSubstitution(
        [package_share, "models", "recovery_blocker", "model.sdf"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "goal_x_m",
                default_value="1.2",
                description="Blocked goal x in odom frame.",
            ),
            DeclareLaunchArgument(
                "goal_y_m",
                default_value="0.4",
                description="Blocked goal y in odom frame.",
            ),
            DeclareLaunchArgument(
                "blocker_x",
                default_value="1.2",
                description="Recovery blocker x pose.",
            ),
            DeclareLaunchArgument(
                "blocker_y",
                default_value="0.4",
                description="Recovery blocker y pose.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the project visualization layout.",
            ),
            IncludeLaunchDescription(
                sim_launch,
                launch_arguments={
                    "control_mode": "planned",
                    "spawn_target": "false",
                    "enable_perception": "false",
                    "goal_x_m": LaunchConfiguration("goal_x_m"),
                    "goal_y_m": LaunchConfiguration("goal_y_m"),
                    "start_with_parameter_goal": "false",
                    "accept_direct_goal_pose": "false",
                    "planner_start_with_parameter_goal": "true",
                    "path_following_mode": "pure_pursuit",
                    "path_lookahead_distance_m": "0.45",
                    "spawn_occluder": "false",
                    "spawn_second_occluder": "false",
                    "planner_obstacles_text": "",
                    "planner_use_scan_obstacles": "true",
                    "planner_require_scan_for_planning": "true",
                    "planner_replan_on_scan_change": "false",
                    "planner_replan_when_path_blocked": "true",
                    "planner_replan_cooldown_s": "1.0",
                    "planner_keep_last_scan_map": "true",
                    "planner_persistent_scan_map": "true",
                    "planner_scan_memory_time_s": "20.0",
                    "planner_scan_obstacle_inflation_radius_m": "0.30",
                    "enable_recovery_behavior": "true",
                    "recovery_max_attempts": "3",
                    "recovery_backup_time_s": "0.8",
                    "recovery_rotate_time_s": "1.2",
                    "recovery_replan_wait_time_s": "1.0",
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
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
                            blocker_sdf,
                            "-name",
                            "recovery_blocker",
                            "-x",
                            LaunchConfiguration("blocker_x"),
                            "-y",
                            LaunchConfiguration("blocker_y"),
                            "-z",
                            "0.3",
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
