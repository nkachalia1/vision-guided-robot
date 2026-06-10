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
                description="Planned-navigation goal x in odom frame.",
            ),
            DeclareLaunchArgument(
                "goal_y_m",
                default_value="0.8",
                description="Planned-navigation goal y in odom frame.",
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
                    "planner_obstacles_text": "1.2,0.4,0.10,0.80",
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
        ]
    )
