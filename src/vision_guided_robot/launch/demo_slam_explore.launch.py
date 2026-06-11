from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")

    slam_demo = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, "launch", "demo_slam.launch.py"])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the mapping/navigation layout.",
            ),
            DeclareLaunchArgument(
                "explorer_start_delay_s",
                default_value="10.0",
                description="Seconds to wait before starting frontier exploration.",
            ),
            DeclareLaunchArgument(
                "goal_cooldown_s",
                default_value="0.5",
                description="Seconds to wait after a frontier goal finishes before selecting another.",
            ),
            DeclareLaunchArgument(
                "max_goals",
                default_value="4",
                description="Maximum frontier goals to send; 0 means unlimited.",
            ),
            DeclareLaunchArgument(
                "max_goal_distance_m",
                default_value="3.5",
                description="Reject frontier goals farther than this from the robot.",
            ),
            DeclareLaunchArgument(
                "obstacle_clearance_m",
                default_value="0.25",
                description="Reject frontier goals near occupied cells by this clearance.",
            ),
            DeclareLaunchArgument(
                "min_cluster_size",
                default_value="4",
                description="Minimum number of connected frontier cells before sending a goal.",
            ),
            DeclareLaunchArgument(
                "distance_weight",
                default_value="0.8",
                description="Higher values prefer closer frontiers; lower values prefer larger information gain.",
            ),
            IncludeLaunchDescription(
                slam_demo,
                launch_arguments={"rviz": LaunchConfiguration("rviz")}.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("explorer_start_delay_s"),
                actions=[
                    Node(
                        package="vision_guided_robot",
                        executable="frontier_explorer",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "max_goals": ParameterValue(
                                    LaunchConfiguration("max_goals"),
                                    value_type=int,
                                ),
                                "goal_cooldown_s": ParameterValue(
                                    LaunchConfiguration("goal_cooldown_s"),
                                    value_type=float,
                                ),
                                "max_goal_distance_m": ParameterValue(
                                    LaunchConfiguration("max_goal_distance_m"),
                                    value_type=float,
                                ),
                                "obstacle_clearance_m": ParameterValue(
                                    LaunchConfiguration("obstacle_clearance_m"),
                                    value_type=float,
                                ),
                                "min_cluster_size": ParameterValue(
                                    LaunchConfiguration("min_cluster_size"),
                                    value_type=int,
                                ),
                                "distance_weight": ParameterValue(
                                    LaunchConfiguration("distance_weight"),
                                    value_type=float,
                                ),
                            }
                        ],
                    )
                ],
            ),
        ]
    )
