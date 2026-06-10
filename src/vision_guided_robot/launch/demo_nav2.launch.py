from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")
    nav2_share = FindPackageShare("nav2_bringup")

    sim_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, "launch", "sim.launch.py"])
    )
    nav2_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([nav2_share, "launch", "navigation_launch.py"])
    )
    nav2_params = PathJoinSubstitution([package_share, "config", "nav2_params.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start the project RViz layout alongside Nav2.",
            ),
            DeclareLaunchArgument(
                "spawn_occluder",
                default_value="true",
                description="Spawn the wall obstacle used by custom-planner demos.",
            ),
            DeclareLaunchArgument(
                "occluder_x",
                default_value="1.2",
                description="Primary wall obstacle x pose.",
            ),
            DeclareLaunchArgument(
                "occluder_y",
                default_value="0.4",
                description="Primary wall obstacle y pose.",
            ),
            DeclareLaunchArgument(
                "spawn_second_occluder",
                default_value="false",
                description="Spawn a second wall obstacle for harder Nav2 tests.",
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
                "nav2_start_delay_s",
                default_value="4.0",
                description="Seconds to wait for Gazebo, /tf, /odom, and /scan before Nav2 starts.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=nav2_params,
                description="Nav2 parameter file to load.",
            ),
            IncludeLaunchDescription(
                sim_launch,
                launch_arguments={
                    "control_mode": "nav2",
                    "spawn_target": "false",
                    "enable_perception": "false",
                    "enable_safety_filter": "false",
                    "spawn_occluder": LaunchConfiguration("spawn_occluder"),
                    "occluder_x": LaunchConfiguration("occluder_x"),
                    "occluder_y": LaunchConfiguration("occluder_y"),
                    "spawn_second_occluder": LaunchConfiguration("spawn_second_occluder"),
                    "second_occluder_x": LaunchConfiguration("second_occluder_x"),
                    "second_occluder_y": LaunchConfiguration("second_occluder_y"),
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("nav2_start_delay_s"),
                actions=[
                    IncludeLaunchDescription(
                        nav2_launch,
                        launch_arguments={
                            "use_sim_time": "True",
                            "params_file": LaunchConfiguration("nav2_params_file"),
                            "autostart": "True",
                            "use_composition": "False",
                            "use_respawn": "False",
                        }.items(),
                    )
                ],
            ),
        ]
    )
