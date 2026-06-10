from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the mapping/navigation layout.",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=slam_params,
                description="SLAM Toolbox parameter file.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=nav2_params,
                description="Nav2 parameter file for SLAM-produced maps.",
            ),
            DeclareLaunchArgument(
                "slam_start_delay_s",
                default_value="4.0",
                description="Seconds to wait for Gazebo, TF, odom, and scan before SLAM starts.",
            ),
            DeclareLaunchArgument(
                "nav2_start_delay_s",
                default_value="8.0",
                description="Seconds to wait before starting Nav2 navigation.",
            ),
            IncludeLaunchDescription(
                sim_launch,
                launch_arguments={
                    "control_mode": "nav2",
                    "spawn_target": "false",
                    "enable_perception": "false",
                    "enable_safety_filter": "false",
                    "spawn_occluder": "true",
                    "occluder_x": "1.2",
                    "occluder_y": "0.4",
                    "spawn_second_occluder": "true",
                    "second_occluder_x": "1.6",
                    "second_occluder_y": "-0.45",
                    "rviz": "false",
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
                            "slam_params_file": LaunchConfiguration("slam_params_file"),
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("nav2_start_delay_s"),
                actions=[
                    IncludeLaunchDescription(
                        navigation_launch,
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
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
