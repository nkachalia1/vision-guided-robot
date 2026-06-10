from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")
    nav2_demo_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, "launch", "demo_nav2.launch.py"])
    )
    stress_params = PathJoinSubstitution(
        [package_share, "config", "nav2_recovery_stress_params.yaml"]
    )
    blocker_sdf = PathJoinSubstitution(
        [package_share, "models", "recovery_blocker", "model.sdf"]
    )

    return LaunchDescription(
        [
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
                nav2_demo_launch,
                launch_arguments={
                    "rviz": LaunchConfiguration("rviz"),
                    "spawn_occluder": "false",
                    "spawn_second_occluder": "false",
                    "nav2_params_file": stress_params,
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
