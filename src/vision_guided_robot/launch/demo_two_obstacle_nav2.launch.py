from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")
    nav2_demo_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, "launch", "demo_nav2.launch.py"])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the project visualization layout.",
            ),
            IncludeLaunchDescription(
                nav2_demo_launch,
                launch_arguments={
                    "rviz": LaunchConfiguration("rviz"),
                    "spawn_occluder": "true",
                    "occluder_x": "1.2",
                    "occluder_y": "0.4",
                    "spawn_second_occluder": "true",
                    "second_occluder_x": "1.6",
                    "second_occluder_y": "-0.45",
                }.items(),
            ),
        ]
    )
