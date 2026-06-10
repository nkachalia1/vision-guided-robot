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
                "ball_x",
                default_value="2.0",
                description="Ball x pose for the HSV demo.",
            ),
            DeclareLaunchArgument(
                "ball_y",
                default_value="0.0",
                description="Ball y pose for the HSV demo.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz with the project visualization layout.",
            ),
            IncludeLaunchDescription(
                sim_launch,
                launch_arguments={
                    "control_mode": "vision",
                    "ball_x": LaunchConfiguration("ball_x"),
                    "ball_y": LaunchConfiguration("ball_y"),
                    "detector_backend": "hsv",
                    "detector_real_diameter_m": "0.20",
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
        ]
    )
