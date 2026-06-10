from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")

    amcl_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, "launch", "demo_nav2_amcl.launch.py"])
    )
    slam_map = PathJoinSubstitution(
        [package_share, "maps", "slam_two_wall_map_padded.yaml"]
    )
    fast_nav2_params = PathJoinSubstitution(
        [package_share, "config", "nav2_map_wall_pass_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the map-localization layout.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=slam_map,
                description="SLAM-generated padded occupancy map YAML.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=fast_nav2_params,
                description="Fast Nav2 map-frame parameter file.",
            ),
            IncludeLaunchDescription(
                amcl_launch,
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "nav2_params_file": LaunchConfiguration("nav2_params_file"),
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
        ]
    )
