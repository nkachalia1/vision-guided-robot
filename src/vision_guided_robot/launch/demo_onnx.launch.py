from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")
    sim_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, "launch", "sim.launch.py"])
    )
    default_model_path = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "vision_guided_robot_ws",
            "models",
            "ml",
            "red_ball_yolo11n_best.onnx",
        ]
    )
    default_class_names_path = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "vision_guided_robot_ws",
            "models",
            "ml",
            "red_ball_classes.txt",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ball_x",
                default_value="2.0",
                description="Ball x pose for the ONNX demo.",
            ),
            DeclareLaunchArgument(
                "ball_y",
                default_value="0.0",
                description="Ball y pose for the ONNX demo.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz with the project visualization layout.",
            ),
            DeclareLaunchArgument(
                "detector_model_path",
                default_value=default_model_path,
                description="Best custom red-ball ONNX model path.",
            ),
            DeclareLaunchArgument(
                "detector_class_names_path",
                default_value=default_class_names_path,
                description="Class-name file for the custom red-ball ONNX model.",
            ),
            IncludeLaunchDescription(
                sim_launch,
                launch_arguments={
                    "control_mode": "vision",
                    "ball_x": LaunchConfiguration("ball_x"),
                    "ball_y": LaunchConfiguration("ball_y"),
                    "detector_backend": "onnx",
                    "detector_model_path": LaunchConfiguration("detector_model_path"),
                    "detector_class_names_path": LaunchConfiguration(
                        "detector_class_names_path"
                    ),
                    "detector_target_class_names": "red_ball",
                    "detector_confidence_threshold": "0.10",
                    "detector_real_diameter_m": "0.272",
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
        ]
    )
