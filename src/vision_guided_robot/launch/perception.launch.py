from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vision_guided_robot")
    detector_config = PathJoinSubstitution([package_share, "config", "detector.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector_backend",
                default_value="hsv",
                description="Detector backend to use. Current baseline: 'hsv'.",
            ),
            DeclareLaunchArgument(
                "detector_model_path",
                default_value="",
                description="ONNX model path for detector_backend:=onnx/ml/yolo.",
            ),
            DeclareLaunchArgument(
                "detector_class_names_path",
                default_value="",
                description="Optional class-name file for ONNX detector backends.",
            ),
            DeclareLaunchArgument(
                "detector_target_class_names",
                default_value="sports ball",
                description="Comma-separated target classes for ONNX detector backends.",
            ),
            Node(
                package="vision_guided_robot",
                executable="ball_tracker",
                parameters=[
                    detector_config,
                    {
                        "detector_backend": ParameterValue(
                            LaunchConfiguration("detector_backend"),
                            value_type=str,
                        ),
                        "detector_model_path": ParameterValue(
                            LaunchConfiguration("detector_model_path"),
                            value_type=str,
                        ),
                        "detector_class_names_path": ParameterValue(
                            LaunchConfiguration("detector_class_names_path"),
                            value_type=str,
                        ),
                        "detector_target_class_names": ParameterValue(
                            LaunchConfiguration("detector_target_class_names"),
                            value_type=str,
                        ),
                    },
                ],
                output="screen",
            )
        ]
    )
