import os
from glob import glob

from setuptools import find_packages, setup

package_name = "vision_guided_robot"


def collect_data_files(directory):
    data_files = []
    for root, _, files in os.walk(directory):
        if not files:
            continue
        share_dir = os.path.join("share", package_name, root)
        data_files.append((share_dir, [os.path.join(root, file_name) for file_name in files]))
    return data_files


data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    (os.path.join("share", package_name, "maps"), glob("maps/*")),
    (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
]
data_files.extend(collect_data_files("models"))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Student",
    maintainer_email="student@example.com",
    description="Vision-guided differential-drive robot for ROS 2 and Gazebo.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ball_tracker = vision_guided_robot.ball_tracker_node:main",
            "visual_servo = vision_guided_robot.visual_servo_node:main",
            "safety_filter = vision_guided_robot.safety_filter_node:main",
            "waypoint_driver = vision_guided_robot.waypoint_driver_node:main",
            "grid_planner = vision_guided_robot.grid_planner_node:main",
            "frontier_explorer = vision_guided_robot.frontier_explorer_node:main",
            "robot_visualization = vision_guided_robot.robot_visualization_node:main",
            "webcam_detector = vision_guided_robot.webcam_detector:main",
            "distance_calibrator = vision_guided_robot.distance_calibrator:main",
            "detector_evaluator = vision_guided_robot.detector_evaluator:main",
            "detector_compare = vision_guided_robot.detector_compare:main",
            "dataset_prep = vision_guided_robot.dataset_prep:main",
            "manual_label = vision_guided_robot.manual_label:main",
            "manual_label_batch = vision_guided_robot.manual_label_batch:main",
            "dataset_audit = vision_guided_robot.dataset_audit:main",
            "ros_image_capture = vision_guided_robot.ros_image_capture:main",
        ],
    },
)
