from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image

from vision_guided_robot.detector_backend import create_detector_backend
from vision_guided_robot.red_ball_detector import (
    DetectorConfig,
    annotate_detection,
    estimate_lateral_offset_m,
)


class BallTrackerNode(Node):
    def __init__(self):
        super().__init__("ball_tracker")

        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("target_topic", "/ball/relative_position")
        self.declare_parameter("annotated_image_topic", "/ball/annotated_image")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("detector_backend", "hsv")
        self.declare_parameter("detector_model_path", "")
        self.declare_parameter("detector_class_names_path", "")
        self.declare_parameter("detector_target_class_names", "sports ball")
        self.declare_parameter("detector_confidence_threshold", 0.25)
        self.declare_parameter("detector_nms_threshold", 0.45)
        self.declare_parameter("detector_input_size_px", 640)
        self.declare_parameter("min_area_px", 150.0)
        self.declare_parameter("min_circularity", 0.55)
        self.declare_parameter("real_diameter_m", 0.20)
        self.declare_parameter("horizontal_fov_rad", 1.047)
        self.declare_parameter("blur_kernel_size", 5)
        self.declare_parameter("morph_kernel_size", 5)
        self.declare_parameter("lower_red_1", [0, 90, 80])
        self.declare_parameter("upper_red_1", [10, 255, 255])
        self.declare_parameter("lower_red_2", [170, 90, 80])
        self.declare_parameter("upper_red_2", [180, 255, 255])

        self.bridge = CvBridge()
        self.config = self._detector_config_from_parameters()
        self.backend_name = str(self.get_parameter("detector_backend").value)
        self.detector = create_detector_backend(
            self.backend_name,
            self.config,
            model_path=_empty_to_none(self.get_parameter("detector_model_path").value),
            class_names_path=_empty_to_none(
                self.get_parameter("detector_class_names_path").value
            ),
            target_class_names=_class_names_from_parameter(
                self.get_parameter("detector_target_class_names").value
            ),
            confidence_threshold=float(
                self.get_parameter("detector_confidence_threshold").value
            ),
            nms_threshold=float(self.get_parameter("detector_nms_threshold").value),
            input_size_px=int(self.get_parameter("detector_input_size_px").value),
        )

        image_topic = self.get_parameter("image_topic").value
        target_topic = self.get_parameter("target_topic").value
        annotated_topic = self.get_parameter("annotated_image_topic").value

        self.target_pub = self.create_publisher(PointStamped, target_topic, 10)
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Tracking red ball from {image_topic} using {self.backend_name} backend"
        )

    def on_image(self, msg: Image) -> None:
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Could not convert image: {exc}")
            return

        detection = self.detector.detect(bgr_image)

        if detection is not None and detection.distance_m is not None:
            target_msg = PointStamped()
            target_msg.header = msg.header
            target_msg.header.frame_id = self.get_parameter("camera_frame").value
            target_msg.point.x = estimate_lateral_offset_m(
                detection.center_x,
                detection.distance_m,
                bgr_image.shape[1],
                self.config.horizontal_fov_rad,
            )
            target_msg.point.y = 0.0
            target_msg.point.z = detection.distance_m
            self.target_pub.publish(target_msg)

        if self.get_parameter("publish_annotated_image").value:
            annotated = annotate_detection(bgr_image, detection)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = msg.header
            annotated_msg.header.frame_id = self.get_parameter("camera_frame").value
            self.annotated_pub.publish(annotated_msg)

    def _detector_config_from_parameters(self) -> DetectorConfig:
        return DetectorConfig(
            min_area_px=float(self.get_parameter("min_area_px").value),
            min_circularity=float(self.get_parameter("min_circularity").value),
            real_diameter_m=float(self.get_parameter("real_diameter_m").value),
            horizontal_fov_rad=float(self.get_parameter("horizontal_fov_rad").value),
            blur_kernel_size=int(self.get_parameter("blur_kernel_size").value),
            morph_kernel_size=int(self.get_parameter("morph_kernel_size").value),
            lower_red_1=_as_hsv_tuple(self.get_parameter("lower_red_1").value),
            upper_red_1=_as_hsv_tuple(self.get_parameter("upper_red_1").value),
            lower_red_2=_as_hsv_tuple(self.get_parameter("lower_red_2").value),
            upper_red_2=_as_hsv_tuple(self.get_parameter("upper_red_2").value),
        )


def _as_hsv_tuple(values) -> tuple[int, int, int]:
    if len(values) != 3:
        raise ValueError("HSV parameter must contain exactly three values")
    return tuple(int(value) for value in values)


def _empty_to_none(value) -> str | None:
    text = str(value).strip()
    return text or None


def _class_names_from_parameter(value) -> list[str]:
    if isinstance(value, str):
        return [part.strip() for part in value.split(",") if part.strip()]
    return [str(part).strip() for part in value if str(part).strip()]


def main(args=None):
    rclpy.init(args=args)
    node = BallTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
