from __future__ import annotations

import argparse
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


class RosImageCapture(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("ros_image_capture")
        self.args = args
        self.bridge = CvBridge()
        self.received_count = 0
        self.saved_count = 0
        self.start_time_s = time.monotonic()
        self.last_target_time_s: float | None = None
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.subscription = self.create_subscription(
            Image,
            args.topic,
            self.on_image,
            10,
        )
        self.target_subscription = None
        if args.require_target_topic:
            self.target_subscription = self.create_subscription(
                PointStamped,
                args.require_target_topic,
                self.on_target,
                10,
            )

        self.timer = self.create_timer(0.2, self.check_timeout)
        self.get_logger().info(
            f"Saving {args.count} images from {args.topic} to {self.output_dir}"
        )
        if args.require_target_topic:
            self.get_logger().info(
                "Only saving frames after a recent target message from "
                f"{args.require_target_topic}"
            )

    def on_image(self, msg: Image) -> None:
        self.received_count += 1
        if self.received_count % self.args.every_n != 0:
            return
        if self.saved_count >= self.args.count:
            return
        if not self.has_recent_target():
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover - ROS runtime guard
            self.get_logger().warning(f"Could not convert image: {exc}")
            return

        self.saved_count += 1
        filename = f"{self.args.prefix}_{self.saved_count:04d}.jpg"
        path = self.output_dir / filename
        cv2.imwrite(str(path), frame)
        self.get_logger().info(f"saved {path}")

        if self.saved_count >= self.args.count:
            self.get_logger().info("capture complete")
            rclpy.shutdown()

    def on_target(self, msg: PointStamped) -> None:
        self.last_target_time_s = time.monotonic()

    def has_recent_target(self) -> bool:
        if not self.args.require_target_topic:
            return True
        if self.last_target_time_s is None:
            return False
        age_s = time.monotonic() - self.last_target_time_s
        return age_s <= self.args.target_max_age_s

    def check_timeout(self) -> None:
        if self.args.max_seconds <= 0.0:
            return
        elapsed_s = time.monotonic() - self.start_time_s
        if elapsed_s > self.args.max_seconds:
            self.get_logger().warning(
                f"capture timed out after {elapsed_s:.1f}s; saved {self.saved_count} images"
            )
            rclpy.shutdown()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Save images from a ROS image topic.")
    parser.add_argument("--topic", default="/camera/image", help="ROS image topic to save.")
    parser.add_argument("--output-dir", required=True, help="Directory for saved images.")
    parser.add_argument("--prefix", default="ros_image", help="Filename prefix.")
    parser.add_argument("--count", type=int, default=10, help="Number of images to save.")
    parser.add_argument("--every-n", type=int, default=5, help="Save every Nth received frame.")
    parser.add_argument(
        "--require-target-topic",
        default="",
        help="Only save frames after this PointStamped target topic publishes.",
    )
    parser.add_argument(
        "--target-max-age-s",
        type=float,
        default=0.75,
        help="Maximum age of the required target message.",
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=30.0,
        help="Stop after this many seconds even if count is not reached; <=0 disables.",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args(remove_ros_args()[1:])
    if args.count <= 0:
        raise SystemExit("--count must be positive")
    if args.every_n <= 0:
        raise SystemExit("--every-n must be positive")

    rclpy.init()
    node = RosImageCapture(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
