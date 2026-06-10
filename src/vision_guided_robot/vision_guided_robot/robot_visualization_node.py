from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker

from vision_guided_robot.robot_visualization import FootprintConfig, footprint_corners


class RobotVisualizationNode(Node):
    def __init__(self):
        super().__init__("robot_visualization")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/odom_path")
        self.declare_parameter("footprint_topic", "/robot_footprint")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_path_poses", 500)
        self.declare_parameter("min_path_step_m", 0.02)
        self.declare_parameter("footprint_length_m", 0.34)
        self.declare_parameter("footprint_width_m", 0.24)
        self.declare_parameter("footprint_publish_hz", 5.0)

        self.path = Path()
        self.path.header.frame_id = self.get_parameter("odom_frame").value
        self.last_path_x: float | None = None
        self.last_path_y: float | None = None

        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.footprint_pub = self.create_publisher(
            Marker,
            self.get_parameter("footprint_topic").value,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.on_odom,
            10,
        )

        footprint_publish_hz = max(1.0, float(self.get_parameter("footprint_publish_hz").value))
        self.timer = self.create_timer(1.0 / footprint_publish_hz, self.publish_footprint)

        self.get_logger().info(
            f"Publishing RViz helpers: {self.get_parameter('path_topic').value}, "
            f"{self.get_parameter('footprint_topic').value}"
        )

    def on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        if not self._should_append_pose(float(position.x), float(position.y)):
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)

        max_path_poses = max(1, int(self.get_parameter("max_path_poses").value))
        if len(self.path.poses) > max_path_poses:
            self.path.poses = self.path.poses[-max_path_poses:]

        self.last_path_x = float(position.x)
        self.last_path_y = float(position.y)
        self.path_pub.publish(self.path)

    def publish_footprint(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.get_parameter("base_frame").value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.025
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.35
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        config = FootprintConfig(
            length_m=float(self.get_parameter("footprint_length_m").value),
            width_m=float(self.get_parameter("footprint_width_m").value),
        )
        for x, y in footprint_corners(config):
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.01
            marker.points.append(point)

        self.footprint_pub.publish(marker)

    def _should_append_pose(self, x: float, y: float) -> bool:
        if self.last_path_x is None or self.last_path_y is None:
            return True

        min_path_step_m = max(0.0, float(self.get_parameter("min_path_step_m").value))
        dx = x - self.last_path_x
        dy = y - self.last_path_y
        return (dx * dx + dy * dy) ** 0.5 >= min_path_step_m


def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
