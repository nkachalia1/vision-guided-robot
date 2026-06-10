from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from vision_guided_robot.safety_filter import (
    DesiredVelocity,
    SafetyConfig,
    SafetyState,
    ScanSample,
    compute_avoidance_velocity,
    filter_velocity,
)


class SafetyFilterNode(Node):
    def __init__(self):
        super().__init__("safety_filter")

        self.declare_parameter("cmd_vel_raw_topic", "/cmd_vel_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("state_topic", "/safety/state")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("obstacle_stop_distance_m", 0.35)
        self.declare_parameter("obstacle_slow_distance_m", 0.90)
        self.declare_parameter("front_sector_angle_rad", 0.45)
        self.declare_parameter("scan_timeout_s", 0.6)
        self.declare_parameter("avoid_turn_speed_radps", 0.55)
        self.declare_parameter("avoid_forward_speed_mps", 0.50)
        self.declare_parameter("avoid_hold_time_s", 1.2)
        self.declare_parameter("avoid_forward_sector_angle_rad", 0.22)

        self.latest_desired = DesiredVelocity(linear_x=0.0, angular_z=0.0)
        self.latest_scan_samples: list[ScanSample] | None = None
        self.latest_scan_time_s: float | None = None
        self.last_state_name: str | None = None
        self.avoid_until_time_s: float | None = None
        self.avoid_turn_direction: float | None = None

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.state_pub = self.create_publisher(String, self.get_parameter("state_topic").value, 10)
        self.raw_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_raw_topic").value,
            self.on_desired_velocity,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
            qos_profile_sensor_data,
        )

        control_rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / control_rate_hz, self.on_timer)

        self.get_logger().info(
            f"Safety filter: {self.get_parameter('cmd_vel_raw_topic').value} -> "
            f"{self.get_parameter('cmd_vel_topic').value}"
        )

    def on_desired_velocity(self, msg: Twist) -> None:
        self.latest_desired = DesiredVelocity(
            linear_x=float(msg.linear.x),
            angular_z=float(msg.angular.z),
        )

    def on_scan(self, msg: LaserScan) -> None:
        samples = []
        angle = float(msg.angle_min)
        for range_m in msg.ranges:
            if math.isfinite(range_m) and msg.range_min <= range_m <= msg.range_max:
                samples.append(ScanSample(angle_rad=angle, range_m=float(range_m)))
            angle += float(msg.angle_increment)

        self.latest_scan_samples = samples
        self.latest_scan_time_s = self._now_s()

    def on_timer(self) -> None:
        now_s = self._now_s()
        scan_age_s = None
        if self.latest_scan_time_s is not None:
            scan_age_s = now_s - self.latest_scan_time_s

        config = self._config_from_parameters()
        if self._avoidance_is_active(now_s):
            output = compute_avoidance_velocity(
                self.latest_scan_samples,
                config=config,
                turn_direction=self.avoid_turn_direction,
            )
        else:
            output = filter_velocity(
                desired=self.latest_desired,
                scan_samples=self.latest_scan_samples,
                scan_age_s=scan_age_s,
                config=config,
            )
            if output.state == SafetyState.BLOCKED:
                self.avoid_until_time_s = now_s + config.avoid_hold_time_s
                if self.avoid_turn_direction is None:
                    self.avoid_turn_direction = 1.0 if output.angular_z >= 0.0 else -1.0
                output = compute_avoidance_velocity(
                    self.latest_scan_samples,
                    config=config,
                    turn_direction=self.avoid_turn_direction,
                )
            elif output.state == SafetyState.CLEAR:
                self.avoid_turn_direction = None

        twist = Twist()
        twist.linear.x = output.linear_x
        twist.angular.z = output.angular_z
        self.cmd_pub.publish(twist)

        state_msg = String()
        state_msg.data = output.state.value
        self.state_pub.publish(state_msg)

        if state_msg.data != self.last_state_name:
            range_text = "n/a"
            if output.min_front_range_m is not None:
                range_text = f"{output.min_front_range_m:.2f} m"
            self.get_logger().info(f"Safety state: {state_msg.data}, front range: {range_text}")
            self.last_state_name = state_msg.data

    def _config_from_parameters(self) -> SafetyConfig:
        return SafetyConfig(
            obstacle_stop_distance_m=float(self.get_parameter("obstacle_stop_distance_m").value),
            obstacle_slow_distance_m=float(self.get_parameter("obstacle_slow_distance_m").value),
            front_sector_angle_rad=float(self.get_parameter("front_sector_angle_rad").value),
            scan_timeout_s=float(self.get_parameter("scan_timeout_s").value),
            avoid_turn_speed_radps=float(self.get_parameter("avoid_turn_speed_radps").value),
            avoid_forward_speed_mps=float(self.get_parameter("avoid_forward_speed_mps").value),
            avoid_hold_time_s=float(self.get_parameter("avoid_hold_time_s").value),
            avoid_forward_sector_angle_rad=float(
                self.get_parameter("avoid_forward_sector_angle_rad").value
            ),
        )

    def _avoidance_is_active(self, now_s: float) -> bool:
        return self.avoid_until_time_s is not None and now_s < self.avoid_until_time_s

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilterNode()
    try:
        rclpy.spin(node)
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
