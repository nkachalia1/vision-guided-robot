from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String

from vision_guided_robot.behavior_state_machine import (
    StateMachineConfig,
    VisualServoStateMachine,
)
from vision_guided_robot.control_law import (
    ControlConfig,
    TargetObservation,
)


class VisualServoNode(Node):
    def __init__(self):
        super().__init__("visual_servo")

        self.declare_parameter("target_topic", "/ball/relative_position")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_raw")
        self.declare_parameter("state_topic", "/visual_servo/state")
        self.declare_parameter("safety_state_topic", "/safety/state")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("target_timeout_s", 0.6)
        self.declare_parameter("stop_hold_time_s", 1.5)
        self.declare_parameter("track_angle_threshold_rad", 0.18)
        self.declare_parameter("search_sweep_period_s", 3.0)
        self.declare_parameter("recover_timeout_s", 0.1)
        self.declare_parameter("post_avoid_recover_time_s", 0.1)
        self.declare_parameter("stop_distance_m", 0.45)
        self.declare_parameter("distance_tolerance_m", 0.04)
        self.declare_parameter("stop_lateral_tolerance_m", 0.06)
        self.declare_parameter("linear_kp", 1.0)
        self.declare_parameter("angular_kp", 2.2)
        self.declare_parameter("max_linear_speed_mps", 1.2)
        self.declare_parameter("max_angular_speed_radps", 1.8)
        self.declare_parameter("alignment_slowdown_angle_rad", 0.7)
        self.declare_parameter("search_angular_speed_radps", 0.85)
        self.declare_parameter("recover_angular_speed_radps", 0.35)

        self.config = self._control_config_from_parameters()
        self.state_config = self._state_machine_config_from_parameters()
        self.state_machine = VisualServoStateMachine(self.state_config, self.config)
        self.last_state_name: str | None = None
        self.latest_safety_state = "CLEAR"
        self.latest_target: TargetObservation | None = None
        self.latest_target_time_s: float | None = None

        target_topic = self.get_parameter("target_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        state_topic = self.get_parameter("state_topic").value
        safety_state_topic = self.get_parameter("safety_state_topic").value
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.target_sub = self.create_subscription(PointStamped, target_topic, self.on_target, 10)
        self.safety_state_sub = self.create_subscription(
            String,
            safety_state_topic,
            self.on_safety_state,
            10,
        )

        control_rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / control_rate_hz, self.on_timer)

        self.get_logger().info(f"Visual servo controller publishing to {cmd_vel_topic}")

    def on_target(self, msg: PointStamped) -> None:
        self.latest_target = TargetObservation(
            lateral_m=float(msg.point.x),
            distance_m=float(msg.point.z),
        )
        self.latest_target_time_s = self._now_s()

    def on_safety_state(self, msg: String) -> None:
        previous_state = self.latest_safety_state
        self.latest_safety_state = msg.data
        if previous_state == "AVOID" and msg.data != "AVOID":
            self.state_machine.request_recovery(
                now_s=self._now_s(),
                duration_s=float(self.get_parameter("post_avoid_recover_time_s").value),
            )

    def on_timer(self) -> None:
        now_s = self._now_s()
        self.config = self._control_config_from_parameters()
        self.state_config = self._state_machine_config_from_parameters()
        self.state_machine.control_config = self.config
        self.state_machine.config = self.state_config

        if self.latest_target_time_s is None:
            target_age_s = None
        else:
            target_age_s = now_s - self.latest_target_time_s

        output = self.state_machine.update(
            now_s=now_s,
            target=self.latest_target,
            target_age_s=target_age_s,
            recovery_allowed=self.latest_safety_state not in {"AVOID", "BLOCKED"},
        )

        twist = Twist()
        twist.linear.x = output.command.linear_x
        twist.angular.z = output.command.angular_z
        self.cmd_pub.publish(twist)

        state_msg = String()
        state_msg.data = output.state.value
        self.state_pub.publish(state_msg)

        if state_msg.data != self.last_state_name:
            self.get_logger().info(f"Visual servo state: {state_msg.data}")
            self.last_state_name = state_msg.data

    def _control_config_from_parameters(self) -> ControlConfig:
        return ControlConfig(
            stop_distance_m=float(self.get_parameter("stop_distance_m").value),
            distance_tolerance_m=float(self.get_parameter("distance_tolerance_m").value),
            stop_lateral_tolerance_m=float(
                self.get_parameter("stop_lateral_tolerance_m").value
            ),
            linear_kp=float(self.get_parameter("linear_kp").value),
            angular_kp=float(self.get_parameter("angular_kp").value),
            max_linear_speed_mps=float(self.get_parameter("max_linear_speed_mps").value),
            max_angular_speed_radps=float(self.get_parameter("max_angular_speed_radps").value),
            alignment_slowdown_angle_rad=float(
                self.get_parameter("alignment_slowdown_angle_rad").value
            ),
            search_angular_speed_radps=float(self.get_parameter("search_angular_speed_radps").value),
            recover_angular_speed_radps=float(
                self.get_parameter("recover_angular_speed_radps").value
            ),
        )

    def _state_machine_config_from_parameters(self) -> StateMachineConfig:
        return StateMachineConfig(
            target_timeout_s=float(self.get_parameter("target_timeout_s").value),
            stop_hold_time_s=float(self.get_parameter("stop_hold_time_s").value),
            track_angle_threshold_rad=float(self.get_parameter("track_angle_threshold_rad").value),
            search_sweep_period_s=float(self.get_parameter("search_sweep_period_s").value),
            recover_timeout_s=float(self.get_parameter("recover_timeout_s").value),
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
