from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RobotPose:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class TargetEstimate:
    lateral_m: float
    distance_m: float


@dataclass(frozen=True)
class ApproachGoal:
    x_m: float
    y_m: float
    yaw_rad: float
    target_range_m: float


def compute_approach_goal(
    robot_pose: RobotPose,
    target: TargetEstimate,
    *,
    camera_forward_offset_m: float = 0.19,
    stand_off_distance_m: float = 0.55,
) -> ApproachGoal:
    """Project a camera-frame target estimate into a map-frame approach goal.

    `/ball/relative_position` uses camera optical coordinates:
    x is image-right lateral error and z is forward range. ROS base_link y is
    left, so camera optical x maps to negative base_link y.
    """
    target_base_x = max(0.0, camera_forward_offset_m + target.distance_m)
    target_base_y = -target.lateral_m
    target_range_m = math.hypot(target_base_x, target_base_y)

    if target_range_m <= 1e-6:
        goal_base_x = 0.0
        goal_base_y = 0.0
    else:
        approach_distance_m = max(0.0, target_range_m - stand_off_distance_m)
        scale = approach_distance_m / target_range_m
        goal_base_x = target_base_x * scale
        goal_base_y = target_base_y * scale

    cos_yaw = math.cos(robot_pose.yaw_rad)
    sin_yaw = math.sin(robot_pose.yaw_rad)
    goal_x = robot_pose.x_m + goal_base_x * cos_yaw - goal_base_y * sin_yaw
    goal_y = robot_pose.y_m + goal_base_x * sin_yaw + goal_base_y * cos_yaw
    goal_yaw = normalize_angle(robot_pose.yaw_rad + math.atan2(target_base_y, target_base_x))

    return ApproachGoal(
        x_m=goal_x,
        y_m=goal_y,
        yaw_rad=goal_yaw,
        target_range_m=target_range_m,
    )


def normalize_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad
