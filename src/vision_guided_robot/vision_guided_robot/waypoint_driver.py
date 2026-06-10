from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class WaypointState(str, Enum):
    WAITING_FOR_GOAL = "WAITING_FOR_GOAL"
    WAITING_FOR_ODOM = "WAITING_FOR_ODOM"
    ROTATE_TO_GOAL = "ROTATE_TO_GOAL"
    DRIVE_TO_GOAL = "DRIVE_TO_GOAL"
    FOLLOW_PATH = "FOLLOW_PATH"
    RECOVERING = "RECOVERING"
    ROTATE_TO_FINAL = "ROTATE_TO_FINAL"
    DONE = "DONE"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class WaypointGoal:
    x: float
    y: float
    yaw: float = 0.0
    use_final_yaw: bool = False


@dataclass(frozen=True)
class WaypointConfig:
    goal_tolerance_m: float = 0.10
    heading_tolerance_rad: float = 0.12
    final_yaw_tolerance_rad: float = 0.12
    linear_kp: float = 0.8
    angular_kp: float = 1.8
    max_linear_speed_mps: float = 0.6
    max_angular_speed_radps: float = 1.2


@dataclass(frozen=True)
class WaypointCommand:
    linear_x: float
    angular_z: float
    state: WaypointState
    distance_to_goal_m: float
    heading_error_rad: float


def compute_waypoint_command(
    pose: Pose2D | None,
    goal: WaypointGoal | None,
    config: WaypointConfig | None = None,
) -> WaypointCommand:
    config = config or WaypointConfig()
    if goal is None:
        return WaypointCommand(
            linear_x=0.0,
            angular_z=0.0,
            state=WaypointState.WAITING_FOR_GOAL,
            distance_to_goal_m=math.inf,
            heading_error_rad=0.0,
        )

    if pose is None:
        return WaypointCommand(
            linear_x=0.0,
            angular_z=0.0,
            state=WaypointState.WAITING_FOR_ODOM,
            distance_to_goal_m=math.inf,
            heading_error_rad=0.0,
        )

    dx = goal.x - pose.x
    dy = goal.y - pose.y
    distance_m = math.hypot(dx, dy)
    desired_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(desired_heading - pose.yaw)

    if distance_m > config.goal_tolerance_m:
        angular_z = clamp(
            config.angular_kp * heading_error,
            -config.max_angular_speed_radps,
            config.max_angular_speed_radps,
        )
        if abs(heading_error) > config.heading_tolerance_rad:
            return WaypointCommand(
                linear_x=0.0,
                angular_z=angular_z,
                state=WaypointState.ROTATE_TO_GOAL,
                distance_to_goal_m=distance_m,
                heading_error_rad=heading_error,
            )

        linear_x = clamp(
            config.linear_kp * distance_m,
            0.0,
            config.max_linear_speed_mps,
        )
        return WaypointCommand(
            linear_x=linear_x,
            angular_z=angular_z,
            state=WaypointState.DRIVE_TO_GOAL,
            distance_to_goal_m=distance_m,
            heading_error_rad=heading_error,
        )

    final_yaw_error = normalize_angle(goal.yaw - pose.yaw)
    if goal.use_final_yaw and abs(final_yaw_error) > config.final_yaw_tolerance_rad:
        angular_z = clamp(
            config.angular_kp * final_yaw_error,
            -config.max_angular_speed_radps,
            config.max_angular_speed_radps,
        )
        return WaypointCommand(
            linear_x=0.0,
            angular_z=angular_z,
            state=WaypointState.ROTATE_TO_FINAL,
            distance_to_goal_m=distance_m,
            heading_error_rad=final_yaw_error,
        )

    return WaypointCommand(
        linear_x=0.0,
        angular_z=0.0,
        state=WaypointState.DONE,
        distance_to_goal_m=distance_m,
        heading_error_rad=final_yaw_error,
    )


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def parse_waypoints_text(
    waypoints_text: str,
    default_use_final_yaw: bool = False,
) -> list[WaypointGoal]:
    waypoints = []
    if not waypoints_text.strip():
        return waypoints

    for index, raw_entry in enumerate(waypoints_text.split(";"), start=1):
        entry = raw_entry.strip()
        if not entry:
            continue

        parts = entry.replace(",", " ").split()
        if len(parts) not in (2, 3):
            raise ValueError(
                f"waypoint {index} must contain x,y or x,y,yaw; got '{raw_entry}'"
            )

        try:
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) == 3 else 0.0
        except ValueError as exc:
            raise ValueError(f"waypoint {index} contains a non-numeric value") from exc

        waypoints.append(
            WaypointGoal(
                x=x,
                y=y,
                yaw=yaw,
                use_final_yaw=default_use_final_yaw or len(parts) == 3,
            )
        )

    return waypoints
