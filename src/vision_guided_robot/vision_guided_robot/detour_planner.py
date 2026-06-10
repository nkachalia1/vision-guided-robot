from __future__ import annotations

from dataclasses import dataclass
import math

from vision_guided_robot.waypoint_driver import Pose2D, WaypointGoal


@dataclass(frozen=True)
class DetourConfig:
    forward_offset_m: float = 0.60
    lateral_offset_m: float = 1.00
    near_goal_heading_distance_m: float = 0.25


@dataclass(frozen=True)
class DetourPlan:
    goal: WaypointGoal
    resume_goal: WaypointGoal
    side_sign: float


def plan_detour(
    pose: Pose2D | None,
    resume_goal: WaypointGoal | None,
    side_sign: float,
    config: DetourConfig | None = None,
) -> DetourPlan | None:
    if pose is None or resume_goal is None:
        return None

    config = config or DetourConfig()
    side = 1.0 if side_sign >= 0.0 else -1.0

    dx = resume_goal.x - pose.x
    dy = resume_goal.y - pose.y
    distance_m = math.hypot(dx, dy)
    if distance_m < config.near_goal_heading_distance_m:
        path_x = math.cos(pose.yaw)
        path_y = math.sin(pose.yaw)
    else:
        path_x = dx / distance_m
        path_y = dy / distance_m

    left_x = -path_y
    left_y = path_x
    detour_x = (
        pose.x
        + path_x * config.forward_offset_m
        + left_x * side * config.lateral_offset_m
    )
    detour_y = (
        pose.y
        + path_y * config.forward_offset_m
        + left_y * side * config.lateral_offset_m
    )
    detour_yaw = math.atan2(resume_goal.y - detour_y, resume_goal.x - detour_x)

    return DetourPlan(
        goal=WaypointGoal(x=detour_x, y=detour_y, yaw=detour_yaw, use_final_yaw=False),
        resume_goal=resume_goal,
        side_sign=side,
    )
