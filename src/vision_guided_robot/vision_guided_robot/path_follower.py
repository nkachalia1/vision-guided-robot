from __future__ import annotations

from dataclasses import dataclass
import math

from vision_guided_robot.waypoint_driver import (
    Pose2D,
    WaypointCommand,
    WaypointGoal,
    WaypointState,
    clamp,
    normalize_angle,
)


@dataclass(frozen=True)
class PathFollowerConfig:
    goal_tolerance_m: float = 0.10
    lookahead_distance_m: float = 0.45
    linear_kp: float = 1.1
    angular_kp: float = 2.2
    max_linear_speed_mps: float = 0.9
    max_angular_speed_radps: float = 1.8
    heading_slowdown_angle_rad: float = 1.0
    stop_heading_error_rad: float = 1.45


@dataclass(frozen=True)
class LookaheadTarget:
    x: float
    y: float
    distance_to_final_m: float
    distance_to_target_m: float


def compute_path_following_command(
    pose: Pose2D | None,
    path: list[WaypointGoal],
    config: PathFollowerConfig | None = None,
) -> WaypointCommand:
    config = config or PathFollowerConfig()
    if not path:
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

    target = select_lookahead_target(
        pose=pose,
        path=path,
        lookahead_distance_m=max(0.01, config.lookahead_distance_m),
    )
    heading_error = target_heading_error(pose, target.x, target.y)

    if target.distance_to_final_m <= config.goal_tolerance_m:
        return WaypointCommand(
            linear_x=0.0,
            angular_z=0.0,
            state=WaypointState.DONE,
            distance_to_goal_m=target.distance_to_final_m,
            heading_error_rad=heading_error,
        )

    angular_z = clamp(
        config.angular_kp * heading_error,
        -config.max_angular_speed_radps,
        config.max_angular_speed_radps,
    )

    if abs(heading_error) >= config.stop_heading_error_rad:
        linear_x = 0.0
    else:
        slowdown = 1.0
        if config.heading_slowdown_angle_rad > 0.0:
            slowdown = clamp(
                1.0 - abs(heading_error) / config.heading_slowdown_angle_rad,
                0.20,
                1.0,
            )
        linear_x = clamp(
            config.linear_kp * target.distance_to_final_m * slowdown,
            0.0,
            config.max_linear_speed_mps,
        )

    return WaypointCommand(
        linear_x=linear_x,
        angular_z=angular_z,
        state=WaypointState.FOLLOW_PATH,
        distance_to_goal_m=target.distance_to_final_m,
        heading_error_rad=heading_error,
    )


def select_lookahead_target(
    pose: Pose2D,
    path: list[WaypointGoal],
    lookahead_distance_m: float,
) -> LookaheadTarget:
    if len(path) == 1:
        goal = path[0]
        return LookaheadTarget(
            x=goal.x,
            y=goal.y,
            distance_to_final_m=math.hypot(goal.x - pose.x, goal.y - pose.y),
            distance_to_target_m=math.hypot(goal.x - pose.x, goal.y - pose.y),
        )

    points = [(goal.x, goal.y) for goal in path]
    segment_lengths = [
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(points, points[1:])
    ]
    cumulative = [0.0]
    for length in segment_lengths:
        cumulative.append(cumulative[-1] + length)

    total_length = cumulative[-1]
    if total_length <= 1e-9:
        final_x, final_y = points[-1]
        return LookaheadTarget(
            x=final_x,
            y=final_y,
            distance_to_final_m=math.hypot(final_x - pose.x, final_y - pose.y),
            distance_to_target_m=math.hypot(final_x - pose.x, final_y - pose.y),
        )

    closest_s = _closest_path_distance_s(pose, points, segment_lengths, cumulative)
    target_s = min(total_length, closest_s + lookahead_distance_m)
    target_x, target_y = _interpolate_path_point(points, segment_lengths, cumulative, target_s)
    final_x, final_y = points[-1]

    return LookaheadTarget(
        x=target_x,
        y=target_y,
        distance_to_final_m=math.hypot(final_x - pose.x, final_y - pose.y),
        distance_to_target_m=math.hypot(target_x - pose.x, target_y - pose.y),
    )


def target_heading_error(pose: Pose2D, target_x: float, target_y: float) -> float:
    desired_heading = math.atan2(target_y - pose.y, target_x - pose.x)
    return normalize_angle(desired_heading - pose.yaw)


def _closest_path_distance_s(
    pose: Pose2D,
    points: list[tuple[float, float]],
    segment_lengths: list[float],
    cumulative: list[float],
) -> float:
    best_distance_sq = math.inf
    best_s = 0.0

    for index, length in enumerate(segment_lengths):
        if length <= 1e-9:
            continue

        x0, y0 = points[index]
        x1, y1 = points[index + 1]
        dx = x1 - x0
        dy = y1 - y0
        t = clamp(((pose.x - x0) * dx + (pose.y - y0) * dy) / (length * length), 0.0, 1.0)
        proj_x = x0 + t * dx
        proj_y = y0 + t * dy
        distance_sq = (pose.x - proj_x) ** 2 + (pose.y - proj_y) ** 2
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_s = cumulative[index] + t * length

    return best_s


def _interpolate_path_point(
    points: list[tuple[float, float]],
    segment_lengths: list[float],
    cumulative: list[float],
    target_s: float,
) -> tuple[float, float]:
    for index, length in enumerate(segment_lengths):
        segment_start_s = cumulative[index]
        segment_end_s = cumulative[index + 1]
        if length <= 1e-9:
            continue
        if target_s <= segment_end_s:
            t = clamp((target_s - segment_start_s) / length, 0.0, 1.0)
            x0, y0 = points[index]
            x1, y1 = points[index + 1]
            return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))

    return points[-1]
