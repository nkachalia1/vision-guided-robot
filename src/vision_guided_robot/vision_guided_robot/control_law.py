from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TargetObservation:
    lateral_m: float
    distance_m: float


@dataclass(frozen=True)
class ControlConfig:
    stop_distance_m: float = 0.45
    distance_tolerance_m: float = 0.04
    stop_lateral_tolerance_m: float = 0.06
    linear_kp: float = 0.45
    angular_kp: float = 1.6
    max_linear_speed_mps: float = 0.35
    max_angular_speed_radps: float = 1.2
    alignment_slowdown_angle_rad: float = 0.7
    search_angular_speed_radps: float = 0.85
    recover_angular_speed_radps: float = 0.35


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float
    angular_z: float


def compute_visual_servo_command(
    target: TargetObservation | None,
    config: ControlConfig | None = None,
) -> VelocityCommand:
    config = config or ControlConfig()
    if target is None:
        return VelocityCommand(linear_x=0.0, angular_z=config.search_angular_speed_radps)

    distance_m = max(0.0, target.distance_m)
    angle_error_rad = math.atan2(target.lateral_m, max(distance_m, 1e-6))
    angular_z = clamp(
        -config.angular_kp * angle_error_rad,
        -config.max_angular_speed_radps,
        config.max_angular_speed_radps,
    )

    if distance_m <= config.stop_distance_m + config.distance_tolerance_m:
        if abs(target.lateral_m) > config.stop_lateral_tolerance_m:
            return VelocityCommand(linear_x=0.0, angular_z=angular_z)
        return VelocityCommand(linear_x=0.0, angular_z=0.0)

    distance_error_m = distance_m - config.stop_distance_m
    linear_x = clamp(config.linear_kp * distance_error_m, 0.0, config.max_linear_speed_mps)

    slowdown_angle = max(config.alignment_slowdown_angle_rad, 1e-6)
    alignment_scale = max(0.0, 1.0 - abs(angle_error_rad) / slowdown_angle)
    linear_x *= alignment_scale

    return VelocityCommand(linear_x=linear_x, angular_z=angular_z)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
