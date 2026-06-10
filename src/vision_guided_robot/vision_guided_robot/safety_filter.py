from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class SafetyState(str, Enum):
    CLEAR = "CLEAR"
    SLOW = "SLOW"
    BLOCKED = "BLOCKED"
    AVOID = "AVOID"
    STALE_SCAN = "STALE_SCAN"


@dataclass(frozen=True)
class SafetyConfig:
    obstacle_stop_distance_m: float = 0.35
    obstacle_slow_distance_m: float = 0.90
    front_sector_angle_rad: float = 0.45
    scan_timeout_s: float = 0.6
    avoid_turn_speed_radps: float = 0.55
    avoid_forward_speed_mps: float = 0.50
    avoid_hold_time_s: float = 1.2
    avoid_forward_sector_angle_rad: float = 0.22


@dataclass(frozen=True)
class DesiredVelocity:
    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class FilteredVelocity:
    linear_x: float
    angular_z: float
    state: SafetyState
    min_front_range_m: float | None


@dataclass(frozen=True)
class ScanSample:
    angle_rad: float
    range_m: float


def filter_velocity(
    desired: DesiredVelocity,
    scan_samples: list[ScanSample] | None,
    scan_age_s: float | None,
    config: SafetyConfig | None = None,
) -> FilteredVelocity:
    config = config or SafetyConfig()

    if scan_samples is None or scan_age_s is None or scan_age_s > config.scan_timeout_s:
        return FilteredVelocity(
            linear_x=0.0,
            angular_z=0.0,
            state=SafetyState.STALE_SCAN,
            min_front_range_m=None,
        )

    front_ranges = ranges_in_sector(scan_samples, config.front_sector_angle_rad)
    min_front = min(front_ranges) if front_ranges else None

    if desired.linear_x <= 0.0:
        return FilteredVelocity(
            linear_x=desired.linear_x,
            angular_z=desired.angular_z,
            state=SafetyState.CLEAR,
            min_front_range_m=min_front,
        )

    if min_front is None:
        return FilteredVelocity(
            linear_x=0.0,
            angular_z=0.0,
            state=SafetyState.STALE_SCAN,
            min_front_range_m=None,
        )

    if min_front < config.obstacle_stop_distance_m:
        return FilteredVelocity(
            linear_x=0.0,
            angular_z=choose_avoidance_turn(scan_samples, config),
            state=SafetyState.BLOCKED,
            min_front_range_m=min_front,
        )

    if min_front < config.obstacle_slow_distance_m:
        scale = (min_front - config.obstacle_stop_distance_m) / (
            config.obstacle_slow_distance_m - config.obstacle_stop_distance_m
        )
        scale = max(0.0, min(1.0, scale))
        return FilteredVelocity(
            linear_x=desired.linear_x * scale,
            angular_z=desired.angular_z,
            state=SafetyState.SLOW,
            min_front_range_m=min_front,
        )

    return FilteredVelocity(
        linear_x=desired.linear_x,
        angular_z=desired.angular_z,
        state=SafetyState.CLEAR,
        min_front_range_m=min_front,
    )


def compute_avoidance_velocity(
    scan_samples: list[ScanSample] | None,
    config: SafetyConfig | None = None,
    turn_direction: float | None = None,
) -> FilteredVelocity:
    config = config or SafetyConfig()

    if scan_samples is None:
        return FilteredVelocity(
            linear_x=0.0,
            angular_z=0.0,
            state=SafetyState.STALE_SCAN,
            min_front_range_m=None,
        )

    front_ranges = ranges_in_sector(scan_samples, config.front_sector_angle_rad)
    center_ranges = ranges_in_sector(scan_samples, config.avoid_forward_sector_angle_rad)
    min_front = min(front_ranges) if front_ranges else None
    min_center = min(center_ranges) if center_ranges else None

    if turn_direction is None:
        angular_z = choose_avoidance_turn(scan_samples, config)
    else:
        angular_z = sign(turn_direction) * abs(config.avoid_turn_speed_radps)

    linear_x = config.avoid_forward_speed_mps
    if min_center is not None and min_center < config.obstacle_stop_distance_m:
        linear_x = 0.0

    return FilteredVelocity(
        linear_x=linear_x,
        angular_z=angular_z,
        state=SafetyState.AVOID,
        min_front_range_m=min_front,
    )


def ranges_in_sector(scan_samples: list[ScanSample], half_angle_rad: float) -> list[float]:
    return [
        sample.range_m
        for sample in scan_samples
        if abs(sample.angle_rad) <= half_angle_rad and math.isfinite(sample.range_m)
    ]


def choose_avoidance_turn(scan_samples: list[ScanSample], config: SafetyConfig) -> float:
    left_clearance = mean_clearance(scan_samples, lower=0.0, upper=config.front_sector_angle_rad * 2.0)
    right_clearance = mean_clearance(scan_samples, lower=-config.front_sector_angle_rad * 2.0, upper=0.0)
    direction = 1.0 if left_clearance >= right_clearance else -1.0
    return direction * abs(config.avoid_turn_speed_radps)


def mean_clearance(scan_samples: list[ScanSample], lower: float, upper: float) -> float:
    ranges = [
        sample.range_m
        for sample in scan_samples
        if lower <= sample.angle_rad <= upper and math.isfinite(sample.range_m)
    ]
    if not ranges:
        return 0.0
    return sum(ranges) / len(ranges)


def sign(value: float) -> float:
    return 1.0 if value >= 0.0 else -1.0
