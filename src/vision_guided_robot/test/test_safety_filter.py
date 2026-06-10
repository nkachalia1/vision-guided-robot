import pytest

from vision_guided_robot.safety_filter import (
    DesiredVelocity,
    SafetyConfig,
    SafetyState,
    ScanSample,
    compute_avoidance_velocity,
    filter_velocity,
)


def test_stale_scan_stops_robot():
    desired = DesiredVelocity(linear_x=0.5, angular_z=0.1)

    output = filter_velocity(desired, scan_samples=None, scan_age_s=None)

    assert output.state == SafetyState.STALE_SCAN
    assert output.linear_x == 0.0
    assert output.angular_z == 0.0


def test_clear_path_passes_velocity():
    desired = DesiredVelocity(linear_x=0.5, angular_z=0.2)
    samples = [ScanSample(angle_rad=0.0, range_m=2.0)]

    output = filter_velocity(desired, samples, scan_age_s=0.1)

    assert output.state == SafetyState.CLEAR
    assert output.linear_x == pytest.approx(0.5)
    assert output.angular_z == pytest.approx(0.2)


def test_near_obstacle_slows_forward_motion():
    desired = DesiredVelocity(linear_x=1.0, angular_z=0.0)
    config = SafetyConfig(obstacle_stop_distance_m=0.5, obstacle_slow_distance_m=1.0)
    samples = [ScanSample(angle_rad=0.0, range_m=0.75)]

    output = filter_velocity(desired, samples, scan_age_s=0.1, config=config)

    assert output.state == SafetyState.SLOW
    assert output.linear_x == pytest.approx(0.5)


def test_blocked_obstacle_stops_and_turns():
    desired = DesiredVelocity(linear_x=1.0, angular_z=0.0)
    config = SafetyConfig(obstacle_stop_distance_m=0.5, avoid_turn_speed_radps=0.7)
    samples = [
        ScanSample(angle_rad=-0.5, range_m=2.0),
        ScanSample(angle_rad=0.0, range_m=0.3),
        ScanSample(angle_rad=0.5, range_m=1.0),
    ]

    output = filter_velocity(desired, samples, scan_age_s=0.1, config=config)

    assert output.state == SafetyState.BLOCKED
    assert output.linear_x == 0.0
    assert abs(output.angular_z) == pytest.approx(0.7)


def test_rotation_without_forward_motion_is_allowed():
    desired = DesiredVelocity(linear_x=0.0, angular_z=0.5)
    samples = [ScanSample(angle_rad=0.0, range_m=0.2)]

    output = filter_velocity(desired, samples, scan_age_s=0.1)

    assert output.state == SafetyState.CLEAR
    assert output.linear_x == 0.0
    assert output.angular_z == pytest.approx(0.5)


def test_avoidance_turns_and_moves_slowly_when_front_is_clear_enough():
    config = SafetyConfig(
        obstacle_stop_distance_m=0.35,
        avoid_forward_speed_mps=0.18,
        avoid_turn_speed_radps=0.7,
    )
    samples = [
        ScanSample(angle_rad=0.0, range_m=0.6),
        ScanSample(angle_rad=0.5, range_m=1.5),
    ]

    output = compute_avoidance_velocity(samples, config=config, turn_direction=1.0)

    assert output.state == SafetyState.AVOID
    assert output.linear_x == pytest.approx(0.18)
    assert output.angular_z == pytest.approx(0.7)


def test_avoidance_can_creep_when_obstacle_is_beside_center_path():
    config = SafetyConfig(
        obstacle_stop_distance_m=0.35,
        front_sector_angle_rad=0.5,
        avoid_forward_sector_angle_rad=0.2,
        avoid_forward_speed_mps=0.30,
        avoid_turn_speed_radps=0.9,
    )
    samples = [
        ScanSample(angle_rad=0.0, range_m=1.2),
        ScanSample(angle_rad=0.4, range_m=0.2),
    ]

    output = compute_avoidance_velocity(samples, config=config, turn_direction=1.0)

    assert output.state == SafetyState.AVOID
    assert output.min_front_range_m == pytest.approx(0.2)
    assert output.linear_x == pytest.approx(0.30)
    assert output.angular_z == pytest.approx(0.9)


def test_avoidance_does_not_move_forward_when_still_too_close():
    config = SafetyConfig(
        obstacle_stop_distance_m=0.35,
        avoid_forward_speed_mps=0.18,
        avoid_turn_speed_radps=0.7,
    )
    samples = [ScanSample(angle_rad=0.0, range_m=0.2)]

    output = compute_avoidance_velocity(samples, config=config, turn_direction=-1.0)

    assert output.state == SafetyState.AVOID
    assert output.linear_x == 0.0
    assert output.angular_z == pytest.approx(-0.7)
