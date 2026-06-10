import pytest

from vision_guided_robot.control_law import (
    ControlConfig,
    TargetObservation,
    compute_visual_servo_command,
)


def test_no_target_searches_in_place():
    config = ControlConfig(search_angular_speed_radps=0.3)

    command = compute_visual_servo_command(None, config)

    assert command.linear_x == 0.0
    assert command.angular_z == pytest.approx(0.3)


def test_target_on_right_turns_right():
    target = TargetObservation(lateral_m=0.5, distance_m=2.0)

    command = compute_visual_servo_command(target)

    assert command.angular_z < 0.0


def test_target_on_left_turns_left():
    target = TargetObservation(lateral_m=-0.5, distance_m=2.0)

    command = compute_visual_servo_command(target)

    assert command.angular_z > 0.0


def test_far_centered_target_drives_forward():
    config = ControlConfig(stop_distance_m=0.45)
    target = TargetObservation(lateral_m=0.0, distance_m=2.0)

    command = compute_visual_servo_command(target, config)

    assert command.linear_x > 0.0
    assert command.angular_z == pytest.approx(0.0)


def test_close_target_stops():
    config = ControlConfig(stop_distance_m=0.45, distance_tolerance_m=0.04)
    target = TargetObservation(lateral_m=0.0, distance_m=0.46)

    command = compute_visual_servo_command(target, config)

    assert command.linear_x == 0.0
    assert command.angular_z == 0.0


def test_close_off_center_target_keeps_aligning():
    config = ControlConfig(
        stop_distance_m=0.45,
        distance_tolerance_m=0.04,
        stop_lateral_tolerance_m=0.06,
    )
    target = TargetObservation(lateral_m=-0.15, distance_m=0.49)

    command = compute_visual_servo_command(target, config)

    assert command.linear_x == 0.0
    assert command.angular_z > 0.0


def test_large_angle_slows_forward_motion():
    config = ControlConfig(alignment_slowdown_angle_rad=0.7, max_linear_speed_mps=0.35)
    centered = TargetObservation(lateral_m=0.0, distance_m=2.0)
    off_axis = TargetObservation(lateral_m=2.0, distance_m=2.0)

    centered_command = compute_visual_servo_command(centered, config)
    off_axis_command = compute_visual_servo_command(off_axis, config)

    assert centered_command.linear_x > off_axis_command.linear_x
