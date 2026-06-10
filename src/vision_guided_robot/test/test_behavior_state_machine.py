import pytest

from vision_guided_robot.behavior_state_machine import (
    StateMachineConfig,
    VisualServoState,
    VisualServoStateMachine,
)
from vision_guided_robot.control_law import ControlConfig, TargetObservation


def test_missing_target_enters_search():
    machine = VisualServoStateMachine(
        StateMachineConfig(target_timeout_s=0.5),
        ControlConfig(search_angular_speed_radps=0.3),
    )

    output = machine.update(now_s=1.0, target=None, target_age_s=None)

    assert output.state == VisualServoState.SEARCH
    assert output.command.linear_x == 0.0
    assert output.command.angular_z == pytest.approx(0.3)
    assert not output.target_fresh


def test_search_rotates_continuously():
    machine = VisualServoStateMachine(
        StateMachineConfig(search_sweep_period_s=2.0),
        ControlConfig(search_angular_speed_radps=0.25),
    )

    first = machine.update(now_s=0.0, target=None, target_age_s=None)
    second = machine.update(now_s=2.1, target=None, target_age_s=None)

    assert first.command.angular_z == pytest.approx(0.25)
    assert second.command.angular_z == pytest.approx(0.25)


def test_search_uses_last_seen_target_direction():
    machine = VisualServoStateMachine(
        StateMachineConfig(target_timeout_s=0.5, recover_timeout_s=0.2),
        ControlConfig(search_angular_speed_radps=0.25),
    )
    target = TargetObservation(lateral_m=0.5, distance_m=2.0)

    machine.update(now_s=1.0, target=target, target_age_s=0.0)
    output = machine.update(now_s=2.0, target=target, target_age_s=1.0)

    assert output.state == VisualServoState.SEARCH
    assert output.command.angular_z == pytest.approx(-0.25)


def test_lost_target_recovers_toward_last_seen_side_before_searching():
    machine = VisualServoStateMachine(
        StateMachineConfig(target_timeout_s=0.5, recover_timeout_s=1.2),
        ControlConfig(search_angular_speed_radps=0.25, recover_angular_speed_radps=0.16),
    )
    target = TargetObservation(lateral_m=0.5, distance_m=2.0)

    machine.update(now_s=1.0, target=target, target_age_s=0.0)
    recover = machine.update(now_s=1.7, target=target, target_age_s=0.7)
    search = machine.update(now_s=2.3, target=target, target_age_s=1.3)

    assert recover.state == VisualServoState.RECOVER
    assert recover.command.linear_x == 0.0
    assert recover.command.angular_z == pytest.approx(-0.16)
    assert search.state == VisualServoState.SEARCH


def test_recovery_can_be_blocked_while_safety_owns_motion():
    machine = VisualServoStateMachine(
        StateMachineConfig(target_timeout_s=0.5, recover_timeout_s=1.2),
        ControlConfig(search_angular_speed_radps=0.25, recover_angular_speed_radps=0.16),
    )
    target = TargetObservation(lateral_m=0.5, distance_m=2.0)

    machine.update(now_s=1.0, target=target, target_age_s=0.0)
    output = machine.update(
        now_s=1.7,
        target=target,
        target_age_s=0.7,
        recovery_allowed=False,
    )

    assert output.state == VisualServoState.SEARCH


def test_post_avoid_recovery_restarts_after_normal_recovery_window():
    machine = VisualServoStateMachine(
        StateMachineConfig(target_timeout_s=0.5, recover_timeout_s=1.2),
        ControlConfig(search_angular_speed_radps=0.25, recover_angular_speed_radps=0.16),
    )
    target = TargetObservation(lateral_m=-0.5, distance_m=2.0)

    machine.update(now_s=1.0, target=target, target_age_s=0.0)
    machine.update(now_s=2.6, target=target, target_age_s=1.6)
    machine.request_recovery(now_s=4.0, duration_s=2.0)
    recover = machine.update(now_s=4.2, target=target, target_age_s=3.2)

    assert recover.state == VisualServoState.RECOVER
    assert recover.command.angular_z == pytest.approx(0.16)


def test_far_off_axis_target_enters_track_without_forward_motion():
    machine = VisualServoStateMachine(
        StateMachineConfig(track_angle_threshold_rad=0.1),
        ControlConfig(),
    )
    target = TargetObservation(lateral_m=-1.0, distance_m=2.0)

    output = machine.update(now_s=1.0, target=target, target_age_s=0.0)

    assert output.state == VisualServoState.TRACK
    assert output.command.linear_x == 0.0
    assert output.command.angular_z > 0.0


def test_centered_far_target_enters_approach():
    machine = VisualServoStateMachine(
        StateMachineConfig(track_angle_threshold_rad=0.2),
        ControlConfig(stop_distance_m=0.45),
    )
    target = TargetObservation(lateral_m=0.0, distance_m=2.0)

    output = machine.update(now_s=1.0, target=target, target_age_s=0.0)

    assert output.state == VisualServoState.APPROACH
    assert output.command.linear_x > 0.0
    assert output.command.angular_z == pytest.approx(0.0)


def test_close_target_enters_stop():
    machine = VisualServoStateMachine(
        StateMachineConfig(),
        ControlConfig(stop_distance_m=0.45, distance_tolerance_m=0.04),
    )
    target = TargetObservation(lateral_m=0.0, distance_m=0.47)

    output = machine.update(now_s=1.0, target=target, target_age_s=0.0)

    assert output.state == VisualServoState.STOP
    assert output.command.linear_x == 0.0
    assert output.command.angular_z == 0.0


def test_close_off_center_target_stays_in_track():
    machine = VisualServoStateMachine(
        StateMachineConfig(),
        ControlConfig(
            stop_distance_m=0.45,
            distance_tolerance_m=0.04,
            stop_lateral_tolerance_m=0.06,
        ),
    )
    target = TargetObservation(lateral_m=-0.15, distance_m=0.49)

    output = machine.update(now_s=1.0, target=target, target_age_s=0.0)

    assert output.state == VisualServoState.TRACK
    assert output.command.linear_x == 0.0
    assert output.command.angular_z > 0.0


def test_stop_holds_through_short_target_flicker():
    machine = VisualServoStateMachine(
        StateMachineConfig(stop_hold_time_s=1.5),
        ControlConfig(stop_distance_m=0.45, distance_tolerance_m=0.04),
    )
    close_target = TargetObservation(lateral_m=0.0, distance_m=0.47)

    stopped = machine.update(now_s=1.0, target=close_target, target_age_s=0.0)
    flicker = machine.update(now_s=1.5, target=None, target_age_s=None)

    assert stopped.state == VisualServoState.STOP
    assert flicker.state == VisualServoState.STOP
    assert flicker.command.linear_x == 0.0
    assert flicker.command.angular_z == 0.0


def test_stop_hold_expires_back_to_search():
    machine = VisualServoStateMachine(
        StateMachineConfig(stop_hold_time_s=1.5),
        ControlConfig(stop_distance_m=0.45, distance_tolerance_m=0.04),
    )
    close_target = TargetObservation(lateral_m=0.0, distance_m=0.47)

    machine.update(now_s=1.0, target=close_target, target_age_s=0.0)
    output = machine.update(now_s=3.0, target=None, target_age_s=None)

    assert output.state == VisualServoState.SEARCH
    assert output.command.angular_z > 0.0
