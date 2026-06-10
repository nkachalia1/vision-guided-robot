from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from vision_guided_robot.control_law import (
    ControlConfig,
    TargetObservation,
    VelocityCommand,
    compute_visual_servo_command,
)


class VisualServoState(str, Enum):
    SEARCH = "SEARCH"
    RECOVER = "RECOVER"
    TRACK = "TRACK"
    APPROACH = "APPROACH"
    STOP = "STOP"


@dataclass(frozen=True)
class StateMachineConfig:
    target_timeout_s: float = 0.6
    stop_hold_time_s: float = 1.5
    track_angle_threshold_rad: float = 0.18
    search_sweep_period_s: float = 3.0
    recover_timeout_s: float = 0.1


@dataclass(frozen=True)
class StateMachineOutput:
    state: VisualServoState
    command: VelocityCommand
    target_fresh: bool


class VisualServoStateMachine:
    def __init__(
        self,
        config: StateMachineConfig | None = None,
        control_config: ControlConfig | None = None,
    ):
        self.config = config or StateMachineConfig()
        self.control_config = control_config or ControlConfig()
        self.state = VisualServoState.SEARCH
        self.state_entered_time_s = 0.0
        self.last_target_seen_time_s: float | None = None
        self.last_target_lateral_m: float | None = None
        self.recover_until_time_s: float | None = None

    def request_recovery(self, now_s: float, duration_s: float) -> None:
        if self.last_target_lateral_m is None:
            return
        self.recover_until_time_s = now_s + max(0.0, duration_s)

    def update(
        self,
        now_s: float,
        target: TargetObservation | None,
        target_age_s: float | None,
        recovery_allowed: bool = True,
    ) -> StateMachineOutput:
        target_fresh = self._target_is_fresh(target, target_age_s)
        if target_fresh and target is not None:
            self.last_target_seen_time_s = now_s
            if abs(target.lateral_m) > 1e-3:
                self.last_target_lateral_m = target.lateral_m

        next_state = self._choose_state(
            now_s,
            target if target_fresh else None,
            recovery_allowed=recovery_allowed,
        )
        if next_state != self.state:
            self.state = next_state
            self.state_entered_time_s = now_s

        command = self._command_for_state(now_s, target if target_fresh else None)
        return StateMachineOutput(
            state=self.state,
            command=command,
            target_fresh=target_fresh,
        )

    def _target_is_fresh(
        self,
        target: TargetObservation | None,
        target_age_s: float | None,
    ) -> bool:
        return (
            target is not None
            and target_age_s is not None
            and target_age_s <= self.config.target_timeout_s
        )

    def _choose_state(
        self,
        now_s: float,
        target: TargetObservation | None,
        recovery_allowed: bool,
    ) -> VisualServoState:
        if target is None:
            if (
                self.state == VisualServoState.STOP
                and now_s - self.state_entered_time_s < self.config.stop_hold_time_s
            ):
                return VisualServoState.STOP
            if recovery_allowed and self._should_recover(now_s):
                return VisualServoState.RECOVER
            return VisualServoState.SEARCH

        if target.distance_m <= (
            self.control_config.stop_distance_m + self.control_config.distance_tolerance_m
        ):
            if abs(target.lateral_m) > self.control_config.stop_lateral_tolerance_m:
                return VisualServoState.TRACK
            return VisualServoState.STOP

        angle_error_rad = math.atan2(target.lateral_m, max(target.distance_m, 1e-6))
        if abs(angle_error_rad) > self.config.track_angle_threshold_rad:
            return VisualServoState.TRACK

        return VisualServoState.APPROACH

    def _command_for_state(
        self,
        now_s: float,
        target: TargetObservation | None,
    ) -> VelocityCommand:
        if self.state == VisualServoState.SEARCH:
            return VelocityCommand(linear_x=0.0, angular_z=self._search_angular_speed())

        if self.state == VisualServoState.RECOVER:
            return VelocityCommand(linear_x=0.0, angular_z=self._recover_angular_speed())

        if self.state == VisualServoState.STOP:
            return VelocityCommand(linear_x=0.0, angular_z=0.0)

        if target is None:
            return VelocityCommand(linear_x=0.0, angular_z=0.0)

        command = compute_visual_servo_command(target, self.control_config)
        if self.state == VisualServoState.TRACK:
            return VelocityCommand(linear_x=0.0, angular_z=command.angular_z)

        return command

    def _search_angular_speed(self) -> float:
        return self._turn_direction_toward_last_target() * (
            self.control_config.search_angular_speed_radps
        )

    def _should_recover(self, now_s: float) -> bool:
        if self.last_target_seen_time_s is None or self.last_target_lateral_m is None:
            return False
        if now_s - self.last_target_seen_time_s <= self.config.recover_timeout_s:
            return True
        return self.recover_until_time_s is not None and now_s <= self.recover_until_time_s

    def _recover_angular_speed(self) -> float:
        return self._turn_direction_toward_last_target() * (
            self.control_config.recover_angular_speed_radps
        )

    def _turn_direction_toward_last_target(self) -> float:
        if self.last_target_lateral_m is None:
            return 1.0
        return -1.0 if self.last_target_lateral_m > 0.0 else 1.0
