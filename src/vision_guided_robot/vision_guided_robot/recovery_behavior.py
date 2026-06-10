from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class RecoveryPhase(str, Enum):
    IDLE = "IDLE"
    BACK_UP = "BACK_UP"
    ROTATE = "ROTATE"
    CLEAR_COSTMAP = "CLEAR_COSTMAP"
    WAIT_FOR_REPLAN = "WAIT_FOR_REPLAN"


@dataclass(frozen=True)
class RecoveryConfig:
    backup_time_s: float = 0.8
    backup_speed_mps: float = 0.20
    rotate_time_s: float = 1.2
    rotate_speed_radps: float = 0.85
    replan_wait_time_s: float = 1.0
    max_attempts: int = 2


@dataclass(frozen=True)
class RecoveryCommand:
    linear_x: float
    angular_z: float
    phase: RecoveryPhase
    clear_costmap: bool = False
    finished: bool = False


class RecoveryBehavior:
    def __init__(self, config: RecoveryConfig | None = None):
        self.config = config or RecoveryConfig()
        self.phase = RecoveryPhase.IDLE
        self.phase_start_s = 0.0
        self.attempts = 0
        self.rotate_direction = 1.0
        self._clear_sent = False

    @property
    def active(self) -> bool:
        return self.phase != RecoveryPhase.IDLE

    def reset(self) -> None:
        self.phase = RecoveryPhase.IDLE
        self.phase_start_s = 0.0
        self.attempts = 0
        self.rotate_direction = 1.0
        self._clear_sent = False

    def can_start(self) -> bool:
        return not self.active and self.attempts < max(0, self.config.max_attempts)

    def start(self, now_s: float) -> bool:
        if not self.can_start():
            return False

        self.attempts += 1
        self.rotate_direction = 1.0 if self.attempts % 2 == 1 else -1.0
        self.phase = RecoveryPhase.BACK_UP
        self.phase_start_s = now_s
        self._clear_sent = False
        return True

    def update(self, now_s: float) -> RecoveryCommand:
        if self.phase == RecoveryPhase.IDLE:
            return RecoveryCommand(
                linear_x=0.0,
                angular_z=0.0,
                phase=RecoveryPhase.IDLE,
            )

        elapsed_s = max(0.0, now_s - self.phase_start_s)

        if self.phase == RecoveryPhase.BACK_UP:
            if elapsed_s >= max(0.0, self.config.backup_time_s):
                self._advance(RecoveryPhase.ROTATE, now_s)
                return self.update(now_s)
            return RecoveryCommand(
                linear_x=-abs(self.config.backup_speed_mps),
                angular_z=0.0,
                phase=self.phase,
            )

        if self.phase == RecoveryPhase.ROTATE:
            if elapsed_s >= max(0.0, self.config.rotate_time_s):
                self._advance(RecoveryPhase.CLEAR_COSTMAP, now_s)
                return self.update(now_s)
            return RecoveryCommand(
                linear_x=0.0,
                angular_z=self.rotate_direction * abs(self.config.rotate_speed_radps),
                phase=self.phase,
            )

        if self.phase == RecoveryPhase.CLEAR_COSTMAP:
            if not self._clear_sent:
                self._clear_sent = True
                return RecoveryCommand(
                    linear_x=0.0,
                    angular_z=0.0,
                    phase=self.phase,
                    clear_costmap=True,
                )
            self._advance(RecoveryPhase.WAIT_FOR_REPLAN, now_s)
            return self.update(now_s)

        if self.phase == RecoveryPhase.WAIT_FOR_REPLAN:
            if elapsed_s >= max(0.0, self.config.replan_wait_time_s):
                self.phase = RecoveryPhase.IDLE
                return RecoveryCommand(
                    linear_x=0.0,
                    angular_z=0.0,
                    phase=RecoveryPhase.IDLE,
                    finished=True,
                )
            return RecoveryCommand(
                linear_x=0.0,
                angular_z=0.0,
                phase=self.phase,
            )

        return RecoveryCommand(
            linear_x=0.0,
            angular_z=0.0,
            phase=RecoveryPhase.IDLE,
            finished=True,
        )

    def _advance(self, phase: RecoveryPhase, now_s: float) -> None:
        self.phase = phase
        self.phase_start_s = now_s
