from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math


class MissionState(str, Enum):
    WAITING_FOR_GOAL = "WAITING_FOR_GOAL"
    NAVIGATING = "NAVIGATING"
    REROUTING = "REROUTING"
    RECOVERING = "RECOVERING"
    PAUSED_FOR_SAFETY = "PAUSED_FOR_SAFETY"
    BLOCKED = "BLOCKED"
    DONE = "DONE"


SAFETY_PAUSE_STATES = {"AVOID", "BLOCKED", "STALE_SCAN"}


def choose_mission_state(
    has_goal: bool,
    waypoint_state: str,
    safety_state: str,
    safety_pause_duration_s: float,
    blocked_timeout_s: float,
    stuck_blocked: bool = False,
    safety_oscillating: bool = False,
    rerouting: bool = False,
    recovering: bool = False,
) -> MissionState:
    if recovering:
        return MissionState.RECOVERING

    if not has_goal:
        return MissionState.WAITING_FOR_GOAL

    if stuck_blocked or safety_oscillating:
        return MissionState.BLOCKED

    if safety_state in SAFETY_PAUSE_STATES:
        if safety_pause_duration_s >= blocked_timeout_s:
            return MissionState.BLOCKED
        return MissionState.PAUSED_FOR_SAFETY

    if rerouting:
        return MissionState.REROUTING

    if waypoint_state == "DONE":
        return MissionState.DONE

    return MissionState.NAVIGATING


@dataclass
class MissionProgressWatchdog:
    min_progress_m: float = 0.10
    stuck_timeout_s: float = 10.0
    best_distance_m: float = math.inf
    last_progress_time_s: float | None = None
    safety_interrupted_since_progress: bool = False
    blocked: bool = False
    blocking_since_s: float | None = field(default=None, init=False)

    def reset(self) -> None:
        self.best_distance_m = math.inf
        self.last_progress_time_s = None
        self.safety_interrupted_since_progress = False
        self.blocked = False
        self.blocking_since_s = None

    def update(
        self,
        now_s: float,
        has_goal: bool,
        waypoint_state: str,
        safety_state: str,
        distance_to_goal_m: float,
    ) -> bool:
        if (
            not has_goal
            or waypoint_state in {"WAITING_FOR_GOAL", "WAITING_FOR_ODOM", "DONE"}
            or not math.isfinite(distance_to_goal_m)
        ):
            self.reset()
            return False

        if self.blocked:
            return True

        if self.last_progress_time_s is None:
            self.best_distance_m = distance_to_goal_m
            self.last_progress_time_s = now_s
            return False

        if distance_to_goal_m <= self.best_distance_m - self.min_progress_m:
            self.best_distance_m = distance_to_goal_m
            self.last_progress_time_s = now_s
            self.safety_interrupted_since_progress = False

        if safety_state in SAFETY_PAUSE_STATES:
            self.safety_interrupted_since_progress = True

        if (
            self.safety_interrupted_since_progress
            and now_s - self.last_progress_time_s >= self.stuck_timeout_s
        ):
            self.blocked = True
            self.blocking_since_s = now_s

        return self.blocked


@dataclass
class SafetyOscillationWatchdog:
    max_interruptions: int = 2
    window_s: float = 8.0
    first_interrupt_time_s: float | None = None
    interruption_count: int = 0
    previous_paused: bool = False
    triggered: bool = False
    triggering_since_s: float | None = field(default=None, init=False)

    def reset(self) -> None:
        self.first_interrupt_time_s = None
        self.interruption_count = 0
        self.previous_paused = False
        self.triggered = False
        self.triggering_since_s = None

    def update(
        self,
        now_s: float,
        has_goal: bool,
        waypoint_state: str,
        safety_state: str,
    ) -> bool:
        if not has_goal or waypoint_state in {"WAITING_FOR_GOAL", "WAITING_FOR_ODOM", "DONE"}:
            self.reset()
            return False

        if self.triggered:
            return True

        paused = safety_state in SAFETY_PAUSE_STATES
        if paused and not self.previous_paused:
            self._record_interruption(now_s)

        self.previous_paused = paused

        if (
            self.first_interrupt_time_s is not None
            and now_s - self.first_interrupt_time_s > self.window_s
        ):
            self.first_interrupt_time_s = None
            self.interruption_count = 0

        return self.triggered

    def _record_interruption(self, now_s: float) -> None:
        if (
            self.first_interrupt_time_s is None
            or now_s - self.first_interrupt_time_s > self.window_s
        ):
            self.first_interrupt_time_s = now_s
            self.interruption_count = 1
        else:
            self.interruption_count += 1

        if self.interruption_count >= self.max_interruptions:
            self.triggered = True
            self.triggering_since_s = now_s
