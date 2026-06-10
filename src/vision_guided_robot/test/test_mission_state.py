from vision_guided_robot.mission_state import (
    MissionProgressWatchdog,
    MissionState,
    SafetyOscillationWatchdog,
    choose_mission_state,
)


def test_waits_when_there_is_no_goal():
    state = choose_mission_state(
        has_goal=False,
        waypoint_state="WAITING_FOR_GOAL",
        safety_state="CLEAR",
        safety_pause_duration_s=0.0,
        blocked_timeout_s=8.0,
    )

    assert state == MissionState.WAITING_FOR_GOAL


def test_navigates_when_goal_exists_and_safety_is_clear():
    state = choose_mission_state(
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
        safety_pause_duration_s=0.0,
        blocked_timeout_s=8.0,
    )

    assert state == MissionState.NAVIGATING


def test_reports_done_when_waypoint_is_done_and_safety_is_clear():
    state = choose_mission_state(
        has_goal=True,
        waypoint_state="DONE",
        safety_state="CLEAR",
        safety_pause_duration_s=0.0,
        blocked_timeout_s=8.0,
    )

    assert state == MissionState.DONE


def test_reports_rerouting_when_following_a_detour():
    state = choose_mission_state(
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
        safety_pause_duration_s=0.0,
        blocked_timeout_s=8.0,
        rerouting=True,
    )

    assert state == MissionState.REROUTING


def test_reports_recovering_when_recovery_is_active():
    state = choose_mission_state(
        has_goal=False,
        waypoint_state="RECOVERING",
        safety_state="CLEAR",
        safety_pause_duration_s=0.0,
        blocked_timeout_s=8.0,
        recovering=True,
    )

    assert state == MissionState.RECOVERING


def test_pauses_when_reactive_safety_is_avoiding():
    state = choose_mission_state(
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="AVOID",
        safety_pause_duration_s=2.0,
        blocked_timeout_s=8.0,
    )

    assert state == MissionState.PAUSED_FOR_SAFETY


def test_reports_blocked_after_safety_pause_timeout():
    state = choose_mission_state(
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="AVOID",
        safety_pause_duration_s=8.0,
        blocked_timeout_s=8.0,
    )

    assert state == MissionState.BLOCKED


def test_reports_blocked_when_progress_watchdog_is_blocked():
    state = choose_mission_state(
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
        safety_pause_duration_s=0.0,
        blocked_timeout_s=8.0,
        stuck_blocked=True,
    )

    assert state == MissionState.BLOCKED


def test_reports_blocked_when_safety_is_oscillating():
    state = choose_mission_state(
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
        safety_pause_duration_s=0.0,
        blocked_timeout_s=8.0,
        safety_oscillating=True,
    )

    assert state == MissionState.BLOCKED


def test_progress_watchdog_blocks_after_safety_interrupts_without_progress():
    watchdog = MissionProgressWatchdog(min_progress_m=0.10, stuck_timeout_s=5.0)

    assert not watchdog.update(
        now_s=0.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
        distance_to_goal_m=2.0,
    )
    assert not watchdog.update(
        now_s=1.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="AVOID",
        distance_to_goal_m=1.98,
    )
    assert watchdog.update(
        now_s=5.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
        distance_to_goal_m=1.97,
    )


def test_progress_watchdog_resets_when_distance_improves():
    watchdog = MissionProgressWatchdog(min_progress_m=0.10, stuck_timeout_s=5.0)

    watchdog.update(
        now_s=0.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
        distance_to_goal_m=2.0,
    )
    watchdog.update(
        now_s=1.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="AVOID",
        distance_to_goal_m=1.98,
    )
    assert not watchdog.update(
        now_s=4.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
        distance_to_goal_m=1.85,
    )
    assert not watchdog.update(
        now_s=8.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
        distance_to_goal_m=1.84,
    )


def test_safety_oscillation_watchdog_triggers_after_repeated_interruptions():
    watchdog = SafetyOscillationWatchdog(max_interruptions=2, window_s=8.0)

    assert not watchdog.update(
        now_s=0.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="AVOID",
    )
    assert not watchdog.update(
        now_s=1.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
    )
    assert watchdog.update(
        now_s=4.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="AVOID",
    )


def test_safety_oscillation_watchdog_resets_after_window():
    watchdog = SafetyOscillationWatchdog(max_interruptions=2, window_s=3.0)

    assert not watchdog.update(
        now_s=0.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="AVOID",
    )
    assert not watchdog.update(
        now_s=1.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="CLEAR",
    )
    assert not watchdog.update(
        now_s=5.0,
        has_goal=True,
        waypoint_state="DRIVE_TO_GOAL",
        safety_state="AVOID",
    )
