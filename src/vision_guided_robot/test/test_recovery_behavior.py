from vision_guided_robot.recovery_behavior import (
    RecoveryBehavior,
    RecoveryConfig,
    RecoveryPhase,
)


def test_recovery_runs_backup_rotate_clear_wait_sequence():
    recovery = RecoveryBehavior(
        RecoveryConfig(
            backup_time_s=1.0,
            backup_speed_mps=0.2,
            rotate_time_s=1.0,
            rotate_speed_radps=0.8,
            replan_wait_time_s=1.0,
            max_attempts=1,
        )
    )

    assert recovery.start(now_s=10.0)

    backup = recovery.update(now_s=10.5)
    assert backup.phase == RecoveryPhase.BACK_UP
    assert backup.linear_x == -0.2
    assert backup.angular_z == 0.0

    rotate = recovery.update(now_s=11.1)
    assert rotate.phase == RecoveryPhase.ROTATE
    assert rotate.linear_x == 0.0
    assert rotate.angular_z == 0.8

    clear = recovery.update(now_s=12.2)
    assert clear.phase == RecoveryPhase.CLEAR_COSTMAP
    assert clear.clear_costmap

    wait = recovery.update(now_s=12.3)
    assert wait.phase == RecoveryPhase.WAIT_FOR_REPLAN
    assert not wait.finished

    finished = recovery.update(now_s=13.4)
    assert finished.finished
    assert not recovery.active


def test_recovery_attempt_limit_prevents_restarting():
    recovery = RecoveryBehavior(RecoveryConfig(max_attempts=1))

    assert recovery.start(now_s=0.0)
    recovery.phase = RecoveryPhase.IDLE

    assert not recovery.can_start()
    assert not recovery.start(now_s=1.0)


def test_recovery_alternates_rotate_direction_between_attempts():
    recovery = RecoveryBehavior(RecoveryConfig(max_attempts=2))

    assert recovery.start(now_s=0.0)
    assert recovery.rotate_direction == 1.0
    recovery.phase = RecoveryPhase.IDLE

    assert recovery.start(now_s=1.0)
    assert recovery.rotate_direction == -1.0
