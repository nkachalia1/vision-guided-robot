import pytest

from vision_guided_robot.detour_planner import DetourConfig, plan_detour
from vision_guided_robot.waypoint_driver import Pose2D, WaypointGoal


def test_plan_detour_offsets_left_of_path():
    pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    resume_goal = WaypointGoal(x=2.0, y=0.0)

    plan = plan_detour(
        pose,
        resume_goal,
        side_sign=1.0,
        config=DetourConfig(forward_offset_m=0.5, lateral_offset_m=1.0),
    )

    assert plan is not None
    assert plan.goal.x == pytest.approx(0.5)
    assert plan.goal.y == pytest.approx(1.0)
    assert plan.resume_goal == resume_goal
    assert plan.side_sign == pytest.approx(1.0)


def test_plan_detour_offsets_right_of_path():
    pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    resume_goal = WaypointGoal(x=2.0, y=0.0)

    plan = plan_detour(
        pose,
        resume_goal,
        side_sign=-1.0,
        config=DetourConfig(forward_offset_m=0.5, lateral_offset_m=1.0),
    )

    assert plan is not None
    assert plan.goal.x == pytest.approx(0.5)
    assert plan.goal.y == pytest.approx(-1.0)
    assert plan.side_sign == pytest.approx(-1.0)


def test_plan_detour_uses_robot_heading_when_goal_is_too_close():
    pose = Pose2D(x=1.0, y=1.0, yaw=0.0)
    resume_goal = WaypointGoal(x=1.05, y=1.0)

    plan = plan_detour(
        pose,
        resume_goal,
        side_sign=1.0,
        config=DetourConfig(
            forward_offset_m=0.5,
            lateral_offset_m=1.0,
            near_goal_heading_distance_m=0.25,
        ),
    )

    assert plan is not None
    assert plan.goal.x == pytest.approx(1.5)
    assert plan.goal.y == pytest.approx(2.0)


def test_plan_detour_returns_none_without_pose_or_goal():
    assert plan_detour(None, WaypointGoal(x=1.0, y=0.0), side_sign=1.0) is None
    assert plan_detour(Pose2D(x=0.0, y=0.0, yaw=0.0), None, side_sign=1.0) is None
