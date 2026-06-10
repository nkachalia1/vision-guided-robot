import pytest

from vision_guided_robot.path_follower import (
    PathFollowerConfig,
    compute_path_following_command,
    select_lookahead_target,
)
from vision_guided_robot.waypoint_driver import Pose2D, WaypointGoal, WaypointState


def test_selects_lookahead_point_along_straight_path():
    pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    path = [
        WaypointGoal(0.0, 0.0),
        WaypointGoal(1.0, 0.0),
        WaypointGoal(2.0, 0.0),
    ]

    target = select_lookahead_target(pose, path, lookahead_distance_m=0.5)

    assert target.x == pytest.approx(0.5)
    assert target.y == pytest.approx(0.0)


def test_selects_lookahead_point_after_nearest_segment_projection():
    pose = Pose2D(x=1.0, y=0.2, yaw=0.0)
    path = [
        WaypointGoal(0.0, 0.0),
        WaypointGoal(1.0, 0.0),
        WaypointGoal(1.0, 1.0),
    ]

    target = select_lookahead_target(pose, path, lookahead_distance_m=0.4)

    assert target.x == pytest.approx(1.0)
    assert target.y == pytest.approx(0.6)


def test_path_follower_drives_and_steers_toward_lookahead_target():
    pose = Pose2D(x=0.0, y=0.2, yaw=0.0)
    path = [
        WaypointGoal(0.0, 0.0),
        WaypointGoal(1.0, 0.0),
        WaypointGoal(2.0, 0.0),
    ]

    output = compute_path_following_command(pose, path)

    assert output.state == WaypointState.FOLLOW_PATH
    assert output.linear_x > 0.0
    assert output.angular_z < 0.0


def test_path_follower_stops_at_final_path_point():
    pose = Pose2D(x=1.96, y=0.0, yaw=0.0)
    path = [
        WaypointGoal(0.0, 0.0),
        WaypointGoal(1.0, 0.0),
        WaypointGoal(2.0, 0.0),
    ]
    config = PathFollowerConfig(goal_tolerance_m=0.10)

    output = compute_path_following_command(pose, path, config)

    assert output.state == WaypointState.DONE
    assert output.linear_x == 0.0
    assert output.angular_z == 0.0
