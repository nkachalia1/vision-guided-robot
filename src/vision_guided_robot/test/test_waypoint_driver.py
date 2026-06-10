import math

import pytest

from vision_guided_robot.waypoint_driver import (
    Pose2D,
    WaypointConfig,
    WaypointGoal,
    WaypointState,
    compute_waypoint_command,
    normalize_angle,
    parse_waypoints_text,
)


def test_waits_for_goal_before_commanding_motion():
    pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

    output = compute_waypoint_command(pose, None)

    assert output.state == WaypointState.WAITING_FOR_GOAL
    assert output.linear_x == 0.0
    assert output.angular_z == 0.0


def test_waits_for_odom_before_commanding_motion():
    goal = WaypointGoal(x=1.0, y=0.0)

    output = compute_waypoint_command(None, goal)

    assert output.state == WaypointState.WAITING_FOR_ODOM
    assert output.linear_x == 0.0
    assert output.angular_z == 0.0


def test_rotates_toward_off_axis_goal_before_driving():
    pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    goal = WaypointGoal(x=0.0, y=1.0)
    config = WaypointConfig(heading_tolerance_rad=0.1)

    output = compute_waypoint_command(pose, goal, config)

    assert output.state == WaypointState.ROTATE_TO_GOAL
    assert output.linear_x == 0.0
    assert output.angular_z > 0.0


def test_drives_forward_when_aligned_to_goal():
    pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    goal = WaypointGoal(x=1.0, y=0.0)

    output = compute_waypoint_command(pose, goal)

    assert output.state == WaypointState.DRIVE_TO_GOAL
    assert output.linear_x > 0.0
    assert output.angular_z == pytest.approx(0.0)


def test_stops_when_goal_is_reached():
    pose = Pose2D(x=0.96, y=0.0, yaw=0.0)
    goal = WaypointGoal(x=1.0, y=0.0)
    config = WaypointConfig(goal_tolerance_m=0.10)

    output = compute_waypoint_command(pose, goal, config)

    assert output.state == WaypointState.DONE
    assert output.linear_x == 0.0
    assert output.angular_z == 0.0


def test_rotates_to_final_yaw_after_reaching_position():
    pose = Pose2D(x=1.0, y=0.0, yaw=0.0)
    goal = WaypointGoal(x=1.0, y=0.0, yaw=math.pi / 2.0, use_final_yaw=True)

    output = compute_waypoint_command(pose, goal)

    assert output.state == WaypointState.ROTATE_TO_FINAL
    assert output.linear_x == 0.0
    assert output.angular_z > 0.0


def test_angle_normalization_wraps_to_shortest_turn():
    assert normalize_angle(3.5) == pytest.approx(-2.7831853071795867)


def test_parse_waypoints_text_accepts_xy_and_xy_yaw_entries():
    waypoints = parse_waypoints_text("1.0,0.0; 1.5,0.8,1.57")

    assert len(waypoints) == 2
    assert waypoints[0].x == pytest.approx(1.0)
    assert waypoints[0].y == pytest.approx(0.0)
    assert not waypoints[0].use_final_yaw
    assert waypoints[1].x == pytest.approx(1.5)
    assert waypoints[1].y == pytest.approx(0.8)
    assert waypoints[1].yaw == pytest.approx(1.57)
    assert waypoints[1].use_final_yaw


def test_parse_waypoints_text_rejects_malformed_entry():
    with pytest.raises(ValueError):
        parse_waypoints_text("1.0")
