import math

import pytest

from vision_guided_robot.target_search import (
    RobotPose,
    TargetEstimate,
    compute_approach_goal,
)


def test_computes_stand_off_goal_for_target_straight_ahead():
    goal = compute_approach_goal(
        RobotPose(x_m=0.0, y_m=0.0, yaw_rad=0.0),
        TargetEstimate(lateral_m=0.0, distance_m=2.0),
        camera_forward_offset_m=0.2,
        stand_off_distance_m=0.5,
    )

    assert goal.x_m == pytest.approx(1.7)
    assert goal.y_m == pytest.approx(0.0)
    assert goal.yaw_rad == pytest.approx(0.0)
    assert goal.target_range_m == pytest.approx(2.2)


def test_camera_right_maps_to_negative_base_y():
    goal = compute_approach_goal(
        RobotPose(x_m=0.0, y_m=0.0, yaw_rad=0.0),
        TargetEstimate(lateral_m=0.4, distance_m=2.0),
        camera_forward_offset_m=0.0,
        stand_off_distance_m=0.5,
    )

    assert goal.x_m > 0.0
    assert goal.y_m < 0.0
    assert goal.yaw_rad < 0.0


def test_goal_rotates_with_robot_pose_in_map():
    goal = compute_approach_goal(
        RobotPose(x_m=1.0, y_m=2.0, yaw_rad=math.pi / 2.0),
        TargetEstimate(lateral_m=0.0, distance_m=1.5),
        camera_forward_offset_m=0.0,
        stand_off_distance_m=0.5,
    )

    assert goal.x_m == pytest.approx(1.0)
    assert goal.y_m == pytest.approx(3.0)
    assert goal.yaw_rad == pytest.approx(math.pi / 2.0)
