import pytest

from vision_guided_robot.grid_planner import (
    GridBounds,
    RectangleObstacle,
    build_point_obstacle_cells,
    parse_obstacle_rectangles,
    path_cells_intersect_blocked,
    plan_grid_path,
    point_in_inflated_obstacle,
    world_to_grid,
)


def test_world_to_grid_converts_odom_points():
    bounds = GridBounds(
        origin_x=-1.0,
        origin_y=-1.0,
        width_m=4.0,
        height_m=4.0,
        resolution_m=0.5,
    )

    assert world_to_grid((0.0, 0.0), bounds) == (2, 2)
    assert world_to_grid((-2.0, 0.0), bounds) is None


def test_parse_obstacle_rectangles():
    obstacles = parse_obstacle_rectangles("1.2,0.4,0.10,0.80;2.0 -0.5 0.2 0.3")

    assert obstacles == [
        RectangleObstacle(center_x=1.2, center_y=0.4, size_x=0.10, size_y=0.80),
        RectangleObstacle(center_x=2.0, center_y=-0.5, size_x=0.2, size_y=0.3),
    ]


def test_parse_obstacle_rectangles_rejects_bad_entries():
    with pytest.raises(ValueError):
        parse_obstacle_rectangles("1.0,2.0,3.0")


def test_inflated_obstacle_contains_clearance_margin():
    obstacle = RectangleObstacle(center_x=1.0, center_y=0.0, size_x=0.2, size_y=0.4)

    assert point_in_inflated_obstacle(1.0, 0.29, obstacle, inflation_radius_m=0.10)
    assert not point_in_inflated_obstacle(1.0, 0.35, obstacle, inflation_radius_m=0.10)


def test_plans_direct_path_without_obstacles():
    bounds = GridBounds(-1.0, -1.0, 4.0, 3.0, 0.10)

    plan = plan_grid_path(
        start=(0.0, 0.0),
        goal=(2.0, 0.0),
        bounds=bounds,
        obstacles=[],
        inflation_radius_m=0.20,
    )

    assert plan is not None
    assert plan.points[0] == pytest.approx((0.0, 0.0))
    assert plan.points[-1] == pytest.approx((2.0, 0.0))
    assert len(plan.points) == 2


def test_plans_around_inflated_wall():
    bounds = GridBounds(-1.0, -2.0, 5.0, 4.0, 0.10)
    wall = RectangleObstacle(center_x=1.0, center_y=0.0, size_x=0.10, size_y=1.2)

    plan = plan_grid_path(
        start=(0.0, 0.0),
        goal=(2.0, 0.0),
        bounds=bounds,
        obstacles=[wall],
        inflation_radius_m=0.20,
    )

    assert plan is not None
    assert len(plan.points) > 2
    assert all(
        not point_in_inflated_obstacle(x, y, wall, inflation_radius_m=0.20)
        for x, y in plan.points
    )


def test_point_obstacles_create_inflated_blocked_cells():
    bounds = GridBounds(-1.0, -1.0, 4.0, 3.0, 0.10)

    blocked = build_point_obstacle_cells(
        bounds,
        points=[(1.0, 0.0)],
        inflation_radius_m=0.20,
    )

    assert world_to_grid((1.0, 0.0), bounds) in blocked
    assert world_to_grid((1.15, 0.0), bounds) in blocked
    assert world_to_grid((1.5, 0.0), bounds) not in blocked


def test_plans_around_scan_derived_blocked_cells():
    bounds = GridBounds(-1.0, -2.0, 5.0, 4.0, 0.10)
    scan_points = [(1.0, y * 0.1) for y in range(-6, 7)]
    blocked = build_point_obstacle_cells(bounds, scan_points, inflation_radius_m=0.20)

    plan = plan_grid_path(
        start=(0.0, 0.0),
        goal=(2.0, 0.0),
        bounds=bounds,
        obstacles=[],
        inflation_radius_m=0.20,
        additional_blocked_cells=blocked,
    )

    assert plan is not None
    assert len(plan.points) > 2


def test_path_cells_intersect_blocked_detects_blocked_segment():
    path = [(0, 0), (5, 0)]

    assert path_cells_intersect_blocked(path, {(3, 0)})
    assert not path_cells_intersect_blocked(path, {(3, 1)})


def test_returns_none_when_goal_is_inside_obstacle():
    bounds = GridBounds(-1.0, -1.0, 4.0, 3.0, 0.10)
    obstacle = RectangleObstacle(center_x=2.0, center_y=0.0, size_x=0.6, size_y=0.6)

    plan = plan_grid_path(
        start=(0.0, 0.0),
        goal=(2.0, 0.0),
        bounds=bounds,
        obstacles=[obstacle],
        inflation_radius_m=0.20,
    )

    assert plan is None
