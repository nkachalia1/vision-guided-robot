import pytest

from vision_guided_robot.frontier_exploration import (
    FrontierConfig,
    GridSpec,
    cluster_frontiers,
    find_frontier_cells,
    select_frontier_goal,
)


def make_partly_known_map(width=8, height=6):
    data = [-1] * (width * height)
    for y in range(1, 5):
        for x in range(1, 4):
            data[y * width + x] = 0
    return data


def test_find_frontier_cells_where_free_space_touches_unknown():
    spec = GridSpec(width=8, height=6, resolution_m=0.5, origin_x_m=0.0, origin_y_m=0.0)
    data = make_partly_known_map(spec.width, spec.height)

    frontiers = find_frontier_cells(
        data,
        spec,
        FrontierConfig(obstacle_clearance_m=0.0),
    )

    assert (3, 2) in frontiers
    assert (2, 2) not in frontiers


def test_cluster_frontiers_groups_touching_cells():
    clusters = cluster_frontiers({(1, 1), (1, 2), (5, 5)})

    assert sorted(len(cluster) for cluster in clusters) == [1, 2]


def test_select_frontier_goal_returns_world_pose_for_best_cluster():
    spec = GridSpec(width=8, height=6, resolution_m=0.5, origin_x_m=0.0, origin_y_m=0.0)
    data = make_partly_known_map(spec.width, spec.height)

    goal = select_frontier_goal(
        data,
        spec,
        robot_xy_m=(0.75, 1.25),
        config=FrontierConfig(
            min_cluster_size=2,
            obstacle_clearance_m=0.0,
            min_goal_distance_m=0.1,
            max_goal_distance_m=5.0,
        ),
    )

    assert goal is not None
    assert goal.x_m == pytest.approx(1.75, abs=0.5)
    assert goal.y_m == pytest.approx(1.50, abs=1.0)
    assert goal.cluster_size >= 2
    assert goal.information_gain > 0


def test_obstacle_clearance_rejects_unsafe_frontier_cells():
    spec = GridSpec(width=8, height=6, resolution_m=0.5, origin_x_m=0.0, origin_y_m=0.0)
    data = make_partly_known_map(spec.width, spec.height)
    data[2 * spec.width + 4] = 100

    frontiers = find_frontier_cells(
        data,
        spec,
        FrontierConfig(obstacle_clearance_m=0.6),
    )

    assert (3, 2) not in frontiers
