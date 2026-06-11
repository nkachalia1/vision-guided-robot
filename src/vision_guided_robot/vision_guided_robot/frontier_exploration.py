from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


GridCell = tuple[int, int]


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float


@dataclass(frozen=True)
class FrontierConfig:
    free_cost_max: int = 20
    occupied_cost_min: int = 65
    unknown_value: int = -1
    min_cluster_size: int = 4
    obstacle_clearance_m: float = 0.25
    information_radius_cells: int = 3
    min_goal_distance_m: float = 0.35
    max_goal_distance_m: float = 4.0
    distance_weight: float = 2.0


@dataclass(frozen=True)
class FrontierCandidate:
    x_m: float
    y_m: float
    cell: GridCell
    cluster_size: int
    information_gain: int
    distance_m: float
    score: float


def cell_index(cell: GridCell, width: int) -> int:
    x, y = cell
    return y * width + x


def in_bounds(cell: GridCell, spec: GridSpec) -> bool:
    x, y = cell
    return 0 <= x < spec.width and 0 <= y < spec.height


def cell_to_world(cell: GridCell, spec: GridSpec) -> tuple[float, float]:
    x, y = cell
    return (
        spec.origin_x_m + (x + 0.5) * spec.resolution_m,
        spec.origin_y_m + (y + 0.5) * spec.resolution_m,
    )


def is_free(value: int, config: FrontierConfig) -> bool:
    return 0 <= value <= config.free_cost_max


def is_unknown(value: int, config: FrontierConfig) -> bool:
    return value == config.unknown_value


def is_occupied(value: int, config: FrontierConfig) -> bool:
    return value >= config.occupied_cost_min


def neighbors4(cell: GridCell) -> list[GridCell]:
    x, y = cell
    return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]


def neighbors8(cell: GridCell) -> list[GridCell]:
    x, y = cell
    return [
        (x + dx, y + dy)
        for dy in (-1, 0, 1)
        for dx in (-1, 0, 1)
        if dx != 0 or dy != 0
    ]


def obstacle_clearance_cells(spec: GridSpec, config: FrontierConfig) -> int:
    if config.obstacle_clearance_m <= 0.0:
        return 0
    return int(math.ceil(config.obstacle_clearance_m / spec.resolution_m))


def has_nearby_obstacle(
    data: list[int] | tuple[int, ...],
    cell: GridCell,
    spec: GridSpec,
    config: FrontierConfig,
) -> bool:
    clearance_cells = obstacle_clearance_cells(spec, config)
    if clearance_cells <= 0:
        return False

    cx, cy = cell
    for y in range(cy - clearance_cells, cy + clearance_cells + 1):
        for x in range(cx - clearance_cells, cx + clearance_cells + 1):
            neighbor = (x, y)
            if not in_bounds(neighbor, spec):
                continue
            if is_occupied(data[cell_index(neighbor, spec.width)], config):
                return True
    return False


def find_frontier_cells(
    data: list[int] | tuple[int, ...],
    spec: GridSpec,
    config: FrontierConfig,
) -> set[GridCell]:
    if len(data) != spec.width * spec.height:
        raise ValueError(
            f"occupancy data has {len(data)} cells, expected {spec.width * spec.height}"
        )

    frontier_cells = set()
    for y in range(spec.height):
        for x in range(spec.width):
            cell = (x, y)
            value = data[cell_index(cell, spec.width)]
            if not is_free(value, config):
                continue
            if has_nearby_obstacle(data, cell, spec, config):
                continue
            if any(
                in_bounds(neighbor, spec)
                and is_unknown(data[cell_index(neighbor, spec.width)], config)
                for neighbor in neighbors4(cell)
            ):
                frontier_cells.add(cell)
    return frontier_cells


def cluster_frontiers(frontier_cells: set[GridCell]) -> list[list[GridCell]]:
    remaining = set(frontier_cells)
    clusters: list[list[GridCell]] = []

    while remaining:
        start = remaining.pop()
        cluster = [start]
        queue = deque([start])

        while queue:
            cell = queue.popleft()
            for neighbor in neighbors8(cell):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                cluster.append(neighbor)
                queue.append(neighbor)

        clusters.append(cluster)

    return clusters


def count_unknown_near_cluster(
    data: list[int] | tuple[int, ...],
    cluster: list[GridCell],
    spec: GridSpec,
    config: FrontierConfig,
) -> int:
    radius = max(1, int(config.information_radius_cells))
    unknown_cells = set()

    for cx, cy in cluster:
        for y in range(cy - radius, cy + radius + 1):
            for x in range(cx - radius, cx + radius + 1):
                cell = (x, y)
                if not in_bounds(cell, spec):
                    continue
                if is_unknown(data[cell_index(cell, spec.width)], config):
                    unknown_cells.add(cell)

    return len(unknown_cells)


def choose_cluster_goal_cell(cluster: list[GridCell]) -> GridCell:
    mean_x = sum(cell[0] for cell in cluster) / len(cluster)
    mean_y = sum(cell[1] for cell in cluster) / len(cluster)
    return min(
        cluster,
        key=lambda cell: (cell[0] - mean_x) ** 2 + (cell[1] - mean_y) ** 2,
    )


def frontier_candidates(
    data: list[int] | tuple[int, ...],
    spec: GridSpec,
    robot_xy_m: tuple[float, float],
    config: FrontierConfig,
) -> list[FrontierCandidate]:
    frontiers = find_frontier_cells(data, spec, config)
    clusters = [
        cluster
        for cluster in cluster_frontiers(frontiers)
        if len(cluster) >= config.min_cluster_size
    ]

    candidates = []
    for cluster in clusters:
        goal_cell = choose_cluster_goal_cell(cluster)
        goal_x, goal_y = cell_to_world(goal_cell, spec)
        distance_m = math.hypot(goal_x - robot_xy_m[0], goal_y - robot_xy_m[1])
        if distance_m < config.min_goal_distance_m:
            continue
        if distance_m > config.max_goal_distance_m:
            continue
        information_gain = count_unknown_near_cluster(data, cluster, spec, config)
        score = (
            float(information_gain)
            + 0.25 * float(len(cluster))
            - config.distance_weight * distance_m
        )
        candidates.append(
            FrontierCandidate(
                x_m=goal_x,
                y_m=goal_y,
                cell=goal_cell,
                cluster_size=len(cluster),
                information_gain=information_gain,
                distance_m=distance_m,
                score=score,
            )
        )

    return sorted(candidates, key=lambda candidate: candidate.score, reverse=True)


def select_frontier_goal(
    data: list[int] | tuple[int, ...],
    spec: GridSpec,
    robot_xy_m: tuple[float, float],
    config: FrontierConfig,
) -> FrontierCandidate | None:
    candidates = frontier_candidates(data, spec, robot_xy_m, config)
    return candidates[0] if candidates else None
