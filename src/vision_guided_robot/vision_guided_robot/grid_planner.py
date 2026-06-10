from __future__ import annotations

from dataclasses import dataclass
import heapq
import math


GridCell = tuple[int, int]
WorldPoint = tuple[float, float]


@dataclass(frozen=True)
class GridBounds:
    origin_x: float
    origin_y: float
    width_m: float
    height_m: float
    resolution_m: float

    @property
    def width_cells(self) -> int:
        return max(1, int(math.ceil(self.width_m / self.resolution_m)))

    @property
    def height_cells(self) -> int:
        return max(1, int(math.ceil(self.height_m / self.resolution_m)))


@dataclass(frozen=True)
class RectangleObstacle:
    center_x: float
    center_y: float
    size_x: float
    size_y: float


@dataclass(frozen=True)
class GridPlan:
    points: list[WorldPoint]
    cells: list[GridCell]
    expanded_cells: int


def plan_grid_path(
    start: WorldPoint,
    goal: WorldPoint,
    bounds: GridBounds,
    obstacles: list[RectangleObstacle],
    inflation_radius_m: float = 0.20,
    additional_blocked_cells: set[GridCell] | None = None,
    simplify: bool = True,
) -> GridPlan | None:
    blocked = build_blocked_cells(bounds, obstacles, inflation_radius_m)
    if additional_blocked_cells:
        blocked = set(blocked) | additional_blocked_cells
    start_cell = world_to_grid(start, bounds)
    goal_cell = world_to_grid(goal, bounds)
    if start_cell is None or goal_cell is None:
        return None
    blocked.discard(start_cell)
    if goal_cell in blocked:
        return None

    cell_path, expanded_cells = astar_cells(
        start=start_cell,
        goal=goal_cell,
        bounds=bounds,
        blocked=blocked,
    )
    if not cell_path:
        return None

    if simplify:
        cell_path = simplify_cell_path(cell_path, blocked)

    points = [grid_to_world(cell, bounds) for cell in cell_path]
    points[0] = start
    points[-1] = goal
    return GridPlan(points=points, cells=cell_path, expanded_cells=expanded_cells)


def build_blocked_cells(
    bounds: GridBounds,
    obstacles: list[RectangleObstacle],
    inflation_radius_m: float,
) -> set[GridCell]:
    blocked: set[GridCell] = set()
    inflation = max(0.0, inflation_radius_m)
    for gy in range(bounds.height_cells):
        for gx in range(bounds.width_cells):
            x, y = grid_to_world((gx, gy), bounds)
            if any(point_in_inflated_obstacle(x, y, obstacle, inflation) for obstacle in obstacles):
                blocked.add((gx, gy))
    return blocked


def build_point_obstacle_cells(
    bounds: GridBounds,
    points: list[WorldPoint],
    inflation_radius_m: float,
) -> set[GridCell]:
    blocked: set[GridCell] = set()
    inflation = max(0.0, inflation_radius_m)
    radius_cells = int(math.ceil(inflation / bounds.resolution_m))

    for point in points:
        center_cell = world_to_grid(point, bounds)
        if center_cell is None:
            continue
        cx, cy = center_cell
        for gx in range(cx - radius_cells, cx + radius_cells + 1):
            for gy in range(cy - radius_cells, cy + radius_cells + 1):
                cell = (gx, gy)
                if not cell_in_bounds(cell, bounds):
                    continue
                cell_x, cell_y = grid_to_world(cell, bounds)
                if math.hypot(cell_x - point[0], cell_y - point[1]) <= inflation:
                    blocked.add(cell)

    return blocked


def point_in_inflated_obstacle(
    x: float,
    y: float,
    obstacle: RectangleObstacle,
    inflation_radius_m: float,
) -> bool:
    half_x = obstacle.size_x * 0.5 + inflation_radius_m
    half_y = obstacle.size_y * 0.5 + inflation_radius_m
    return (
        obstacle.center_x - half_x <= x <= obstacle.center_x + half_x
        and obstacle.center_y - half_y <= y <= obstacle.center_y + half_y
    )


def astar_cells(
    start: GridCell,
    goal: GridCell,
    bounds: GridBounds,
    blocked: set[GridCell],
) -> tuple[list[GridCell], int]:
    frontier: list[tuple[float, GridCell]] = [(0.0, start)]
    came_from: dict[GridCell, GridCell | None] = {start: None}
    cost_so_far: dict[GridCell, float] = {start: 0.0}
    expanded_cells = 0

    while frontier:
        _, current = heapq.heappop(frontier)
        expanded_cells += 1

        if current == goal:
            return reconstruct_path(came_from, goal), expanded_cells

        for neighbor, step_cost in neighbors(current, bounds, blocked):
            new_cost = cost_so_far[current] + step_cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + cell_distance(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    return [], expanded_cells


def neighbors(
    cell: GridCell,
    bounds: GridBounds,
    blocked: set[GridCell],
) -> list[tuple[GridCell, float]]:
    result = []
    cx, cy = cell
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            neighbor = (cx + dx, cy + dy)
            if not cell_in_bounds(neighbor, bounds) or neighbor in blocked:
                continue
            result.append((neighbor, math.hypot(dx, dy)))
    return result


def reconstruct_path(
    came_from: dict[GridCell, GridCell | None],
    goal: GridCell,
) -> list[GridCell]:
    path = [goal]
    current = goal
    while came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def simplify_cell_path(path: list[GridCell], blocked: set[GridCell]) -> list[GridCell]:
    if len(path) <= 2:
        return path

    simplified = [path[0]]
    anchor_index = 0
    while anchor_index < len(path) - 1:
        next_index = len(path) - 1
        while next_index > anchor_index + 1:
            if line_is_clear(path[anchor_index], path[next_index], blocked):
                break
            next_index -= 1
        simplified.append(path[next_index])
        anchor_index = next_index

    return simplified


def line_is_clear(start: GridCell, goal: GridCell, blocked: set[GridCell]) -> bool:
    for cell in bresenham_line(start, goal):
        if cell in blocked:
            return False
    return True


def path_cells_intersect_blocked(
    path_cells: list[GridCell],
    blocked: set[GridCell],
) -> bool:
    if len(path_cells) < 2:
        return False

    for start, goal in zip(path_cells, path_cells[1:]):
        for cell in bresenham_line(start, goal):
            if cell in blocked:
                return True
    return False


def bresenham_line(start: GridCell, goal: GridCell) -> list[GridCell]:
    x0, y0 = start
    x1, y1 = goal
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    cells = []

    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        twice_err = 2 * err
        if twice_err >= dy:
            err += dy
            x0 += sx
        if twice_err <= dx:
            err += dx
            y0 += sy


def world_to_grid(point: WorldPoint, bounds: GridBounds) -> GridCell | None:
    x, y = point
    gx = int(math.floor((x - bounds.origin_x) / bounds.resolution_m))
    gy = int(math.floor((y - bounds.origin_y) / bounds.resolution_m))
    cell = (gx, gy)
    return cell if cell_in_bounds(cell, bounds) else None


def grid_to_world(cell: GridCell, bounds: GridBounds) -> WorldPoint:
    gx, gy = cell
    return (
        bounds.origin_x + (gx + 0.5) * bounds.resolution_m,
        bounds.origin_y + (gy + 0.5) * bounds.resolution_m,
    )


def cell_in_bounds(cell: GridCell, bounds: GridBounds) -> bool:
    gx, gy = cell
    return 0 <= gx < bounds.width_cells and 0 <= gy < bounds.height_cells


def cell_distance(a: GridCell, b: GridCell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def parse_obstacle_rectangles(text: str) -> list[RectangleObstacle]:
    obstacles = []
    if not text.strip():
        return obstacles

    for index, raw_entry in enumerate(text.split(";"), start=1):
        entry = raw_entry.strip()
        if not entry:
            continue
        parts = entry.replace(",", " ").split()
        if len(parts) != 4:
            raise ValueError(
                f"obstacle {index} must contain center_x,center_y,size_x,size_y; "
                f"got '{raw_entry}'"
            )
        try:
            center_x, center_y, size_x, size_y = (float(value) for value in parts)
        except ValueError as exc:
            raise ValueError(f"obstacle {index} contains a non-numeric value") from exc
        obstacles.append(
            RectangleObstacle(
                center_x=center_x,
                center_y=center_y,
                size_x=max(0.0, size_x),
                size_y=max(0.0, size_y),
            )
        )

    return obstacles
