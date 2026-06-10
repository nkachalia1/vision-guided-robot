from __future__ import annotations

from dataclasses import dataclass, field

from vision_guided_robot.grid_planner import GridCell


@dataclass
class PersistentCostmap:
    memory_time_s: float = 12.0
    last_seen_by_cell: dict[GridCell, float] = field(default_factory=dict)

    def update(self, observed_cells: set[GridCell], now_s: float) -> None:
        for cell in observed_cells:
            self.last_seen_by_cell[cell] = now_s
        self.prune(now_s)

    def prune(self, now_s: float) -> None:
        memory_time_s = max(0.0, self.memory_time_s)
        expired = [
            cell
            for cell, last_seen_s in self.last_seen_by_cell.items()
            if now_s - last_seen_s > memory_time_s
        ]
        for cell in expired:
            del self.last_seen_by_cell[cell]

    def active_cells(self, now_s: float) -> set[GridCell]:
        self.prune(now_s)
        return set(self.last_seen_by_cell)

    def signature(self, now_s: float) -> tuple[GridCell, ...]:
        return tuple(sorted(self.active_cells(now_s)))

    def clear(self) -> None:
        self.last_seen_by_cell.clear()
