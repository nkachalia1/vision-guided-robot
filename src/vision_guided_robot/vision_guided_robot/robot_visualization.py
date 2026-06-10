from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FootprintConfig:
    length_m: float = 0.34
    width_m: float = 0.24


def footprint_corners(config: FootprintConfig | None = None) -> list[tuple[float, float]]:
    config = config or FootprintConfig()
    half_length = config.length_m / 2.0
    half_width = config.width_m / 2.0
    return [
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width),
        (half_length, half_width),
    ]
