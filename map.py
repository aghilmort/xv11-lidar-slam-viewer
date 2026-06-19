"""Occupancy grid helpers for BreezySLAM maps."""

from __future__ import annotations

import numpy as np


class OccupancyGrid:
    """Convert BreezySLAM map bytes into display-friendly arrays."""

    def __init__(self, size_pixels: int, size_meters: float) -> None:
        self.size_pixels = size_pixels
        self.size_meters = size_meters

    @property
    def meters_per_pixel(self) -> float:
        return self.size_meters / self.size_pixels

    def from_bytes(self, map_bytes: bytes) -> np.ndarray:
        grid = np.frombuffer(map_bytes, dtype=np.uint8)
        return grid.reshape((self.size_pixels, self.size_pixels))

    def pose_to_pixels(self, x_mm: float, y_mm: float) -> tuple[float, float]:
        scale = self.size_pixels / (self.size_meters * 1000.0)
        return x_mm * scale, y_mm * scale
