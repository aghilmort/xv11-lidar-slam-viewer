"""Matplotlib visualization for XV11 point clouds and SLAM maps."""

from __future__ import annotations

import math
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np

from lidar import LidarScan, LidarStats
from map import OccupancyGrid
from slam import SlamResult


def scan_to_xy(scan: LidarScan) -> tuple[np.ndarray, np.ndarray]:
    angles = np.deg2rad(np.arange(len(scan.distances_mm)))
    distances_m = np.asarray(scan.distances_mm, dtype=float) / 1000.0
    valid = np.asarray(scan.valid, dtype=bool)
    return distances_m[valid] * np.cos(angles[valid]), distances_m[valid] * np.sin(angles[valid])


class Viewer:
    """Interactive Matplotlib window for live LIDAR and optional SLAM state."""

    def __init__(
        self,
        mode: str,
        max_range_m: float,
        occupancy_grid: Optional[OccupancyGrid] = None,
    ) -> None:
        self.mode = mode
        self.max_range_m = max_range_m
        self.occupancy_grid = occupancy_grid
        self._closed = False

        if mode == "slam":
            self.figure, (self.cloud_axis, self.map_axis) = plt.subplots(1, 2, figsize=(12, 6))
            self.map_image = None
            self.pose_artist = None
        else:
            self.figure, self.cloud_axis = plt.subplots(1, 1, figsize=(7, 7))
            self.map_axis = None
            self.map_image = None
            self.pose_artist = None

        self.figure.canvas.mpl_connect("close_event", self._on_close)
        self.scatter = self.cloud_axis.scatter([], [], s=2)
        self._setup_cloud_axis()
        self.set_status("Waiting for LIDAR data...")

        if self.map_axis is not None:
            self._setup_map_axis()

        plt.ion()
        plt.show(block=False)
        self.idle()

    @property
    def closed(self) -> bool:
        return self._closed or not plt.fignum_exists(self.figure.number)

    def update(self, scan: LidarScan, slam_result: Optional[SlamResult] = None) -> None:
        x, y = scan_to_xy(scan)
        self.scatter.set_offsets(np.column_stack((x, y)) if len(x) else np.empty((0, 2)))
        valid_count = int(np.count_nonzero(scan.valid))
        self.cloud_axis.set_title(f"XV11 point cloud | {valid_count} points | {scan.rpm:.1f} RPM")

        if slam_result is not None and self.map_axis is not None and self.occupancy_grid is not None:
            self._update_map(slam_result)

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        plt.pause(0.001)

    def idle(self, stats: Optional[LidarStats] = None) -> bool:
        if stats is not None and stats.scans == 0:
            self.set_status(
                "Waiting for LIDAR data | "
                f"packets={stats.packets} "
                f"timeouts={stats.timeouts} "
                f"bad_checksum={stats.bad_checksum}"
            )
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        plt.pause(0.01)
        return not self.closed

    def set_status(self, message: str) -> None:
        self.cloud_axis.set_title(message)

    def _setup_cloud_axis(self) -> None:
        self.cloud_axis.set_aspect("equal", adjustable="box")
        self.cloud_axis.set_xlim(-self.max_range_m, self.max_range_m)
        self.cloud_axis.set_ylim(-self.max_range_m, self.max_range_m)
        self.cloud_axis.set_xlabel("x (m)")
        self.cloud_axis.set_ylabel("y (m)")
        self.cloud_axis.grid(True, alpha=0.25)

    def _setup_map_axis(self) -> None:
        assert self.occupancy_grid is not None
        self.map_axis.set_title("BreezySLAM occupancy grid")
        self.map_axis.set_aspect("equal", adjustable="box")
        self.map_axis.set_xlim(0, self.occupancy_grid.size_pixels)
        self.map_axis.set_ylim(self.occupancy_grid.size_pixels, 0)
        self.map_axis.set_xticks([])
        self.map_axis.set_yticks([])

    def _update_map(self, slam_result: SlamResult) -> None:
        assert self.occupancy_grid is not None
        grid = self.occupancy_grid.from_bytes(slam_result.map_bytes)

        if self.map_image is None:
            self.map_image = self.map_axis.imshow(grid, cmap="gray", vmin=0, vmax=255)
        else:
            self.map_image.set_data(grid)

        pose_x, pose_y = self.occupancy_grid.pose_to_pixels(
            slam_result.pose.x_mm,
            slam_result.pose.y_mm,
        )
        heading = math.radians(slam_result.pose.theta_degrees)
        dx = 12 * math.cos(heading)
        dy = 12 * math.sin(heading)

        if self.pose_artist is not None:
            self.pose_artist.remove()
        self.pose_artist = self.map_axis.arrow(
            pose_x,
            pose_y,
            dx,
            dy,
            width=2.0,
            color="tab:red",
            length_includes_head=True,
        )
        self.map_axis.set_title(
            f"Pose x={slam_result.pose.x_mm / 1000:.2f} m "
            f"y={slam_result.pose.y_mm / 1000:.2f} m "
            f"theta={slam_result.pose.theta_degrees:.1f} deg"
        )

    def _on_close(self, _event: object) -> None:
        self._closed = True
