"""BreezySLAM integration for 360-sample XV11 scans."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from lidar import SCAN_SIZE, LidarScan


@dataclass(frozen=True)
class Pose:
    x_mm: float
    y_mm: float
    theta_degrees: float


@dataclass(frozen=True)
class SlamResult:
    pose: Pose
    map_bytes: bytes


class BreezySlamRunner:
    """Thin adapter around BreezySLAM's RMHC_SLAM algorithm."""

    def __init__(
        self,
        map_size_pixels: int = 500,
        map_size_meters: float = 10.0,
        scan_rate_hz: float = 5.5,
        detection_angle_degrees: int = 360,
        distance_no_detection_mm: int = 6000,
        hole_width_mm: int = 200,
    ) -> None:
        try:
            from breezyslam.algorithms import RMHC_SLAM
            from breezyslam.sensors import Laser
        except ImportError as exc:  # pragma: no cover - depends on local env
            raise RuntimeError(
                "BreezySLAM is not installed. Run `pip install breezyslam`, "
                "or start with `python main.py --mode pointcloud`."
            ) from exc

        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.distance_no_detection_mm = distance_no_detection_mm
        self._map = bytearray(map_size_pixels * map_size_pixels)

        laser = Laser(
            SCAN_SIZE,
            scan_rate_hz,
            detection_angle_degrees,
            distance_no_detection_mm,
            detection_margin=0,
            offset_mm=0,
        )
        self._slam = RMHC_SLAM(
            laser,
            map_size_pixels,
            map_size_meters,
            hole_width_mm=hole_width_mm,
        )

    def update(
        self,
        scan: LidarScan,
        pose_change: Optional[tuple[float, float, float]] = None,
    ) -> SlamResult:
        """Update SLAM from one scan.

        pose_change is optional odometry in BreezySLAM format:
        (forward_delta_mm, rotation_delta_degrees, elapsed_seconds).
        """

        distances = [
            distance if is_valid else self.distance_no_detection_mm
            for distance, is_valid in zip(scan.distances_mm, scan.valid)
        ]

        if pose_change is None:
            self._slam.update(distances)
        else:
            self._slam.update(distances, pose_change=pose_change)

        x_mm, y_mm, theta_degrees = self._slam.getpos()
        self._slam.getmap(self._map)
        return SlamResult(Pose(x_mm, y_mm, theta_degrees), bytes(self._map))
