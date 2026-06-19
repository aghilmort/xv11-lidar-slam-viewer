"""Run XV11 point-cloud visualization and optional BreezySLAM localization."""

from __future__ import annotations

import argparse
import sys
import time

from lidar import XV11Lidar, available_ports


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Neato XV11 LIDAR viewer and BreezySLAM runner")
    parser.add_argument("--port", help="Serial port, e.g. COM3 on Windows or /dev/ttyUSB0 on Linux")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--mode", choices=["pointcloud", "slam"], default="pointcloud")
    parser.add_argument("--list-ports", action="store_true", help="List serial ports and exit")
    parser.add_argument("--min-distance-mm", type=int, default=80)
    parser.add_argument("--max-distance-mm", type=int, default=6000)
    parser.add_argument("--disable-checksum", action="store_true", help="Accept packets without checksum validation")
    parser.add_argument("--debug-lidar", action="store_true", help="Print packet and scan counters once per second")
    parser.add_argument("--map-size-pixels", type=int, default=500)
    parser.add_argument("--map-size-meters", type=float, default=10.0)
    parser.add_argument("--scan-rate-hz", type=float, default=5.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list_ports:
        ports = available_ports()
        if ports:
            print("\n".join(ports))
        else:
            print("No serial ports found, or pyserial is not installed.")
        return 0

    if not args.port:
        print("Missing --port. Use --list-ports to find the UART adapter.", file=sys.stderr)
        return 2

    try:
        from viewer import Viewer
    except ImportError as exc:
        if exc.name in {"matplotlib", "numpy"}:
            print(
                f"Missing Python package: {exc.name}. "
                "Install point-cloud dependencies with: "
                "pip install pyserial numpy matplotlib",
                file=sys.stderr,
            )
            return 1
        raise

    slam_runner = None
    occupancy_grid = None
    if args.mode == "slam":
        try:
            from map import OccupancyGrid
            from slam import BreezySlamRunner
        except ImportError as exc:
            print(
                f"Missing Python package: {exc.name}. "
                "Install all dependencies with: pip install -r requirements.txt",
                file=sys.stderr,
            )
            return 1

        occupancy_grid = OccupancyGrid(args.map_size_pixels, args.map_size_meters)
        slam_runner = BreezySlamRunner(
            map_size_pixels=args.map_size_pixels,
            map_size_meters=args.map_size_meters,
            scan_rate_hz=args.scan_rate_hz,
            distance_no_detection_mm=args.max_distance_mm,
        )

    viewer = Viewer(
        mode=args.mode,
        max_range_m=args.max_distance_mm / 1000.0,
        occupancy_grid=occupancy_grid,
    )
    last_debug_at = 0.0

    def on_lidar_idle(stats):
        nonlocal last_debug_at

        if args.debug_lidar and time.monotonic() - last_debug_at >= 1.0:
            print(
                "lidar "
                f"packets={stats.packets} "
                f"scans={stats.scans} "
                f"timeouts={stats.timeouts} "
                f"bad_index={stats.bad_index} "
                f"bad_checksum={stats.bad_checksum} "
                f"valid_points={stats.valid_points} "
                f"rpm={stats.last_rpm:.1f}",
                flush=True,
            )
            last_debug_at = time.monotonic()

        return viewer.idle(stats)

    with XV11Lidar(
        port=args.port,
        baud=args.baud,
        timeout=0.05,
        min_distance_mm=args.min_distance_mm,
        max_distance_mm=args.max_distance_mm,
        require_checksum=not args.disable_checksum,
    ) as lidar:
        for scan in lidar.iter_scans(idle_callback=on_lidar_idle):
            slam_result = slam_runner.update(scan) if slam_runner is not None else None
            viewer.update(scan, slam_result)
            if args.debug_lidar and time.monotonic() - last_debug_at >= 1.0:
                stats = lidar.stats
                print(
                    "lidar "
                    f"packets={stats.packets} "
                    f"scans={stats.scans} "
                    f"timeouts={stats.timeouts} "
                    f"bad_index={stats.bad_index} "
                    f"bad_checksum={stats.bad_checksum} "
                    f"valid_points={stats.valid_points} "
                    f"rpm={stats.last_rpm:.1f}",
                    flush=True,
                )
                last_debug_at = time.monotonic()
            if viewer.closed:
                break

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
