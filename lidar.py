"""Read Neato XV11 / LDS UART packets and publish 360-degree scans."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterator, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - exercised when dependencies are absent
    serial = None
    list_ports = None


PACKET_SIZE = 22
START_BYTE = 0xFA
FIRST_INDEX = 0xA0
LAST_INDEX = 0xF9
SCAN_SIZE = 360


@dataclass(frozen=True)
class LidarScan:
    """One normalized revolution from the XV11 LIDAR."""

    distances_mm: list[int]
    intensities: list[int]
    valid: list[bool]
    rpm: float


@dataclass(frozen=True)
class LidarStats:
    packets: int
    scans: int
    timeouts: int
    bad_index: int
    bad_checksum: int
    valid_points: int
    last_rpm: float


def available_ports() -> list[str]:
    """Return visible serial port device names."""

    if list_ports is None:
        return []
    return [port.device for port in list_ports.comports()]


class XV11Lidar:
    """Serial reader for XV11-compatible 22-byte data packets."""

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout: float = 1.0,
        min_distance_mm: int = 80,
        max_distance_mm: int = 6000,
        require_checksum: bool = True,
    ) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")

        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.min_distance_mm = min_distance_mm
        self.max_distance_mm = max_distance_mm
        self.require_checksum = require_checksum
        self._serial: Optional[serial.Serial] = None
        self.packets = 0
        self.scans = 0
        self.timeouts = 0
        self.bad_index = 0
        self.bad_checksum = 0
        self.valid_points = 0
        self.last_rpm = 0.0

    def __enter__(self) -> "XV11Lidar":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def open(self) -> None:
        self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
        self._serial.reset_input_buffer()

    def close(self) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()

    @property
    def stats(self) -> LidarStats:
        return LidarStats(
            packets=self.packets,
            scans=self.scans,
            timeouts=self.timeouts,
            bad_index=self.bad_index,
            bad_checksum=self.bad_checksum,
            valid_points=self.valid_points,
            last_rpm=self.last_rpm,
        )

    def iter_scans(
        self,
        idle_callback: Optional[Callable[[LidarStats], bool]] = None,
    ) -> Iterator[LidarScan]:
        """Yield complete scans as the packet index wraps from 0xF9 to 0xA0."""

        if self._serial is None:
            self.open()

        distances = [0] * SCAN_SIZE
        intensities = [0] * SCAN_SIZE
        valid = [False] * SCAN_SIZE
        rpm_values: list[float] = []
        last_packet_index: Optional[int] = None
        packets_since_idle = 0

        while True:
            packet = self._read_packet()
            if packet is None:
                if idle_callback is not None and not idle_callback(self.stats):
                    return
                continue

            self.packets += 1
            packets_since_idle += 1
            packet_index = packet[1]
            if not FIRST_INDEX <= packet_index <= LAST_INDEX:
                self.bad_index += 1
                if idle_callback is not None and packets_since_idle >= 25:
                    packets_since_idle = 0
                    if not idle_callback(self.stats):
                        return
                continue

            if self.require_checksum and not self._checksum_ok(packet):
                self.bad_checksum += 1
                if idle_callback is not None and packets_since_idle >= 25:
                    packets_since_idle = 0
                    if not idle_callback(self.stats):
                        return
                continue

            block_index = packet_index - FIRST_INDEX
            if last_packet_index is not None and packet_index < last_packet_index:
                if any(valid):
                    rpm = sum(rpm_values) / len(rpm_values) if rpm_values else 0.0
                    scan = LidarScan(distances[:], intensities[:], valid[:], rpm)
                    self.scans += 1
                    self.valid_points = sum(scan.valid)
                    self.last_rpm = scan.rpm
                    yield scan
                distances = [0] * SCAN_SIZE
                intensities = [0] * SCAN_SIZE
                valid = [False] * SCAN_SIZE
                rpm_values = []

            rpm_values.append(self._decode_rpm(packet))
            self._decode_measurements(packet, block_index, distances, intensities, valid)
            last_packet_index = packet_index

    def _read_packet(self) -> Optional[bytes]:
        assert self._serial is not None

        while True:
            start = self._serial.read(1)
            if not start:
                self.timeouts += 1
                return None
            if start[0] == START_BYTE:
                rest = self._serial.read(PACKET_SIZE - 1)
                if len(rest) == PACKET_SIZE - 1:
                    return start + rest
                return None

    def _decode_measurements(
        self,
        packet: bytes,
        block_index: int,
        distances: list[int],
        intensities: list[int],
        valid: list[bool],
    ) -> None:
        for sample in range(4):
            offset = 4 + sample * 4
            low = packet[offset]
            high_and_flags = packet[offset + 1]
            intensity = packet[offset + 2] | (packet[offset + 3] << 8)

            distance = low | ((high_and_flags & 0x3F) << 8)
            invalid = bool(high_and_flags & 0x80)
            angle = block_index * 4 + sample

            distances[angle] = distance
            intensities[angle] = intensity
            valid[angle] = (
                not invalid
                and self.min_distance_mm <= distance <= self.max_distance_mm
            )

    @staticmethod
    def _decode_rpm(packet: bytes) -> float:
        speed = packet[2] | (packet[3] << 8)
        return speed / 64.0

    @staticmethod
    def _checksum_ok(packet: bytes) -> bool:
        """Validate the XV11 15-bit checksum over the first 20 bytes."""

        expected = packet[20] | (packet[21] << 8)
        checksum = 0
        for i in range(0, 20, 2):
            word = packet[i] | (packet[i + 1] << 8)
            checksum = (checksum << 1) + word
        checksum = (checksum & 0x7FFF) + (checksum >> 15)
        checksum = checksum & 0x7FFF
        return checksum == expected
