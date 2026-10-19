#!/usr/bin/env python3
"""Map USB serial ports to logical names stored in a DYNAMIXEL register."""

from __future__ import annotations

import argparse
import glob
import logging
import os
import re
import selectors
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

LOG = logging.getLogger("robot-port-mapper")
NAME_PATTERN = re.compile(r"^[A-Z][A-Z0-9_-]{0,11}$")


@dataclass(frozen=True)
class DeviceMapping:
    logical_name: str
    port: str
    real_device: str
    usb_serial: str
    dynamixel_id: int


class RobotPortMapper:
    def __init__(
        self,
        device_glob: str,
        link_dir: Path,
        vendor_patterns: tuple[str, ...],
        baudrate: int,
        protocol: float,
        dynamixel_id: int,
        address: int,
        length: int,
        retry_interval: float,
    ) -> None:
        self.device_glob = device_glob
        self.link_dir = link_dir
        self.vendor_patterns = tuple(value.upper() for value in vendor_patterns)
        self.baudrate = baudrate
        self.protocol = protocol
        self.dynamixel_id = dynamixel_id
        self.address = address
        self.length = length
        self.retry_interval = retry_interval
        self.mappings: dict[str, DeviceMapping] = {}
        self.failed_at: dict[str, float] = {}
        self.stop_requested = False

    def request_stop(self, _signum: int, _frame: object) -> None:
        self.stop_requested = True

    def candidates(self) -> list[str]:
        paths = sorted(glob.glob(self.device_glob))
        return [path for path in paths if self._matches_vendor(path)]

    def _matches_vendor(self, path: str) -> bool:
        if not self.vendor_patterns:
            return True
        name = Path(path).name.upper()
        return any(pattern in name for pattern in self.vendor_patterns)

    @staticmethod
    def _usb_serial(path: str) -> str:
        name = Path(path).name
        match = re.match(r"usb-.+_([^_]+)-if\d+(?:-port\d+)?$", name)
        return match.group(1) if match else name

    def read_logical_name(self, path: str) -> Optional[str]:
        port = PortHandler(path)
        packet = PacketHandler(self.protocol)
        try:
            if not port.openPort():
                raise RuntimeError("could not open port")
            if not port.setBaudRate(self.baudrate):
                raise RuntimeError(f"could not set baud rate {self.baudrate}")
            data, communication_result, packet_error = packet.readTxRx(
                port, self.dynamixel_id, self.address, self.length
            )
            if communication_result != COMM_SUCCESS:
                raise RuntimeError(packet.getTxRxResult(communication_result))
            if packet_error:
                raise RuntimeError(packet.getRxPacketError(packet_error))
            raw = bytes(data)
            logical_name = raw.split(b"\x00", 1)[0].decode("ascii")
            if not NAME_PATTERN.fullmatch(logical_name):
                raise RuntimeError(
                    f"invalid logical name {logical_name!r} from {raw.hex(' ')}"
                )
            return logical_name
        except Exception as exc:
            LOG.warning("Could not identify %s: %s", path, exc)
            return None
        finally:
            try:
                port.closePort()
            except Exception:
                pass

    def reconcile(self, force_retry: bool = False) -> None:
        now = time.monotonic()
        candidates = self.candidates()
        candidate_set = set(candidates)

        for path in list(self.mappings):
            if path not in candidate_set or not Path(path).exists():
                mapping = self.mappings.pop(path)
                self.failed_at.pop(path, None)
                LOG.info("Removed %s (%s)", mapping.logical_name, path)

        for path in list(self.failed_at):
            if path not in candidate_set:
                self.failed_at.pop(path, None)

        for path in candidates:
            if path in self.mappings:
                continue
            last_failure = self.failed_at.get(path)
            if (
                not force_retry
                and last_failure is not None
                and now - last_failure < self.retry_interval
            ):
                continue
            logical_name = self.read_logical_name(path)
            if logical_name is None:
                self.failed_at[path] = now
                continue
            self.failed_at.pop(path, None)
            mapping = DeviceMapping(
                logical_name=logical_name,
                port=path,
                real_device=os.path.realpath(path),
                usb_serial=self._usb_serial(path),
                dynamixel_id=self.dynamixel_id,
            )
            self.mappings[path] = mapping
            LOG.info(
                "Mapped %s -> %s (USB serial %s)",
                logical_name,
                path,
                mapping.usb_serial,
            )

        self.publish()

    def publish(self) -> None:
        self.link_dir.mkdir(parents=True, exist_ok=True)
        grouped: dict[str, list[DeviceMapping]] = {}
        for mapping in self.mappings.values():
            grouped.setdefault(mapping.logical_name, []).append(mapping)

        desired_links: dict[str, str] = {}
        for logical_name, mappings in sorted(grouped.items()):
            if len(mappings) == 1:
                mapping = mappings[0]
                desired_links[logical_name.lower()] = mapping.port
            else:
                LOG.error(
                    "Duplicate logical name %s on: %s",
                    logical_name,
                    ", ".join(item.port for item in mappings),
                )

        for entry in self.link_dir.iterdir():
            if not entry.is_symlink():
                continue
            if entry.name not in desired_links:
                entry.unlink(missing_ok=True)

        for logical_name, target in desired_links.items():
            link = self.link_dir / logical_name
            if link.is_symlink() and os.readlink(link) == target:
                continue
            temporary_link = self.link_dir / f".{logical_name}.{os.getpid()}.tmp"
            temporary_link.unlink(missing_ok=True)
            os.symlink(target, temporary_link)
            os.replace(temporary_link, link)

    def run(self, rescan_interval: float) -> int:
        self.link_dir.mkdir(parents=True, exist_ok=True)
        self.reconcile(force_retry=True)

        command = [
            "udevadm",
            "monitor",
            "--udev",
            "--property",
            "--subsystem-match=tty",
        ]
        try:
            monitor = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=0,
            )
        except OSError as exc:
            LOG.error("Could not start udev monitor: %s", exc)
            return 1

        selector = selectors.DefaultSelector()
        assert monitor.stdout is not None
        selector.register(monitor.stdout, selectors.EVENT_READ)
        try:
            while not self.stop_requested:
                events = selector.select(timeout=rescan_interval)
                if events:
                    for key, _mask in events:
                        event_data = os.read(key.fileobj.fileno(), 65_536)
                        if not event_data:
                            if monitor.poll() is not None:
                                LOG.error("udevadm monitor exited with %s", monitor.returncode)
                                return 1
                            continue
                        # udevadm is already filtered to the tty subsystem, so any
                        # output means a serial device changed. Reading raw bytes
                        # avoids TextIO buffering delaying an event until rescan.
                        time.sleep(0.3)
                        self.reconcile(force_retry=True)
                else:
                    self.reconcile(force_retry=False)
            return 0
        finally:
            selector.close()
            monitor.terminate()
            try:
                monitor.wait(timeout=2)
            except subprocess.TimeoutExpired:
                monitor.kill()
                monitor.wait()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device-glob",
        default=os.getenv("ROBOT_PORT_DEVICE_GLOB", "/dev/serial/by-id/*"),
    )
    parser.add_argument(
        "--link-dir",
        type=Path,
        default=Path(os.getenv("ROBOT_PORT_LINK_DIR", "/dev/serial/by-role")),
        help="directory containing logical serial-port symlinks",
    )
    parser.add_argument(
        "--vendor-patterns",
        default=os.getenv(
            "ROBOT_PORT_VENDOR_PATTERNS", "ROBOTIS_AVATAR_CONTROLLER"
        ),
        help=(
            "comma-separated, case-insensitive substrings matched against by-id names "
            "(default: ROBOTIS_Avatar_Controller only)"
        ),
    )
    parser.add_argument("--baudrate", type=int, default=4_000_000)
    parser.add_argument("--protocol", type=float, default=2.0)
    parser.add_argument("--id", type=int, default=200, dest="dynamixel_id")
    parser.add_argument("--address", type=int, default=10_001)
    parser.add_argument("--length", type=int, default=12)
    parser.add_argument("--rescan-interval", type=float, default=60.0)
    parser.add_argument("--retry-interval", type=float, default=30.0)
    parser.add_argument(
        "--once", action="store_true", help="scan once, publish mappings, and exit"
    )
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    vendor_patterns = tuple(
        value.strip() for value in args.vendor_patterns.split(",") if value.strip()
    )
    mapper = RobotPortMapper(
        device_glob=args.device_glob,
        link_dir=args.link_dir,
        vendor_patterns=vendor_patterns,
        baudrate=args.baudrate,
        protocol=args.protocol,
        dynamixel_id=args.dynamixel_id,
        address=args.address,
        length=args.length,
        retry_interval=args.retry_interval,
    )
    if not 0 <= args.dynamixel_id <= 252:
        LOG.error("DYNAMIXEL ID must be from 0 to 252")
        return 2
    if args.length <= 0 or not 0 <= args.address <= 0xFFFF:
        LOG.error("Invalid address or length")
        return 2
    signal.signal(signal.SIGTERM, mapper.request_stop)
    signal.signal(signal.SIGINT, mapper.request_stop)
    if args.once:
        mapper.reconcile(force_retry=True)
        return 0
    return mapper.run(args.rescan_interval)


if __name__ == "__main__":
    raise SystemExit(main())
