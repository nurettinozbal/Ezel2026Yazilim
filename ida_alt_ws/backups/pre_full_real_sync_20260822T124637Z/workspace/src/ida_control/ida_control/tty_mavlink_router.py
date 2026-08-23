"""Single-owner Pixhawk TTY fan-out for the onboard bridge.

The Pixhawk serial device must never be opened concurrently by MAVSDK and
pymavlink.  This small process owns the TTY and fans raw MAVLink bytes to
*loopback-only* UDP sockets.  The sockets are an in-process-host transport; no
MAVLink port is exposed on the vehicle network.

No MAVLink command is created or interpreted here.  If the serial device
closes, the process exits non-zero so the parent launch can shut the vehicle
stack down fail-closed.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import select
import socket
import sys
import time
from typing import Iterable


ALLOWED_BAUDRATES = frozenset({57600, 115200, 230400, 460800, 921600})
DEFAULT_ENDPOINTS = (14540, 14541, 14542)
MAX_PACKET_BYTES = 65535


def validate_config(device: object, baud: object, ports: Iterable[object]) -> tuple[str, int, tuple[int, ...]]:
    """Return a strict, local-only router configuration."""
    if not isinstance(device, str) or not device.startswith("/dev/"):
        raise ValueError("Pixhawk device must be an absolute /dev path")
    path = Path(device)
    if path.name in {"", ".", ".."} or "\x00" in device:
        raise ValueError("invalid Pixhawk device path")
    if isinstance(baud, bool):
        raise ValueError("baud must be an exact integer")
    try:
        baud_int = int(baud)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("baud must be an exact integer") from exc
    if str(baud_int) != str(baud) or baud_int not in ALLOWED_BAUDRATES:
        raise ValueError(f"baud must be one of {sorted(ALLOWED_BAUDRATES)}")

    parsed_ports: list[int] = []
    for value in ports:
        if isinstance(value, bool):
            raise ValueError("endpoint ports must be exact integers")
        try:
            port = int(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("endpoint ports must be exact integers") from exc
        if str(port) != str(value) or not 1024 <= port <= 65535:
            raise ValueError("endpoint ports must be exact integers in [1024,65535]")
        parsed_ports.append(port)
    if not parsed_ports or len(set(parsed_ports)) != len(parsed_ports):
        raise ValueError("endpoint ports must be nonempty and unique")
    return device, baud_int, tuple(parsed_ports)


def open_endpoint_sockets(ports: Iterable[int]) -> list[tuple[socket.socket, tuple[str, int]]]:
    """Create ephemeral loopback sockets paired with fixed consumer ports."""
    endpoints: list[tuple[socket.socket, tuple[str, int]]] = []
    try:
        for port in ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", 0))
            sock.setblocking(False)
            endpoints.append((sock, ("127.0.0.1", port)))
        return endpoints
    except Exception:
        for sock, _target in endpoints:
            sock.close()
        raise


def forward_ready(pixhawk, endpoints, ready) -> None:
    """Forward one select cycle; split out for deterministic unit testing."""
    if pixhawk in ready:
        payload = pixhawk.read(MAX_PACKET_BYTES)
        if payload:
            for sock, target in endpoints:
                sock.sendto(payload, target)
    for sock, _target in endpoints:
        if sock not in ready:
            continue
        payload, source = sock.recvfrom(MAX_PACKET_BYTES)
        if source[0] != "127.0.0.1":
            continue
        if payload:
            pixhawk.write(payload)


def run_router(device: str, baud: int, ports: tuple[int, ...]) -> None:
    """Forward bytes until interrupted or the physical serial link fails."""
    try:
        import serial
    except ImportError as exc:  # pymavlink installs pyserial on the Jetson.
        raise RuntimeError("pyserial is required for the Pixhawk TTY router") from exc

    if not os.path.exists(device):
        raise FileNotFoundError(f"Pixhawk serial device not found: {device}")

    endpoints = open_endpoint_sockets(ports)
    try:
        with serial.Serial(
            port=device,
            baudrate=baud,
            timeout=0,
            write_timeout=0.5,
            exclusive=True,
        ) as pixhawk:
            print(
                "[ida-tty-router] physical=%s baud=%d loopback=%s"
                % (device, baud, ",".join(str(port) for port in ports)),
                flush=True,
            )
            while True:
                readers = [pixhawk, *(sock for sock, _target in endpoints)]
                ready, _writable, errors = select.select(readers, [], readers, 0.5)
                if errors:
                    raise ConnectionError("Pixhawk TTY/loopback endpoint failed")
                forward_ready(pixhawk, endpoints, ready)
    finally:
        for sock, _target in endpoints:
            sock.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="IDA single-owner Pixhawk TTY router")
    parser.add_argument("--device", required=True)
    parser.add_argument("--baud", default="115200")
    parser.add_argument("--endpoint-port", action="append", default=[])
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    ports = args.endpoint_port or [str(port) for port in DEFAULT_ENDPOINTS]
    try:
        device, baud, parsed_ports = validate_config(args.device, args.baud, ports)
        run_router(device, baud, parsed_ports)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"[ida-tty-router] fatal: {exc}", file=sys.stderr, flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
