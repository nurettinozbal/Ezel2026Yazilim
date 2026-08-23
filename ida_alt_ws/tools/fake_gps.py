#!/usr/bin/env python3
"""Reversible bench-only GPS_INPUT helper.

This tool never arms, changes vehicle mode, uploads a mission or emits actuator
commands.  Parameter preparation and restoration are explicit subcommands.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import time
from typing import Any


GPS_INPUT_TYPE = 14
EXPECTED_REAL_GPS_TYPE = 9
# 14540 MAVSDK, 14541 pymavlink bridge tarafından dinlenir. Fake GPS ayrı
# router çıkışını kullanır; aksi halde çalışan saha stack'iyle bind çakışır.
DEFAULT_CONNECTION = "udp:127.0.0.1:14542"
BACKUP_DIR = Path.home() / "ida_alt_ws" / "bench_backups"
ACTIVE_BACKUP = BACKUP_DIR / "fake_gps_active.json"


def validate_coordinates(lat: object, lon: object, alt_m: object) -> tuple[float, float, float]:
    try:
        values = (float(lat), float(lon), float(alt_m))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("coordinates must be finite numbers") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("coordinates must be finite numbers")
    latitude, longitude, altitude = values
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise ValueError("latitude/longitude outside geographic range")
    if not -1000.0 <= altitude <= 10000.0:
        raise ValueError("altitude outside bench range")
    return latitude, longitude, altitude


def _mavutil():
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise RuntimeError("pymavlink is required") from exc
    return mavutil


def _connect(connection: str):
    mavutil = _mavutil()
    conn = mavutil.mavlink_connection(
        connection,
        source_system=254,
        source_component=190,
        autoreconnect=False,
    )
    heartbeat = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=8)
    if heartbeat is None:
        conn.close()
        raise RuntimeError("Pixhawk heartbeat was not received")
    conn.target_system = heartbeat.get_srcSystem()
    conn.target_component = heartbeat.get_srcComponent() or 1
    return mavutil, conn, heartbeat


def _require_disarmed(mavutil: Any, heartbeat: Any) -> None:
    armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    if int(getattr(heartbeat, "base_mode", 0)) & armed_flag:
        raise RuntimeError("vehicle is armed; parameter operation rejected")


def _param_name(message: Any) -> str:
    raw = getattr(message, "param_id", "")
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", "ignore")
    return str(raw).rstrip("\x00")


def _read_param(conn: Any, name: str, timeout_s: float = 4.0) -> float | None:
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component, name.encode("ascii"), -1
    )
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
        if message is not None and _param_name(message) == name:
            return float(message.param_value)
    return None


def _verified_runtime_backup(path: Path, target_system: int) -> dict[str, Any]:
    """Validate the persistent proof created by prepare/adopt.

    The long-running publisher must not request PARAM_VALUE while the vehicle is
    armed. Some pymavlink releases also crash their instance cache when the same
    routed parameter stream is consumed by multiple clients. Parameter mutation
    remains confined to explicit prepare/restore commands; runtime only checks
    this immutable local proof and emits GPS_INPUT.
    """
    if not path.is_file():
        raise RuntimeError("active fake GPS backup not found; prepare/adopt is required")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        temporary = float(payload["gps1_type_temporary"])
        original = float(payload["gps1_type_before"])
        recorded_system = int(payload["target_system"])
    except (OSError, TypeError, ValueError, KeyError, json.JSONDecodeError) as exc:
        raise RuntimeError("active fake GPS backup is malformed") from exc
    if int(round(temporary)) != GPS_INPUT_TYPE:
        raise RuntimeError("active backup does not prove GPS_INPUT mode")
    if int(round(original)) != EXPECTED_REAL_GPS_TYPE:
        raise RuntimeError("active backup does not preserve the real GPS type")
    if recorded_system != int(target_system):
        raise RuntimeError("active backup belongs to another MAVLink system")
    return payload


def _set_param(conn: Any, mavutil: Any, name: str, value: float) -> float:
    conn.mav.param_set_send(
        conn.target_system,
        conn.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        message = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
        if message is not None and _param_name(message) == name:
            confirmed = float(message.param_value)
            if int(round(confirmed)) == int(round(value)):
                return confirmed
    raise RuntimeError(f"{name}={value} readback was not confirmed")


def _reboot(conn: Any, mavutil: Any) -> None:
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def prepare(connection: str, backup_dir: Path) -> dict[str, Any]:
    mavutil, conn, heartbeat = _connect(connection)
    try:
        _require_disarmed(mavutil, heartbeat)
        current = _read_param(conn, "GPS1_TYPE")
        if current is None:
            raise RuntimeError("GPS1_TYPE could not be read")
        if int(round(current)) != EXPECTED_REAL_GPS_TYPE:
            raise RuntimeError(
                f"GPS1_TYPE expected {EXPECTED_REAL_GPS_TYPE}, got {current}; no change"
            )
        backup_dir.mkdir(parents=True, exist_ok=True)
        active = backup_dir / ACTIVE_BACKUP.name
        if active.exists():
            raise RuntimeError(f"active fake GPS backup already exists: {active}")
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        backup = backup_dir / f"fake_gps_backup_{stamp}.json"
        payload = {
            "schema_version": 1,
            "created_utc": stamp,
            "connection": connection,
            "target_system": conn.target_system,
            "target_component": conn.target_component,
            "gps1_type_before": current,
            "gps1_type_temporary": GPS_INPUT_TYPE,
            "backup": str(backup),
        }
        backup.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        try:
            active.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
            confirmed = _set_param(conn, mavutil, "GPS1_TYPE", GPS_INPUT_TYPE)
        except Exception:
            active.unlink(missing_ok=True)
            raise
        payload["gps1_type_after"] = confirmed
        _reboot(conn, mavutil)
        return payload
    finally:
        conn.close()


def restore(connection: str, backup_dir: Path) -> dict[str, Any]:
    active = backup_dir / ACTIVE_BACKUP.name
    if not active.is_file():
        raise RuntimeError(f"active fake GPS backup not found: {active}")
    payload = json.loads(active.read_text(encoding="utf-8"))
    wanted = float(payload["gps1_type_before"])
    if int(round(wanted)) != EXPECTED_REAL_GPS_TYPE:
        raise RuntimeError("backup does not contain the expected real GPS type")
    mavutil, conn, heartbeat = _connect(connection)
    try:
        _require_disarmed(mavutil, heartbeat)
        current = _read_param(conn, "GPS1_TYPE")
        if current is None or int(round(current)) != GPS_INPUT_TYPE:
            raise RuntimeError(f"temporary GPS1_TYPE={GPS_INPUT_TYPE} expected, got {current}")
        confirmed = _set_param(conn, mavutil, "GPS1_TYPE", wanted)
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        record = {
            "schema_version": 1,
            "created_utc": stamp,
            "backup": payload.get("backup"),
            "gps1_type_before": current,
            "gps1_type_restored": confirmed,
        }
        record_path = backup_dir / f"fake_gps_restore_{stamp}.json"
        record_path.write_text(json.dumps(record, indent=2, sort_keys=True), encoding="utf-8")
        active.unlink()
        _reboot(conn, mavutil)
        return record
    finally:
        conn.close()


def adopt_backup(connection: str, backup_dir: Path, backup_path: Path) -> dict[str, Any]:
    """Bind an older verified GPS1_TYPE=9 backup to an already prepared vehicle."""
    backup_root = backup_dir.resolve()
    candidate = backup_path.expanduser().resolve()
    if candidate.parent != backup_root or not candidate.is_file():
        raise RuntimeError("backup must be an existing file directly inside the backup directory")
    active = backup_dir / ACTIVE_BACKUP.name
    if active.exists():
        raise RuntimeError(f"active fake GPS backup already exists: {active}")
    source = json.loads(candidate.read_text(encoding="utf-8"))
    original = source.get("gps1_type_before")
    if original is None:
        original = source.get("before", {}).get("GPS1_TYPE")
    if original is None or int(round(float(original))) != EXPECTED_REAL_GPS_TYPE:
        raise RuntimeError("selected backup does not prove GPS1_TYPE=9")
    mavutil, conn, heartbeat = _connect(connection)
    try:
        _require_disarmed(mavutil, heartbeat)
        current = _read_param(conn, "GPS1_TYPE")
        if current is None or int(round(current)) != GPS_INPUT_TYPE:
            raise RuntimeError(f"vehicle GPS1_TYPE={GPS_INPUT_TYPE} expected, got {current}")
        if source.get("target_system") not in (None, conn.target_system):
            raise RuntimeError("backup target_system does not match the connected vehicle")
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        payload = {
            "schema_version": 1,
            "created_utc": stamp,
            "connection": connection,
            "target_system": conn.target_system,
            "target_component": conn.target_component,
            "gps1_type_before": float(original),
            "gps1_type_temporary": current,
            "backup": str(candidate),
            "adopted_existing_backup": True,
        }
        active.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        return payload
    finally:
        conn.close()


def check_prepared(connection: str) -> dict[str, Any]:
    _mav, conn, heartbeat = _connect(connection)
    try:
        current = _read_param(conn, "GPS1_TYPE")
        return {
            "armed": bool(int(getattr(heartbeat, "base_mode", 0)) & 128),
            "gps1_type": current,
            "prepared": current is not None and int(round(current)) == GPS_INPUT_TYPE,
        }
    finally:
        conn.close()


def run(connection: str, lat: float, lon: float, alt_m: float, rate_hz: float) -> None:
    latitude, longitude, altitude = validate_coordinates(lat, lon, alt_m)
    if not math.isfinite(rate_hz) or not 1.0 <= rate_hz <= 20.0:
        raise ValueError("rate must be in [1,20] Hz")
    mavutil, conn, _heartbeat = _connect(connection)
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    _verified_runtime_backup(ACTIVE_BACKUP, conn.target_system)
    interval = 1.0 / rate_hz
    next_send = time.monotonic()
    print(
        json.dumps({"event": "fake_gps_started", "lat": latitude, "lon": longitude,
                    "alt_m": altitude, "rate_hz": rate_hz}, sort_keys=True),
        flush=True,
    )
    try:
        while running:
            now = time.monotonic()
            if now >= next_send:
                conn.mav.gps_input_send(
                    int(time.time() * 1_000_000),
                    0, 0, 0, 0, 3,
                    int(round(latitude * 1e7)),
                    int(round(longitude * 1e7)),
                    altitude,
                    0.7, 1.0,
                    0.0, 0.0, 0.0,
                    0.3, 0.8, 1.0,
                    12, 0,
                )
                next_send += interval
            time.sleep(min(0.05, max(0.0, next_send - time.monotonic())))
    finally:
        conn.close()


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--connection", default=DEFAULT_CONNECTION)
    ap.add_argument("--backup-dir", type=Path, default=BACKUP_DIR)
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("prepare")
    sub.add_parser("restore")
    sub.add_parser("check")
    validate_parser = sub.add_parser("validate")
    validate_parser.add_argument("--lat", type=float, required=True)
    validate_parser.add_argument("--lon", type=float, required=True)
    validate_parser.add_argument("--alt-m", type=float, default=1015.0)
    adopt_parser = sub.add_parser("adopt")
    adopt_parser.add_argument("--backup", type=Path, required=True)
    run_parser = sub.add_parser("run")
    run_parser.add_argument("--lat", type=float, required=True)
    run_parser.add_argument("--lon", type=float, required=True)
    run_parser.add_argument("--alt-m", type=float, default=1015.0)
    run_parser.add_argument("--rate-hz", type=float, default=5.0)
    return ap


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.command == "prepare":
            result = prepare(args.connection, args.backup_dir)
        elif args.command == "restore":
            result = restore(args.connection, args.backup_dir)
        elif args.command == "check":
            result = check_prepared(args.connection)
        elif args.command == "validate":
            latitude, longitude, altitude = validate_coordinates(args.lat, args.lon, args.alt_m)
            result = {"valid": True, "lat": latitude, "lon": longitude, "alt_m": altitude}
        elif args.command == "adopt":
            result = adopt_backup(args.connection, args.backup_dir, args.backup)
        else:
            run(args.connection, args.lat, args.lon, args.alt_m, args.rate_hz)
            return 0
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except (RuntimeError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"fake-gps error: {exc}", flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
