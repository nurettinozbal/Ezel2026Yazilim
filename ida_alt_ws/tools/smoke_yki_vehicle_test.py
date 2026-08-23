#!/usr/bin/env python3
"""Passive YKI <-> Jetson vehicle-test WebSocket smoke client.

The browser token is read only from EZEL_WS_TOKEN.  This tool can start one
allow-listed passive test; it has no vehicle command or actuation support.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time


TERMINAL = {"PASS", "FAIL", "CANCELLED"}


async def run(
    url: str,
    test_id: str | None,
    timeout_s: float,
    expectations: dict | None,
) -> int:
    try:
        import websockets
    except ImportError as exc:
        raise RuntimeError("python websockets dependency is missing") from exc

    token = os.environ.get("EZEL_WS_TOKEN", "").strip()
    if not token:
        raise RuntimeError("EZEL_WS_TOKEN must be set")

    deadline = time.monotonic() + timeout_s + 10.0
    manifest_seen = False
    snapshot_seen = False
    authorized = False
    started = False
    run_id = None

    async with websockets.connect(
        url,
        origin="http://localhost:5173",
        max_size=1_048_576,
    ) as websocket:
        await websocket.send(json.dumps({"type": "auth", "token": token}))
        while time.monotonic() < deadline:
            remaining = max(0.1, deadline - time.monotonic())
            raw = await asyncio.wait_for(websocket.recv(), timeout=remaining)
            message = json.loads(raw)
            msg_type = message.get("type")
            data = message.get("data", {})

            if msg_type == "auth":
                authorized = data.get("command_authorized") is True
                if not authorized and data.get("auth_required") is True:
                    continue
            elif msg_type == "debug_snapshot":
                snapshot_seen = True
            elif msg_type == "vehicle_test" and data.get("event") == "manifest":
                manifest_seen = True
            elif msg_type == "vehicle_test":
                if data.get("event") == "pending" and data.get("run_id"):
                    run_id = data["run_id"]
                status = data.get("status")
                terminal_event = data.get("event") in {"result", "cancelled"}
                if terminal_event and status in TERMINAL and data.get("run_id") == run_id:
                    summary = {
                        "authorized": authorized,
                        "manifest_seen": manifest_seen,
                        "snapshot_seen": snapshot_seen,
                        "run_id": run_id,
                        "status": status,
                        "pass": data.get("pass"),
                        "artifact": data.get("artifact"),
                    }
                    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
                    return 0

            if authorized and manifest_seen and not started:
                if test_id is None:
                    print(json.dumps({
                        "authorized": True,
                        "manifest_seen": True,
                        "snapshot_seen": snapshot_seen,
                    }, sort_keys=True))
                    return 0
                await websocket.send(json.dumps({
                    "type": "vehicle_test",
                    "data": {
                        "action": "start",
                        "test_id": test_id,
                        "timeout_s": timeout_s,
                        "expectations": expectations,
                    },
                }))
                started = True

    raise TimeoutError(f"passive test did not finish; run_id={run_id}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://127.0.0.1:5000/ws")
    parser.add_argument("--test", choices=[
        "comms", "telemetry", "camera_p1p2", "camera_p3", "lidar",
        "fusion_shadow", "autonomy_shadow", "logging",
    ])
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--case", choices=[
        "positive", "negative", "ambiguity", "lidar_only", "camera_only",
    ])
    parser.add_argument("--color", choices=["orange", "yellow", "red", "green", "black"])
    parser.add_argument("--range-m", type=float)
    parser.add_argument("--bearing-deg", type=float)
    parser.add_argument("--range-tolerance-m", type=float, default=0.5)
    parser.add_argument("--bearing-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--expected-id")
    args = parser.parse_args()
    if not 0.5 <= args.timeout <= 300.0:
        parser.error("--timeout must be in [0.5, 300]")
    fixture_profiles = {
        "camera_p1p2": "camera_p1p2_fixture",
        "camera_p3": "camera_p3_fixture",
        "lidar": "lidar_bottle",
        "fusion_shadow": "fusion_buoy",
    }
    expectations = None
    if args.test in fixture_profiles:
        if args.case is None:
            parser.error("fixture tests require --case")
        if args.case == "positive" and (args.range_m is None or args.bearing_deg is None):
            parser.error("positive fixture requires --range-m and --bearing-deg")
        if args.test in {"camera_p1p2", "camera_p3", "fusion_shadow"} and args.case == "positive" and args.color is None:
            parser.error("positive color fixture requires --color")
        expectations = {
            "profile": fixture_profiles[args.test],
            "case": args.case,
            "expected_color": args.color,
            "expected_range_m": args.range_m,
            "expected_bearing_deg": args.bearing_deg,
            "range_tolerance_m": args.range_tolerance_m,
            "bearing_tolerance_deg": args.bearing_tolerance_deg,
            "expected_id": args.expected_id,
        }
    return asyncio.run(run(args.url, args.test, args.timeout, expectations))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"smoke failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(2)
