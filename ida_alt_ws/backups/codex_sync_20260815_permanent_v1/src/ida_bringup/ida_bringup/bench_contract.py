"""Fail-closed validation for the restrained yellow-buoy bench profile.

This module is deliberately ROS-independent so the safety envelope can be unit
tested without importing or starting ROS.  It authorizes a launch *shape* only;
it never authorizes ARM or proves that the physical scene is safe.
"""

from __future__ import annotations

import math
from typing import Mapping


BENCH_MAX_SPEED_MPS = 0.25
BENCH_STUCK_TIMEOUT_S = 30.0
BENCH_SPEED_CEILING_MPS = 0.30
BENCH_STUCK_TIMEOUT_MIN_S = 10.0
BENCH_STUCK_TIMEOUT_MAX_S = 120.0
BENCH_SAFETY_ACK = "VEHICLE_RESTRAINED_MOTOR_AREA_CLEAR"


def _strict_bool(name: str, value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value in {"true", "false"}:
        return value == "true"
    raise ValueError(f"{name} must be exactly true or false")


def _finite_float(name: str, value: object) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be a finite number")
    return parsed


def validate_bench_profile(
    values: Mapping[str, object], environment: Mapping[str, str]
) -> dict[str, object]:
    """Validate and normalize the non-negotiable bench launch envelope.

    Motor output remains impossible with the defaults.  Opening it requires all
    ownership/mode gates plus an exact, session-local physical acknowledgement.
    The acknowledgement is only a software interlock and never replaces the
    independent propulsion power cut or the safety observer.
    """

    required_bools = (
        "canonical_takeover_enabled",
        "dry_run",
        "mavlink_router_enabled",
        "vehicle_setup_enabled",
        "guided_mode_enabled",
        "motor_command_enabled",
        "fusion_model_loaded",
        "camera_calibrated",
        "lidar_calibrated",
        "extrinsics_calibrated",
    )
    normalized: dict[str, object] = {
        name: _strict_bool(name, values.get(name)) for name in required_bools
    }
    speed = _finite_float("bench_max_speed_mps", values.get("bench_max_speed_mps"))
    stuck = _finite_float(
        "bench_stuck_timeout_s", values.get("bench_stuck_timeout_s")
    )
    if not 0.0 < speed <= BENCH_SPEED_CEILING_MPS:
        raise ValueError(
            f"bench_max_speed_mps must be in (0, {BENCH_SPEED_CEILING_MPS}]"
        )
    if not BENCH_STUCK_TIMEOUT_MIN_S <= stuck <= BENCH_STUCK_TIMEOUT_MAX_S:
        raise ValueError(
            "bench_stuck_timeout_s must be between "
            f"{BENCH_STUCK_TIMEOUT_MIN_S} and {BENCH_STUCK_TIMEOUT_MAX_S}"
        )

    takeover = bool(normalized["canonical_takeover_enabled"])
    setup = bool(normalized["vehicle_setup_enabled"])
    guided = bool(normalized["guided_mode_enabled"])
    motor = bool(normalized["motor_command_enabled"])
    router = bool(normalized["mavlink_router_enabled"])
    dry_run = bool(normalized["dry_run"])
    readiness = (
        bool(normalized["fusion_model_loaded"]),
        bool(normalized["camera_calibrated"]),
        bool(normalized["lidar_calibrated"]),
        bool(normalized["extrinsics_calibrated"]),
    )

    if (setup or guided or motor) and not takeover:
        raise ValueError("vehicle/guided/motor gates require canonical takeover")
    if motor and not guided:
        raise ValueError("motor command gate requires guided mode gate")
    if motor and not router:
        raise ValueError("motor command gate requires the single-owner MAVLink router")
    if motor and dry_run:
        raise ValueError("live motor-direction test requires dry_run=false")
    if motor and not all(readiness):
        raise ValueError(
            "motor-direction test requires model, camera, lidar and extrinsics readiness"
        )
    if motor and environment.get("IDA_BENCH_PHYSICAL_SAFETY_ACK") != BENCH_SAFETY_ACK:
        raise ValueError(
            "motor command gate requires the exact restrained-bench safety acknowledgement"
        )

    normalized["bench_max_speed_mps"] = speed
    normalized["bench_stuck_timeout_s"] = stuck
    return normalized
