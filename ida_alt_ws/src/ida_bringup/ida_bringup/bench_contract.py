"""Fail-closed validation for the restrained yellow-buoy bench profile.

This module is deliberately ROS-independent so the safety envelope can be unit
tested without importing or starting ROS.  It authorizes a launch *shape* only;
it never authorizes ARM or proves that the physical scene is safe.
"""

from __future__ import annotations

import math
from typing import Mapping


BENCH_MAX_SPEED_MPS = 0.25
BENCH_MAX_YAW_RATE_RAD_S = 0.10
BENCH_STUCK_TIMEOUT_S = 30.0
BENCH_SPEED_CEILING_MPS = 0.30
BENCH_YAW_CEILING_RAD_S = 0.15
BENCH_STUCK_TIMEOUT_MIN_S = 10.0
BENCH_STUCK_TIMEOUT_MAX_S = 120.0
BENCH_SAFETY_ACK = "VEHICLE_RESTRAINED_MOTOR_AREA_CLEAR"

# Direct-start P3 decision bench: valid start parkur values.  The autonomy
# node declares these as deployment parameters (never field_profile.yaml
# tuning); the bench launch authorizes the direct-start shape.
BENCH_P3_MIN_START_PARKUR = 1
BENCH_P3_MAX_START_PARKUR = 3

# Bench-only deployment parameters declared by autonomy_node.  They are
# delivered exclusively by the bench launch (bench_p3_decision.launch.py)
# through this contract; they are deliberately NOT part of the field stack
# single-source tuning (field_profile.yaml) and are therefore excluded from
# the field classification test (tools/test_field_stack_service.py).
BENCH_AUTONOMY_DEPLOYMENT_PARAMETERS = (
    "bench_p3_only_enabled",
    "bench_p3_start_parkur",
)


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


def _strict_int(name: str, value: object) -> int:
    """Strict integer: bool is rejected, strings must parse to a whole int."""
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer in [1, 3]")
    if isinstance(value, int):
        parsed = value
    else:
        try:
            parsed = int(str(value).strip())
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{name} must be an integer in [1, 3]") from exc
    if str(value).strip() != str(parsed) or not isinstance(parsed, int):
        raise ValueError(f"{name} must be an integer in [1, 3]")
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
    yaw_rate = _finite_float(
        "bench_max_yaw_rate_rad_s", values.get("bench_max_yaw_rate_rad_s")
    )
    stuck = _finite_float(
        "bench_stuck_timeout_s", values.get("bench_stuck_timeout_s")
    )
    if not 0.0 < speed <= BENCH_SPEED_CEILING_MPS:
        raise ValueError(
            f"bench_max_speed_mps must be in (0, {BENCH_SPEED_CEILING_MPS}]"
        )
    if not 0.0 < yaw_rate <= BENCH_YAW_CEILING_RAD_S:
        raise ValueError(
            f"bench_max_yaw_rate_rad_s must be in (0, {BENCH_YAW_CEILING_RAD_S}]"
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
    normalized["bench_max_yaw_rate_rad_s"] = yaw_rate
    normalized["bench_stuck_timeout_s"] = stuck
    return normalized


def validate_p3_bench_profile(
    values: Mapping[str, object], environment: Mapping[str, str]
) -> dict[str, object]:
    """Validate the direct-start P3 decision bench profile (bench_avoidance shape).

    This profile is a *decision/telemetry* session: the vehicle must start at
    parkur 3 without first satisfying P1/P2 acceptance, but no ARM/motor/mod
    command is ever produced.  The motor path must therefore be closed:
    ``canonical_takeover_enabled``, ``dry_run``, ``motor_command_enabled=false``
    and ``guided_mode_enabled=false`` are mandatory.  A live motor path without
    the exact physical acknowledgement is rejected outright.
    """
    base = validate_bench_profile(values, environment)

    enabled = _strict_bool("bench_p3_only_enabled", values.get("bench_p3_only_enabled"))
    start_parkur = _strict_int(
        "bench_p3_start_parkur", values.get("bench_p3_start_parkur")
    )
    if not BENCH_P3_MIN_START_PARKUR <= start_parkur <= BENCH_P3_MAX_START_PARKUR:
        raise ValueError(
            "bench_p3_start_parkur must be in "
            f"[{BENCH_P3_MIN_START_PARKUR}, {BENCH_P3_MAX_START_PARKUR}]"
        )

    if enabled and not (
        bool(base["dry_run"])
        and not bool(base["motor_command_enabled"])
        and not bool(base["guided_mode_enabled"])
    ):
        raise ValueError(
            "bench_p3_only profile requires dry_run=true, "
            "motor_command_enabled=false and guided_mode_enabled=false"
        )
    if enabled and bool(base["motor_command_enabled"]):
        raise ValueError("P3 bench is a decision/telemetry test; motor command is forbidden")

    base["bench_p3_only_enabled"] = enabled
    base["bench_p3_start_parkur"] = start_parkur
    return base
