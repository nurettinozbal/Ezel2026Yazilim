"""Read the measured/provisional field envelope for opt-in simulation runs.

The normal simulator deliberately keeps its known-good ``autonomy.yaml``
defaults.  Field-fidelity runs opt in to ``field_profile.yaml`` and use this
module to derive the physical speed ceiling without copying that value into a
launch file or service script.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml


class SimFieldProfileError(ValueError):
    """Raised when a field profile cannot safely drive the simulator."""


def _parameters(document: dict[str, Any], node: str) -> dict[str, Any]:
    value = document.get(node)
    if not isinstance(value, dict):
        raise SimFieldProfileError(f"{node}.ros__parameters eksik")
    parameters = value.get("ros__parameters")
    if not isinstance(parameters, dict):
        raise SimFieldProfileError(f"{node}.ros__parameters eksik")
    return parameters


def _finite_number(parameters: dict[str, Any], name: str) -> float:
    value = parameters.get(name)
    if isinstance(value, bool):
        raise SimFieldProfileError(f"{name} sonlu sayi olmali")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise SimFieldProfileError(f"{name} sonlu sayi olmali") from exc
    if not math.isfinite(number):
        raise SimFieldProfileError(f"{name} sonlu sayi olmali")
    return number


def load_sim_field_fidelity(path: str | Path) -> dict[str, float]:
    """Return the bounded simulation envelope derived from one field profile.

    This function never writes Pixhawk/SITL parameters.  It only validates the
    profile and returns values consumed by ROS launch parameters.
    """

    profile_path = Path(path).expanduser().resolve()
    try:
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise SimFieldProfileError(
            f"saha profili okunamadi: {profile_path}"
        ) from exc
    if not isinstance(document, dict):
        raise SimFieldProfileError("saha profili YAML nesnesi olmali")

    field = _parameters(document, "ida_field_envelope")
    thruster = _parameters(document, "ida_thruster_envelope")
    autonomy = _parameters(document, "ida_autonomy")
    limiter = _parameters(document, "ida_command_limiter")

    hard_max = _finite_number(field, "hard_max_speed_mps")
    cruise = _finite_number(thruster, "target_actual_cruise_speed_mps")
    actual_max = _finite_number(thruster, "target_actual_max_speed_mps")
    mot_min = _finite_number(thruster, "pixhawk_mot_thr_min_pct")
    mot_max = _finite_number(thruster, "pixhawk_mot_thr_max_pct")
    slew = _finite_number(thruster, "pixhawk_mot_slewrate_pct_s")
    autonomy_max = _finite_number(autonomy, "max_speed_mps")
    yaw_deg_s = _finite_number(autonomy, "max_yaw_rate_deg_s")
    accel = _finite_number(autonomy, "dwa_accel_max")
    limiter_max = _finite_number(limiter, "max_vx_mps")
    limiter_yaw = _finite_number(limiter, "max_yaw_rate_rad_s")

    if not (0.0 < cruise <= actual_max <= hard_max):
        raise SimFieldProfileError(
            "hiz sirasi 0 < cruise <= actual_max <= hard_max olmali"
        )
    if autonomy_max < actual_max or limiter_max < actual_max:
        raise SimFieldProfileError(
            "autonomy/limiter tavani hedef gercek hiz tavanini kapsamalidir"
        )
    if not (0.0 <= mot_min < mot_max <= 100.0):
        raise SimFieldProfileError("MOT_THR_MIN/MAX yuzde araligi gecersiz")
    if not (0.0 < slew <= 100.0):
        raise SimFieldProfileError("MOT_SLEWRATE (0,100] araliginda olmali")
    if yaw_deg_s <= 0.0 or limiter_yaw <= 0.0 or accel <= 0.0:
        raise SimFieldProfileError("yaw ve ivme sinirlari pozitif olmali")
    if math.radians(yaw_deg_s) > limiter_yaw + 1e-6:
        raise SimFieldProfileError("autonomy yaw tavani limiter tavanini asiyor")

    return {
        "max_speed_mps": actual_max,
        "cruise_speed_mps": cruise,
        "max_yaw_rate_rad_s": limiter_yaw,
        "dwa_accel_max": accel,
        "mot_thr_min_pct": mot_min,
        "mot_thr_max_pct": mot_max,
        "mot_slewrate_pct_s": slew,
    }
