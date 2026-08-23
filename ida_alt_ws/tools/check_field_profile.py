#!/usr/bin/env python3
"""Read-only validation for the single canonical field tuning profile."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from ida_bringup.field_stack_contract import (
    validate_field_drive_envelope,
    validate_field_tuning_coverage,
    validate_thruster_hardware_profile,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("profile")
    args = parser.parse_args()
    path = Path(args.profile).expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f"field profile bulunamadi: {path}")
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        drive = validate_field_drive_envelope(raw)
        validate_field_tuning_coverage(raw)
        hardware = validate_thruster_hardware_profile(raw)
    except Exception as exc:
        raise SystemExit(f"FIELD PROFILE GECERSIZ: {exc}") from exc
    print("FIELD PROFILE OK")
    print(json.dumps({
        "path": str(path),
        "hard_max_speed_mps": drive["hard_max_speed_mps"],
        "max_speed_mps": drive["max_speed_mps"],
        "min_navigation_command_mps": drive["min_navigation_command_mps"],
        "dwa_slow_vx": drive["dwa_slow_vx"],
        "dwa_recovery_vx": drive["dwa_recovery_vx"],
        "p3_approach_speed_mps": drive["p3_approach_speed_mps"],
        "p3_engage_speed_mps": drive["p3_engage_speed_mps"],
        "pixhawk_mot_thr_max_pct": hardware["pixhawk_mot_thr_max_pct"],
        "pixhawk_mot_slewrate_pct_s": hardware["pixhawk_mot_slewrate_pct_s"],
    }, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
