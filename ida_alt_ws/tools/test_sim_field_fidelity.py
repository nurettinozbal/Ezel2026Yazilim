"""Opt-in field-fidelity simulator profile regressions."""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "src" / "ida_bringup" / "ida_bringup" / "sim_field_fidelity.py"
PROFILE = ROOT / "src" / "ida_bringup" / "config" / "field_profile.yaml"
GAZEBO = ROOT / "src" / "ida_bringup" / "launch" / "sim_gazebo.launch.py"
FULL = ROOT / "src" / "ida_bringup" / "launch" / "sim_full_mission.launch.py"
HELPER = ROOT / "scripts" / "sim_yki_server.sh"


def _module():
    spec = importlib.util.spec_from_file_location("sim_field_fidelity_test", MODULE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_field_fidelity_reads_the_single_field_profile() -> None:
    values = _module().load_sim_field_fidelity(PROFILE)
    assert values["cruise_speed_mps"] == 0.75
    assert values["max_speed_mps"] == 1.0
    assert values["mot_thr_min_pct"] == 0.0
    assert values["mot_thr_max_pct"] == 30.0
    assert values["mot_slewrate_pct_s"] == 20.0


def test_invalid_physical_envelope_is_rejected() -> None:
    module = _module()
    document = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    invalid = copy.deepcopy(document)
    invalid["ida_thruster_envelope"]["ros__parameters"][
        "target_actual_max_speed_mps"
    ] = 2.0
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "invalid.yaml"
        path.write_text(yaml.safe_dump(invalid), encoding="utf-8")
        try:
            module.load_sim_field_fidelity(path)
        except module.SimFieldProfileError:
            pass
        else:
            raise AssertionError("unsafe field envelope must be rejected")


def test_sim_defaults_remain_known_good_and_field_mode_is_explicit() -> None:
    for path in (GAZEBO, FULL):
        text = path.read_text(encoding="utf-8")
        assert '"sim_field_fidelity"' in text
        assert 'default_value="false"' in text
        assert "load_sim_field_fidelity" in text
        assert "field_profile.yaml" in text
        assert "sim_effective_max_speed_mps" in text
        assert "UnlessCondition(sim_field_fidelity)" in text
        assert "IfCondition(sim_field_fidelity)" in text


def test_yki_server_explicitly_selects_real_time_field_fidelity() -> None:
    text = HELPER.read_text(encoding="utf-8")
    assert "speedup:=1" in text
    assert "sim_field_fidelity:=true" in text
    assert "sim_max_speed_mps:=1.0" not in text
