"""Isolated YKI simulator profile regressions."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
YKI = ROOT / "ezel-yazilim_yeni" / "ezel-yazilim" / "arayuz"
CONFIG_DIR = YKI / "backend"
SIM_LAUNCHER = YKI / "start_sim.bat"
FIELD_LAUNCHER = YKI / "start.bat"
REMOTE_HELPER = ROOT / "scripts" / "sim_yki_server.sh"


def _import_config(overrides: dict[str, str]) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    for key in (
        "EZEL_DEPLOYMENT_PROFILE", "EZEL_IDA_PORT", "EZEL_IHA_ENABLED",
        "EZEL_SIM_MAVLINK_PORT",
    ):
        environment.pop(key, None)
    environment.update(overrides)
    return subprocess.run(
        [sys.executable, "-c", "import config; print(config.DEPLOYMENT_PROFILE, config.IDA_PORT)"],
        cwd=CONFIG_DIR,
        env=environment,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )


def test_field_profile_defaults_are_unchanged() -> None:
    result = _import_config({})
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "FIELD COM_DISABLED"


def test_sim_profile_accepts_only_dedicated_local_udpin() -> None:
    accepted = _import_config({
        "EZEL_DEPLOYMENT_PROFILE": "SIM",
        "EZEL_IDA_PORT": "udpin:0.0.0.0:14550",
        "EZEL_IHA_ENABLED": "false",
    })
    assert accepted.returncode == 0, accepted.stderr
    assert accepted.stdout.strip() == "SIM udpin:0.0.0.0:14550"

    for forbidden in ("COM3", "/dev/ttyUSB0", "udp:192.168.1.30:14550"):
        rejected = _import_config({
            "EZEL_DEPLOYMENT_PROFILE": "SIM",
            "EZEL_IDA_PORT": forbidden,
            "EZEL_IHA_ENABLED": "false",
        })
        assert rejected.returncode != 0
        assert "refuses serial/remote endpoints" in rejected.stderr


def test_sim_launcher_uses_separate_ports_and_skips_rfd_discovery() -> None:
    sim = SIM_LAUNCHER.read_text(encoding="utf-8")
    field = FIELD_LAUNCHER.read_text(encoding="utf-8")
    helper = REMOTE_HELPER.read_text(encoding="utf-8")

    assert 'set "EZEL_DEPLOYMENT_PROFILE=SIM"' in sim
    assert 'set "EZEL_IDA_PORT=udpin:0.0.0.0:%EZEL_SIM_MAVLINK_PORT%"' in sim
    assert 'set "BACKEND_PORT=5001"' in sim
    assert 'set "FRONTEND_PORT=5174"' in sim
    assert "discover_rfd_port" not in sim
    assert 'if not defined BACKEND_PORT set "BACKEND_PORT=5000"' in field
    assert 'if not defined FRONTEND_PORT set "FRONTEND_PORT=5173"' in field
    assert "auto_mission:=false" in helper
    assert "mission_raw_enabled:=true" in helper
    assert "target_color_param:=SCR_USER4" in helper
    assert "speedup:=1" in helper
    assert "sim_field_fidelity:=true" in helper
    # YKİ sim kabul koşusu eski doğrudan canonical perception yolunu değil,
    # production ile aynı raw kamera/lidar -> fusion -> canonical tek-yazar
    # topolojisini atomik olarak açmalıdır.
    assert "sensor_fusion_enabled:=true" in helper
    assert "sensor_fusion_shadow_mode:=false" in helper
    assert "perception_sim_publish_canonical:=false" in helper
    assert "perception_sim_lidar_range_m:=35.0" in helper


def test_frontend_has_unmistakable_sim_banner() -> None:
    layout = (YKI / "src" / "shared" / "layout" / "MainLayout.jsx").read_text(
        encoding="utf-8"
    )
    assert "VITE_DEPLOYMENT_PROFILE" in layout
    assert "GERÇEK ARAÇ/RFD DEVRE DIŞI" in layout
    assert 'data-testid="sim-profile-banner"' in layout
