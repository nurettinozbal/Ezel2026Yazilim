"""Regression guard for the canonical field/YKI portability contract."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
YKI = ROOT / "ezel-yazilim_yeni" / "ezel-yazilim" / "arayuz"
MODEL_SHA256 = "e3277e1778ec59e5af64cbeea81292fe97daefa6f0dc7bf1cc26780e24e93138"


class PortableRuntimeContractTests(unittest.TestCase):
    def test_canonical_runtime_has_no_team_machine_literals(self) -> None:
        paths = (
            ROOT / "scripts" / "jetson_env.sh",
            ROOT / "scripts" / "ida_alt_ws",
            ROOT / "scripts" / "start_field_stack.sh",
            ROOT / "scripts" / "field_test.live.env",
            ROOT / "scripts" / "field_test.env.example",
            ROOT / "scripts" / "field_test.smoke.env",
            ROOT / "systemd" / "ida-canonical-field.service.in",
            ROOT / "systemd" / "ida-fake-gps.service.in",
            ROOT / "systemd" / "ida-yki-perception.service.in",
            ROOT / "src" / "ida_bringup" / "launch" / "field_stack.launch.py",
            ROOT / "src" / "ida_vehicle_test" / "ida_vehicle_test" / "producer_node.py",
            YKI / "start.bat",
            YKI / "start.sh",
            YKI / "backend" / "config.py",
        )
        forbidden = (
            "/home/ezelproject",
            "User=ezelproject",
            "Group=ezelproject",
            "192.168.",
            "10.221.",
            "172.20.",
            'set "EZEL_IDA_PORT=COM3"',
            'set "EZEL_IHA_PORT=COM99"',
        )
        for path in paths:
            text = path.read_text(encoding="utf-8")
            for literal in forbidden:
                with self.subTest(path=path, literal=literal):
                    self.assertNotIn(literal, text)

    def test_field_devices_and_yki_are_discovered_or_fail_closed(self) -> None:
        profile = (ROOT / "scripts" / "field_test.live.env").read_text(encoding="utf-8")
        discovery = (ROOT / "scripts" / "field_device_discovery.sh").read_text(encoding="utf-8")
        producer = (
            ROOT / "src" / "ida_vehicle_test" / "ida_vehicle_test" / "producer_node.py"
        ).read_text(encoding="utf-8")
        for setting in (
            "IDA_PIXHAWK_PORT=auto",
            "IDA_LIDAR_PORT=auto",
            "IDA_FIELD_CAMERA_DEVICE=auto",
            "IDA_YKI_DEBUG_WEBSOCKET_URL=auto://yki",
        ):
            self.assertIn(setting, profile)
        self.assertIn("/dev/serial/by-id/*", discovery)
        self.assertIn("/dev/v4l/by-id/*Arducam*video-index0", discovery)
        self.assertIn('declare_parameter("websocket_url", "auto://yki")', producer)

    def test_general_model_is_versioned_and_hash_locked(self) -> None:
        model = ROOT / "models" / "p3_candidates" / "general_yolo11s_20260808.pt"
        self.assertTrue(model.is_file(), "canonical general model is missing")
        digest = hashlib.sha256(model.read_bytes()).hexdigest()
        self.assertEqual(digest, MODEL_SHA256)
        manifest = (ROOT / "models" / "MODEL_MANIFEST.sha256").read_text(
            encoding="utf-8"
        )
        self.assertIn(MODEL_SHA256, manifest)
        start = (ROOT / "scripts" / "start_field_stack.sh").read_text(encoding="utf-8")
        self.assertIn("verify_model_hash", start)
        catalog = json.loads((ROOT / "models" / "model_catalog.json").read_text(encoding="utf-8"))
        artifact = catalog["artifacts"]["general-yolo11s-20260808"]
        self.assertEqual(artifact["sha256"], MODEL_SHA256)
        self.assertEqual(catalog["default_profile"], "general")

    def test_field_tuning_profile_overrides_baseline_configs(self) -> None:
        real = (
            ROOT / "src" / "ida_bringup" / "launch" / "real_vehicle.launch.py"
        ).read_text(encoding="utf-8")
        field = (
            ROOT / "src" / "ida_bringup" / "launch" / "field_stack.launch.py"
        ).read_text(encoding="utf-8")
        env = (ROOT / "scripts" / "field_test.live.env").read_text(encoding="utf-8")
        self.assertIn('"field_profile_path": str(profile_path)', field)
        self.assertGreaterEqual(real.count("field_profile_path"), 8)
        self.assertNotIn("IDA_MAX_SPEED_MPS", env)
        self.assertNotIn("IDA_MAX_YAW_RATE", env)

    def test_manual_yki_backend_is_read_only_without_discovery(self) -> None:
        config = (YKI / "backend" / "config.py").read_text(encoding="utf-8")
        windows = (YKI / "start.bat").read_text(encoding="utf-8")
        windows_discovery = (YKI / "discover_rfd_port.ps1").read_text(encoding="utf-8")
        linux = (YKI / "start.sh").read_text(encoding="utf-8")
        self.assertGreaterEqual(config.count('"COM_DISABLED"'), 2)
        self.assertIn(":discover_vehicle_ports", windows)
        self.assertIn("discover_rfd_port.ps1", windows)
        self.assertIn('"--detect-rfd"', windows)
        self.assertNotIn("Get-CimInstance Win32_SerialPort", windows)
        self.assertIn("HARDWARE\\DEVICEMAP\\SERIALCOMM", windows_discovery)
        self.assertIn("BthModem", windows_discovery)
        self.assertIn("discover_vehicle_ports", linux)
        self.assertIn('EZEL_IHA_ENABLED="${EZEL_IHA_ENABLED:-false}"', linux)

    def test_windows_yki_pairing_is_portable_and_bundled_authority_wins(self) -> None:
        windows = (YKI / "start.bat").read_text(encoding="utf-8")
        pairing = (YKI / "contracts" / "jetson-perception-token.txt").read_text(
            encoding="utf-8"
        ).strip()
        bundled_load = windows.index(
            'set /p EZEL_JETSON_DEBUG_TOKEN=<"%BUNDLED_STREAM_TOKEN_FILE%"'
        )
        runtime_load = windows.index(
            'set /p EZEL_JETSON_DEBUG_TOKEN=<"%STREAM_TOKEN_FILE%"'
        )
        self.assertLess(bundled_load, runtime_load)
        self.assertIn(
            '>"%STREAM_TOKEN_FILE%" echo(!EZEL_JETSON_DEBUG_TOKEN!', windows
        )
        self.assertRegex(pairing, r"^[0-9a-fA-F]{64}$")
        vehicle_pairing = (ROOT / "contracts" / "jetson-perception-token.txt").read_text(
            encoding="utf-8"
        ).strip()
        self.assertEqual(vehicle_pairing, pairing)


if __name__ == "__main__":
    unittest.main()
