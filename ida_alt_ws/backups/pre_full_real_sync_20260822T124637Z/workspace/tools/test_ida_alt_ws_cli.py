from __future__ import annotations

import unittest
import json
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CLI = ROOT / "scripts" / "ida_alt_ws"
LIVE = ROOT / "scripts" / "field_test.live.env"
INSTALLER = ROOT / "scripts" / "install_ida_alt_cli.sh"


class IdaAltCliTests(unittest.TestCase):
    def test_help_is_short_and_lists_expected_tools(self) -> None:
        text = CLI.read_text(encoding="utf-8")
        for command in (
            "start", "stop", "status", "logs", "cam", "model", "map",
            "fusion", "gps", "pulse", "yki", "unit", "field-check",
        ):
            self.assertIn(f"  {command}", text)

    def test_field_check_uses_only_workspace_single_source_profile(self) -> None:
        text = CLI.read_text(encoding="utf-8")
        checker = (ROOT / "tools" / "check_field_profile.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("src/ida_bringup/config/field_profile.yaml", text)
        self.assertIn("validate_field_drive_envelope", checker)
        self.assertIn("validate_field_tuning_coverage", checker)
        self.assertIn("validate_thruster_hardware_profile", checker)
        for forbidden in ("pymavlink", "rclpy", "systemctl", "subprocess"):
            self.assertNotIn(forbidden, checker)

    def test_model_selection_is_catalog_driven_not_env_duplicated(self) -> None:
        text = CLI.read_text(encoding="utf-8")
        start = (ROOT / "scripts" / "start_field_stack.sh").read_text(encoding="utf-8")
        self.assertIn("model import", text)
        self.assertIn("model select", text)
        self.assertIn("models/model_catalog.json", text)
        self.assertIn("class_name_map:=", start)
        self.assertNotIn('MODEL_P1P2="${IDA_MODEL_P1P2', start)

    def test_camera_uses_stable_by_id_path_not_video_zero(self) -> None:
        text = CLI.read_text(encoding="utf-8")
        discovery = (ROOT / "scripts" / "field_device_discovery.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("resolve_field_camera_device", text)
        self.assertIn("/dev/v4l/by-id/*Arducam*video-index0", discovery)
        self.assertIn("resolve_field_serial_device", discovery)
        self.assertIn("/dev/serial/by-id/*", discovery)
        self.assertIn("camera_index=", text)
        self.assertNotIn(
            "for device in /dev/idaws_pixhawk /dev/idaws_lidar /dev/video0", text
        )

    def test_cli_contains_no_arm_or_direct_mavlink_actuation_tool(self) -> None:
        text = CLI.read_text(encoding="utf-8")
        for forbidden in ("rc_test.py", "guided_test.py", "vel_test.py", "MAV_CMD_COMPONENT_ARM_DISARM"):
            self.assertNotIn(forbidden, text)
        self.assertIn("ARM ve hareket icin YKI", text)

    def test_bench_pulse_is_bounded_never_arms_and_always_closes_gate(self) -> None:
        text = CLI.read_text(encoding="utf-8")
        limiter = (ROOT / "src/ida_control/ida_control/command_limiter_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("0.0 < speed <= 0.30", text)
        self.assertIn("0.5 <= duration <= 6.0", text)
        self.assertIn("PROPELLER_AREA_CLEAR", text)
        self.assertIn("trap cleanup_pulse EXIT INT TERM", text)
        self.assertIn("bench_pulse_enabled false", text)
        self.assertIn("bench_pulse_enabled true >/dev/null", text)
        self.assertNotIn("grep -q Successful", text)
        self.assertIn("/control/bench_pulse_request", limiter)
        self.assertIn("time.monotonic()", limiter)
        self.assertNotIn("MAV_CMD_COMPONENT_ARM_DISARM", limiter)

    def test_fake_gps_is_explicit_reversible_and_not_actuating(self) -> None:
        text = CLI.read_text(encoding="utf-8")
        tool = (ROOT / "tools" / "fake_gps.py").read_text(encoding="utf-8")
        unit = (ROOT / "systemd" / "ida-fake-gps.service.in").read_text(encoding="utf-8")
        launch = (
            ROOT / "src" / "ida_bringup" / "launch" / "real_vehicle.launch.py"
        ).read_text(encoding="utf-8")
        for action in ("prepare", "adopt", "start", "status", "stop", "restore"):
            self.assertIn(action, text)
        self.assertIn("gps_input_send", tool)
        self.assertIn('FAKE_GPS_CONNECTION="udp:127.0.0.1:14542"', text)
        self.assertIn("fake_gps.py --connection udp:127.0.0.1:14542 run", unit)
        self.assertIn('"--endpoint-port", "14542"', launch)
        self.assertIn("vehicle is armed; parameter operation rejected", tool)
        for forbidden in ("MAV_CMD_COMPONENT_ARM_DISARM", "set_mode_send", "rc_channels_override_send"):
            self.assertNotIn(forbidden, tool)
        self.assertNotIn("[Install]", unit.replace("# Intentionally no [Install]", ""))

    def test_installed_cli_can_resolve_root_protected_workspace_env(self) -> None:
        text = CLI.read_text(encoding="utf-8")
        self.assertIn("/etc/systemd/system/ida-canonical-field.service", text)
        self.assertIn('$1 == "WorkingDirectory"', text)
        self.assertIn("as_root test -f /etc/ida/field-test.env", text)
        self.assertNotIn("as_root test -s /etc/ida/field-test.local.env", text)
        self.assertIn('systemctl disable --now "$PERCEPTION_SERVICE"', text)
        self.assertNotIn('[[ -f /etc/ida/field-test.env ]]', text)
        self.assertNotIn('[[ -s /etc/ida/field-test.local.env ]]', text)

    def test_live_profile_opens_gates_but_forbids_vehicle_setup(self) -> None:
        values = {}
        for line in LIVE.read_text(encoding="utf-8").splitlines():
            if line and not line.startswith("#") and "=" in line:
                key, value = line.split("=", 1)
                values[key] = value
        self.assertEqual(values["IDA_FIELD_DRY_RUN"], "false")
        self.assertEqual(values["IDA_MAVLINK_ROUTER_ENABLED"], "true")
        self.assertEqual(values["IDA_GUIDED_MODE_ENABLED"], "true")
        self.assertEqual(values["IDA_MOTOR_COMMAND_ENABLED"], "true")
        self.assertEqual(values["IDA_VEHICLE_SETUP_ENABLED"], "false")
        self.assertEqual(values["IDA_FIELD_PHYSICAL_SAFETY_ACK"], "FIELD_OPERATOR_READY_NO_AUTO_ARM")
        self.assertNotIn("IDA_MAX_SPEED_MPS", values)
        profile = (ROOT / "src" / "ida_bringup" / "config" / "field_profile.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("max_speed_mps:", profile)

    def test_start_requires_lidar_and_retires_wifi_yki_stream(self) -> None:
        cli = CLI.read_text(encoding="utf-8")
        installer = INSTALLER.read_text(encoding="utf-8")
        self.assertIn("/sllidar_node", cli)
        self.assertIn("/ida_sllidar_bridge", cli)
        self.assertIn("ros2 topic echo --once /scan", cli)
        self.assertIn("ros2 topic echo --once /perception/lidar/raw_obstacles", cli)
        for node in ("/ida_autonomy", "/ida_command_limiter", "/ida_mavsdk_bridge"):
            self.assertIn(node, cli)
        for topic in ("/autonomy/debug", "/telemetry/state", "/control/cmd_vel_body"):
            self.assertIn(topic, cli)
        self.assertIn("otonomi/komut zinciri eksik", cli)
        self.assertNotIn('systemctl start "$PERCEPTION_SERVICE"', cli)
        self.assertIn('systemctl stop "$PERCEPTION_SERVICE"', cli)
        self.assertIn('systemctl disable --now "$PERCEPTION_SERVICE"', cli)
        self.assertIn("disable --now ida-yki-perception.service", installer)
        live = LIVE.read_text(encoding="utf-8")
        self.assertIn("IDA_PIXHAWK_PORT=auto", live)
        self.assertIn("IDA_LIDAR_PORT=auto", live)
        self.assertIn("IDA_MODEL_PROFILE=", live)
        self.assertNotIn("IDA_MODEL_P1P2=", live)
        start = (ROOT / "scripts" / "start_field_stack.sh").read_text(encoding="utf-8")
        self.assertIn("models/model_catalog.json", start)
        self.assertIn("verify_model_hash", start)
        self.assertIn("/etc/ida/model-selection", INSTALLER.read_text(encoding="utf-8"))

    def test_runtime_templates_do_not_pin_team_username_or_home(self) -> None:
        runtime_files = (
            ROOT / "scripts" / "jetson_env.sh",
            ROOT / "scripts" / "ida_alt_ws",
            ROOT / "scripts" / "field_test.live.env",
            ROOT / "scripts" / "field_test.env.example",
            ROOT / "scripts" / "field_test.smoke.env",
            ROOT / "systemd" / "ida-canonical-field.service.in",
            ROOT / "systemd" / "ida-fake-gps.service.in",
        )
        for path in runtime_files:
            with self.subTest(path=path):
                contents = path.read_text(encoding="utf-8")
                self.assertNotIn("/home/ezelproject", contents)
                self.assertNotIn("User=ezelproject", contents)
                self.assertNotIn("Group=ezelproject", contents)

    def test_fake_gps_runtime_uses_verified_backup_not_param_stream(self) -> None:
        from tools.fake_gps import _verified_runtime_backup

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "active.json"
            path.write_text(json.dumps({
                "gps1_type_before": 9,
                "gps1_type_temporary": 14,
                "target_system": 1,
            }), encoding="utf-8")
            self.assertEqual(_verified_runtime_backup(path, 1)["gps1_type_temporary"], 14)
            with self.assertRaises(RuntimeError):
                _verified_runtime_backup(path, 2)


if __name__ == "__main__":
    unittest.main()
