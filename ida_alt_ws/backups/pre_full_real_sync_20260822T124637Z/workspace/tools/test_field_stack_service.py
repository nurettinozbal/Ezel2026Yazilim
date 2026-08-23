"""Safety and wiring tests for the manual canonical field service."""

from pathlib import Path
import re
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "ida_bringup"))

from ida_bringup.field_stack_contract import (  # noqa: E402
    FIELD_ABSOLUTE_MAX_SPEED_MPS,
    FIELD_DEPLOYMENT_PARAMETERS,
    FIELD_SAFETY_ACK,
    FIELD_TUNING_PARAMETERS,
    validate_field_drive_envelope,
    validate_thruster_hardware_profile,
    validate_field_launch_tuning,
    validate_field_tuning_coverage,
    validate_field_profile,
)
from ida_bringup.bench_contract import (  # noqa: E402
    BENCH_AUTONOMY_DEPLOYMENT_PARAMETERS,
)


FIELD_LAUNCH = ROOT / "src" / "ida_bringup" / "launch" / "field_stack.launch.py"
REAL_LAUNCH = ROOT / "src" / "ida_bringup" / "launch" / "real_vehicle.launch.py"
START = ROOT / "scripts" / "start_field_stack.sh"
CLI = ROOT / "scripts" / "ida_alt_ws"
INSTALL = ROOT / "scripts" / "install_field_service.sh"
DEPLOY = ROOT / "tools" / "deploy_field_patch_jetson.sh"
UNIT = ROOT / "systemd" / "ida-canonical-field.service.in"
ENV = ROOT / "scripts" / "field_test.env.example"
FIELD_PROFILE = ROOT / "src" / "ida_bringup" / "config" / "field_profile.yaml"


def load_field_drive_profile_for_test():
    """Read the small numeric field envelope without adding a PyYAML test dependency."""
    text = FIELD_PROFILE.read_text(encoding="utf-8")
    envelope = {}
    autonomy = {}
    section = None
    for raw_line in text.splitlines():
        if raw_line == "ida_field_envelope:":
            section = "envelope"
            continue
        if raw_line == "ida_autonomy:":
            section = "autonomy"
            continue
        if raw_line and not raw_line.startswith((" ", "#")):
            section = None
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, value = (part.strip() for part in stripped.split(":", 1))
        try:
            numeric = float(value)
        except ValueError:
            continue
        if section == "envelope" and raw_line.startswith("  "):
            envelope[key] = numeric
        elif section == "autonomy" and raw_line.startswith("    "):
            autonomy[key] = numeric
    return {
        "ida_field_envelope": envelope,
        "ida_autonomy": {"ros__parameters": autonomy},
    }


def profile(**updates):
    values = {
        "canonical_takeover_enabled": "false",
        "dry_run": "true",
        "mavlink_router_enabled": "false",
        "vehicle_setup_enabled": "false",
        "guided_mode_enabled": "false",
        "motor_command_enabled": "false",
        "fusion_model_loaded": "false",
        "camera_calibrated": "false",
        "lidar_calibrated": "false",
        "extrinsics_calibrated": "false",
        "field_camera_driver_enabled": "true",
        "lidar_enabled": "true",
        "single_general_model_enabled": "true",
        "yki_debug_enabled": "false",
        "yki_debug_websocket_url": "ws://127.0.0.1:5000/ws/jetson-debug",
        "max_speed_mps": "0.6",
        "max_yaw_rate_rad_s": "0.785398163",
    }
    values.update(updates)
    return values


class FieldStackContractTests(unittest.TestCase):
    def test_linux_install_and_deploy_restore_executable_bits(self):
        install = INSTALL.read_text(encoding="utf-8")
        deploy = DEPLOY.read_text(encoding="utf-8")
        for script in (
            "start_field_stack.sh",
            "ida_alt_ws",
            "install_field_service.sh",
            "field_device_discovery.sh",
        ):
            self.assertIn(script, install)
            self.assertIn(script, deploy)
        self.assertIn("chmod 0755", install)
        self.assertIn("disable --now ida-yki-perception.service", install)
        self.assertIn("chmod 0755", deploy)

    def test_defaults_are_non_actuating_and_vehicle_setup_is_forbidden(self):
        result = validate_field_profile(profile(), {})
        self.assertTrue(result["dry_run"])
        self.assertFalse(result["motor_command_enabled"])
        with self.assertRaises(ValueError):
            validate_field_profile(profile(vehicle_setup_enabled="true"), {})

    def test_field_speed_ceiling_accepts_sea_profile_only(self):
        self.assertEqual(
            validate_field_profile(profile(max_speed_mps="1.6"), {})["max_speed_mps"],
            1.6,
        )
        with self.assertRaises(ValueError):
            validate_field_profile(profile(max_speed_mps="1.61"), {})

    def test_field_yaml_uses_one_enforced_drive_envelope(self):
        import yaml

        raw_profile = yaml.safe_load(FIELD_PROFILE.read_text(encoding="utf-8"))
        for node_name, node_parameters in raw_profile.items():
            with self.subTest(node=node_name):
                self.assertIsInstance(node_parameters, dict)
                self.assertIn("ros__parameters", node_parameters)
        field_profile = load_field_drive_profile_for_test()
        result = validate_field_drive_envelope(field_profile)
        self.assertEqual(result["hard_max_speed_mps"], FIELD_ABSOLUTE_MAX_SPEED_MPS)
        self.assertLessEqual(result["max_speed_mps"], result["hard_max_speed_mps"])
        self.assertLessEqual(
            result["min_navigation_command_mps"], result["max_speed_mps"]
        )
        for name in (
            "dwa_recovery_vx", "dwa_slow_vx", "dwa_min_drive_vx",
            "wrong_avoid_speed_mps", "p3_lock_min_speed_mps",
            "p3_approach_speed_mps",
            "p3_engage_speed_mps",
        ):
            self.assertGreaterEqual(result[name], result["min_navigation_command_mps"])
            self.assertLessEqual(result[name], result["max_speed_mps"])

        hardware = validate_thruster_hardware_profile(raw_profile)
        self.assertEqual(hardware["left_servo_channel"], 9)
        self.assertEqual(hardware["right_servo_channel"], 11)
        self.assertFalse(hardware["servo_snapshot_verified"])
        self.assertLess(hardware["servo_min_pwm"], hardware["servo_trim_pwm"])
        self.assertLess(hardware["servo_trim_pwm"], hardware["servo_max_pwm"])
        self.assertEqual(hardware["required_left_servo_function"], 73)
        self.assertEqual(hardware["required_right_servo_function"], 74)
        self.assertEqual(hardware["required_frame_class"], 2)
        self.assertLessEqual(
            hardware["continuous_pwm_limit"], hardware["burst_pwm_limit"]
        )
        self.assertLessEqual(hardware["burst_pwm_limit"], hardware["servo_max_pwm"])
        self.assertFalse(hardware["autonomy_burst_enabled"])

    def test_field_drive_envelope_rejects_ineffective_or_over_limit_values(self):
        import copy

        original = load_field_drive_profile_for_test()
        minimum = original["ida_field_envelope"]["min_navigation_command_mps"]
        maximum = original["ida_field_envelope"]["hard_max_speed_mps"]
        for key, value in (
            ("wrong_avoid_speed_mps", minimum - 0.01),
            ("p3_engage_speed_mps", minimum - 0.01),
            ("dwa_recovery_vx", maximum + 0.01),
        ):
            with self.subTest(key=key):
                invalid = copy.deepcopy(original)
                invalid["ida_autonomy"]["ros__parameters"][key] = value
                with self.assertRaises(ValueError):
                    validate_field_drive_envelope(invalid)
        invalid = copy.deepcopy(original)
        invalid["ida_field_envelope"]["hard_max_speed_mps"] = (
            FIELD_ABSOLUTE_MAX_SPEED_MPS + 0.01
        )
        with self.assertRaises(ValueError):
            validate_field_drive_envelope(invalid)

    def test_live_motor_path_requires_every_gate_and_exact_ack(self):
        live = profile(
            canonical_takeover_enabled="true",
            dry_run="false",
            mavlink_router_enabled="true",
            guided_mode_enabled="true",
            motor_command_enabled="true",
            fusion_model_loaded="true",
            camera_calibrated="true",
            lidar_calibrated="true",
            extrinsics_calibrated="true",
        )
        with self.assertRaises(ValueError):
            validate_field_profile(live, {})
        accepted = validate_field_profile(
            live, {"IDA_FIELD_PHYSICAL_SAFETY_ACK": FIELD_SAFETY_ACK}
        )
        self.assertTrue(accepted["motor_command_enabled"])
        for key in (
            "canonical_takeover_enabled", "mavlink_router_enabled",
            "guided_mode_enabled", "fusion_model_loaded", "camera_calibrated",
            "lidar_calibrated", "extrinsics_calibrated",
        ):
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_field_profile({**live, key: "false"}, {
                    "IDA_FIELD_PHYSICAL_SAFETY_ACK": FIELD_SAFETY_ACK
                })

    def test_launch_uses_one_general_inference_and_role_filters(self):
        field = FIELD_LAUNCH.read_text(encoding="utf-8")
        real = REAL_LAUNCH.read_text(encoding="utf-8")
        env = ENV.read_text(encoding="utf-8")
        self.assertIn('"single_general_model_enabled": "true"', field)
        self.assertIn('"secondary_output_topic": ParameterValue(', real)
        self.assertIn('"secondary_allowed_colors": allowed_colors_p3', real)
        self.assertIn("single_general_model_enabled", real)
        self.assertIn("IDA_MODEL_PROFILE=", env)
        self.assertNotIn("IDA_MODEL_P1P2=", env)
        self.assertIn("models/model_catalog.json", START.read_text(encoding="utf-8"))
        self.assertIn("black,green,orange,red,yellow", field)

    def test_yki_debug_bridge_is_explicit_passive_and_token_gated(self):
        field = FIELD_LAUNCH.read_text(encoding="utf-8")
        real = REAL_LAUNCH.read_text(encoding="utf-8")
        start = START.read_text(encoding="utf-8")
        self.assertIn('executable="vehicle_test_producer"', real)
        self.assertIn('"yki_debug_enabled": "false"', field)
        self.assertIn("IDA_YKI_DEBUG_ENABLED", start)
        with self.assertRaises(ValueError):
            validate_field_profile(profile(yki_debug_enabled="true"), {})
        accepted = validate_field_profile(
            profile(yki_debug_enabled="true"),
            {"EZEL_JETSON_DEBUG_TOKEN": "separate-debug-token"},
        )
        self.assertTrue(accepted["yki_debug_enabled"])

    def test_yki_perception_wifi_stream_is_retired(self):
        configure = (ROOT / "scripts" / "configure_yki_perception_stream.sh").read_text(encoding="utf-8")
        self.assertFalse((ROOT / "systemd" / "ida-yki-perception.service.in").exists())
        self.assertIn("disable --now ida-yki-perception.service", configure)
        self.assertIn("rm -f /etc/systemd/system/ida-yki-perception.service", configure)
        self.assertNotIn("vehicle_test_producer", configure)

    def test_lidar_nodes_respawn_and_field_profile_is_single_tuning_source(self):
        real = REAL_LAUNCH.read_text(encoding="utf-8")
        field = FIELD_LAUNCH.read_text(encoding="utf-8")
        profile_text = (
            ROOT / "src" / "ida_bringup" / "config" / "field_profile.yaml"
        ).read_text(encoding="utf-8")
        self.assertGreaterEqual(real.count("respawn=True"), 2)
        self.assertIn('"field_profile_path": str(profile_path)', field)
        self.assertIn("profile_max_speed", field)
        self.assertIn("dwa_align_before_drive_deg:", profile_text)
        self.assertIn("p3_search_yaw_rate_deg_s:", profile_text)
        self.assertIn("p3_search_scan_degrees:", profile_text)
        self.assertIn("p3_search_reposition_m:", profile_text)
        self.assertIn("p3_search_transit_speed_mps:", profile_text)
        self.assertIn("p3_search_anchor_acceptance_m:", profile_text)
        self.assertNotIn("p3_search_rotations_per_cycle:", profile_text)
        self.assertNotIn("p3_staging_advance_s:", profile_text)
        self.assertIn("IDA_FIELD_PROFILE_PATH", field)
        self.assertIn("IDA_FIELD_PROFILE_PATH", START.read_text(encoding="utf-8"))

    def test_p3_anchor_parameters_exist_in_field_and_sim_fallback(self):
        required = {
            "p3_search_scan_degrees",
            "p3_search_reposition_m",
            "p3_search_transit_speed_mps",
            "p3_search_reposition_arrival_m",
            "p3_search_anchor_acceptance_m",
        }
        for path in (
            FIELD_PROFILE,
            ROOT / "src" / "ida_bringup" / "config" / "autonomy.yaml",
        ):
            text = path.read_text(encoding="utf-8")
            for key in required:
                self.assertRegex(text, rf"(?m)^\s+{key}:\s*")
            self.assertNotIn("p3_search_rotations_per_cycle:", text)
            self.assertNotIn("p3_staging_advance_s:", text)

    def test_p2_uses_lidar_primary_camera_secondary_policy(self):
        import yaml

        for path in (
            FIELD_PROFILE,
            ROOT / "src" / "ida_bringup" / "config" / "autonomy.yaml",
        ):
            profile = yaml.safe_load(path.read_text(encoding="utf-8"))
            params = profile["ida_autonomy"]["ros__parameters"]
            self.assertEqual(params["corridor_bias_parkurs"], [1])
            self.assertLess(params["dwa_w_avoid"], params["dwa_w_obstacle"])
        costmap_text = (
            ROOT / "src" / "ida_planning" / "ida_planning" / "costmap.py"
        ).read_text(encoding="utf-8")
        self.assertIn("det_confidence < 0.5", costmap_text)
        dwa_text = (
            ROOT / "src" / "ida_planning" / "ida_planning" / "dwa.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"avoid":', dwa_text)

    def test_complete_field_tuning_profile_is_required(self):
        import yaml

        profile = yaml.safe_load(FIELD_PROFILE.read_text(encoding="utf-8"))
        covered = validate_field_tuning_coverage(profile)
        self.assertIn("ida_autonomy", covered)
        self.assertIn("ida_command_limiter", covered)
        self.assertIn("ida_mavsdk_bridge", covered)
        self.assertIn("ida_field_camera_driver", covered)
        self.assertIn("sllidar_node", covered)
        self.assertIn("ida_sensor_fusion", covered)

        del profile["ida_autonomy"]["ros__parameters"]["waypoint_threshold_m"]
        with self.assertRaises(ValueError):
            validate_field_tuning_coverage(profile)

    def test_every_live_node_parameter_is_explicitly_classified(self):
        sources = {
            "ida_autonomy": ROOT / "src/ida_autonomy/ida_autonomy/autonomy_node.py",
            "ida_command_limiter": ROOT / "src/ida_control/ida_control/command_limiter_node.py",
            "ida_mavsdk_bridge": ROOT / "src/ida_control/ida_control/mavsdk_bridge_node.py",
            "ida_yolo_camera": ROOT / "src/ida_perception/ida_perception/yolo_camera_node.py",
            "ida_yolo_camera_p3": ROOT / "src/ida_perception/ida_perception/yolo_camera_node.py",
            "ida_sllidar_bridge": ROOT / "src/ida_perception/ida_perception/sllidar_bridge_node.py",
            "ida_sensor_fusion": ROOT / "src/ida_sensor_fusion/ida_sensor_fusion/fusion_node.py",
            "ida_telemetry_logger": ROOT / "src/ida_logging/ida_logging/telemetry_logger_node.py",
            "ida_video_logger": ROOT / "src/ida_logging/ida_logging/video_logger_node.py",
            "ida_map_logger": ROOT / "src/ida_logging/ida_logging/map_logger_node.py",
            "ida_logging_status": ROOT / "src/ida_logging/ida_logging/status_node.py",
        }
        # Bench-only deployment parametreleri field stack'ine ait DEĞİLDİR:
        # yalnız bench launch'ı (bench_p3_decision.launch.py) bu kontrattan
        # verir, field_profile.yaml single-source tuning'ine asla girmez.
        # Exhaustive olma özelliği korunur: bench dışı her yeni node parametresi
        # hâlâ FIELD_TUNING/DEPLOYMENT_PARAMETERS'da sınıflandırılmadan kabul
        # edilmez.
        bench_only = {
            node_name: set(BENCH_AUTONOMY_DEPLOYMENT_PARAMETERS)
            if node_name == "ida_autonomy"
            else set()
            for node_name in sources
        }
        pattern = re.compile(r"declare_parameter\(\s*[\"']([^\"']+)", re.MULTILINE)
        for node_name, source in sources.items():
            declared = set(pattern.findall(source.read_text(encoding="utf-8")))
            classified = (
                set(FIELD_TUNING_PARAMETERS[node_name])
                | set(FIELD_DEPLOYMENT_PARAMETERS[node_name])
                | bench_only[node_name]
            )
            self.assertEqual(declared, classified, node_name)

    def test_field_launch_cannot_override_central_tuning(self):
        import yaml

        profile = yaml.safe_load(FIELD_PROFILE.read_text(encoding="utf-8"))
        autonomy = profile["ida_autonomy"]["ros__parameters"]
        camera = profile["ida_field_camera_driver"]["ros__parameters"]
        lidar = profile["sllidar_node"]["ros__parameters"]
        thruster = validate_thruster_hardware_profile(profile)
        values = {
            "max_speed_mps": str(autonomy["max_speed_mps"]),
            "max_yaw_rate_rad_s": str(
                float(autonomy["max_yaw_rate_deg_s"]) * 3.141592653589793 / 180.0
            ),
            "autonomy_stuck_timeout_s": str(autonomy["stuck_timeout_s"]),
            "field_camera_width": str(camera["image_width"]),
            "field_camera_height": str(camera["image_height"]),
            "field_camera_fps": str(camera["camera_fps"]),
            "camera_fov_deg": str(camera["fov_deg"]),
            "camera_focal_length_px": str(camera["focal_length_px"]),
            "lidar_serial_baudrate": str(lidar["serial_baudrate"]),
            "lidar_scan_mode": str(lidar["scan_mode"]),
            "motor_output_limits_check_enabled": "true",
            "expected_mot_thr_min_pct": str(thruster["pixhawk_mot_thr_min_pct"]),
            "expected_mot_thr_max_pct": str(thruster["pixhawk_mot_thr_max_pct"]),
            "expected_mot_slewrate_pct_s": str(
                thruster["pixhawk_mot_slewrate_pct_s"]
            ),
            "motor_output_limits_poll_s": str(thruster["verification_poll_s"]),
        }
        validate_field_launch_tuning(values, profile)
        values["max_speed_mps"] = str(float(autonomy["max_speed_mps"]) - 0.1)
        with self.assertRaises(ValueError):
            validate_field_launch_tuning(values, profile)
        values["max_speed_mps"] = str(autonomy["max_speed_mps"])
        values["motor_output_limits_check_enabled"] = "false"
        with self.assertRaises(ValueError):
            validate_field_launch_tuning(values, profile)

    def test_live_shell_and_launch_do_not_hide_secondary_tuning_sources(self):
        start = START.read_text(encoding="utf-8")
        env = (ROOT / "scripts/field_test.live.env").read_text(encoding="utf-8")
        real = REAL_LAUNCH.read_text(encoding="utf-8")
        for legacy_env in (
            "IDA_LIDAR_BAUDRATE", "IDA_LIDAR_SCAN_MODE",
            "IDA_FIELD_CAMERA_WIDTH", "IDA_FIELD_CAMERA_HEIGHT",
            "IDA_FIELD_CAMERA_FPS", "IDA_CAMERA_FOV_DEG",
            "IDA_CAMERA_FOCAL_LENGTH_PX",
        ):
            self.assertNotIn(legacy_env, start)
            self.assertNotIn(legacy_env, env)
        for hidden_bridge_override in (
            '"mission_control_poll_hz"', '"mission_control_ros_ack_timeout_s"',
            '"target_color_poll_hz"', '"mission_download_hz"',
            '"param_call_timeout_s"', '"param_max_retries"',
            '"param_consecutive_fail_limit"', '"yki_status_heartbeat_s"',
        ):
            self.assertNotIn(hidden_bridge_override, real)
        self.assertEqual(
            real.count("logging_config, field_profile_path"), 4,
            "all four live logger nodes must receive the central profile last",
        )

    def test_camera_defaults_match_bench_viewer_geometry_and_controls(self):
        field = FIELD_LAUNCH.read_text(encoding="utf-8")
        real = REAL_LAUNCH.read_text(encoding="utf-8")
        env = ENV.read_text(encoding="utf-8")
        start = START.read_text(encoding="utf-8")
        camera_tuning = set(FIELD_TUNING_PARAMETERS["ida_field_camera_driver"])
        self.assertTrue({
            "auto_exposure", "white_balance_automatic", "gain", "saturation",
            "brightness", "contrast", "fov_deg", "focal_length_px",
        }.issubset(camera_tuning))
        self.assertIn('_profile_params(profile, "ida_field_camera_driver")', field)
        self.assertIn("load_camera_control_profile", field)
        self.assertIn("controls_to_ros_parameters", field)
        self.assertIn("IDA_CAMERA_CONTROL_PROFILE", start)
        self.assertIn('--camera-profile "$CAMERA_CONTROL_PROFILE"', CLI.read_text(encoding="utf-8"))
        self.assertIn('"fov_deg": ParameterValue(camera_fov_deg', real)
        self.assertNotIn("IDA_CAMERA_FOV_DEG", env)
        self.assertNotIn("IDA_CAMERA_FOCAL_LENGTH_PX", env)

    def test_service_is_boot_capable_and_never_controls_vehicle_itself(self):
        unit = UNIT.read_text(encoding="utf-8")
        start = START.read_text(encoding="utf-8")
        installer = INSTALL.read_text(encoding="utf-8")
        self.assertIn("Restart=on-failure", unit)
        self.assertIn("RestartSec=10", unit)
        self.assertIn("[Install]", unit)
        self.assertIn("WantedBy=multi-user.target", unit)
        self.assertNotIn("network-online.target", unit)
        self.assertNotIn("systemctl start", installer)
        self.assertNotIn("\nsudo systemctl enable", installer)
        self.assertNotIn("\nsystemctl enable", installer)
        self.assertNotIn("systemctl stop idaws", start)
        self.assertIn("systemctl is-active --quiet idaws.service", start)
        for forbidden in ("ARM_IDA", "START_IDA_MISSION", "ros2 service call", "pymavlink"):
            self.assertNotIn(forbidden, start)

    def test_service_preflight_owns_camera_lidar_and_pixhawk_exclusively(self):
        start = START.read_text(encoding="utf-8")
        for device in ("$LIDAR_PORT", "$CAMERA_DEVICE", "$PIXHAWK_PORT"):
            self.assertIn(f'lsof "{device}"', start)
        self.assertIn("stack lidar olmadan baslatilacak", start)
        self.assertIn("stack kamera olmadan baslatilacak", start)
        self.assertIn('lidar_enabled:="$LIDAR_ENABLED"', start)
        self.assertIn("vehicle_setup", start.lower())
        self.assertIn("FIELD_OPERATOR_READY_NO_AUTO_ARM", start)
        self.assertIn("tty_mavlink_router", start)

    def test_direct_field_launch_uses_portable_device_contracts(self):
        field = FIELD_LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"field_camera_device": "auto"', field)
        self.assertIn('"pixhawk_serial_port": "auto"', field)
        self.assertIn('"lidar_serial_port": "auto"', field)
        self.assertIn("_resolve_serial_devices", field)
        self.assertIn("*Arducam*video-index0", field)


if __name__ == "__main__":
    unittest.main(verbosity=2)
