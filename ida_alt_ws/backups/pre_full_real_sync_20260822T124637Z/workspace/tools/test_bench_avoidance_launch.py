"""Unit/static safety tests for the restrained avoidance bench profile."""

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "ida_bringup"))

from ida_bringup.bench_contract import (  # noqa: E402
    BENCH_AUTONOMY_DEPLOYMENT_PARAMETERS,
    BENCH_P3_MAX_START_PARKUR,
    BENCH_P3_MIN_START_PARKUR,
    BENCH_SAFETY_ACK,
    validate_bench_profile,
    validate_p3_bench_profile,
)


BENCH = ROOT / "src" / "ida_bringup" / "launch" / "bench_avoidance.launch.py"
P3_LAUNCH = ROOT / "src" / "ida_bringup" / "launch" / "bench_p3_decision.launch.py"
REAL = ROOT / "src" / "ida_bringup" / "launch" / "real_vehicle.launch.py"
START = ROOT / "scripts" / "start_real.sh"
BENCH_START = ROOT / "scripts" / "start_bench_avoidance.sh"
PERCEPTION = ROOT / "src" / "ida_bringup" / "config" / "perception.yaml"


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
        "bench_max_speed_mps": "0.25",
        "bench_max_yaw_rate_rad_s": "0.10",
        "bench_stuck_timeout_s": "30.0",
    }
    values.update(updates)
    return values


class BenchContractTests(unittest.TestCase):
    def test_default_profile_is_dry_run_and_non_actuating(self):
        normalized = validate_bench_profile(profile(), {})
        self.assertTrue(normalized["dry_run"])
        self.assertFalse(normalized["motor_command_enabled"])
        self.assertEqual(normalized["bench_max_speed_mps"], 0.25)
        self.assertEqual(normalized["bench_max_yaw_rate_rad_s"], 0.10)
        self.assertEqual(normalized["bench_stuck_timeout_s"], 30.0)

    def test_live_motor_path_requires_every_gate_and_exact_physical_ack(self):
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
            validate_bench_profile(live, {})
        normalized = validate_bench_profile(
            live, {"IDA_BENCH_PHYSICAL_SAFETY_ACK": BENCH_SAFETY_ACK}
        )
        self.assertTrue(normalized["motor_command_enabled"])

    def test_motor_path_rejects_missing_takeover_guided_router_or_live_mode(self):
        base = profile(
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
        env = {"IDA_BENCH_PHYSICAL_SAFETY_ACK": BENCH_SAFETY_ACK}
        for key, value in (
            ("canonical_takeover_enabled", "false"),
            ("guided_mode_enabled", "false"),
            ("mavlink_router_enabled", "false"),
            ("dry_run", "true"),
        ):
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    validate_bench_profile({**base, key: value}, env)

    def test_motor_path_rejects_any_missing_sensor_readiness_gate(self):
        base = profile(
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
        env = {"IDA_BENCH_PHYSICAL_SAFETY_ACK": BENCH_SAFETY_ACK}
        for key in (
            "fusion_model_loaded",
            "camera_calibrated",
            "lidar_calibrated",
            "extrinsics_calibrated",
        ):
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    validate_bench_profile({**base, key: "false"}, env)

    def test_speed_timeout_and_boolean_inputs_are_strict(self):
        for updates in (
            {"bench_max_speed_mps": "0"},
            {"bench_max_speed_mps": "0.31"},
            {"bench_max_speed_mps": "nan"},
            {"bench_max_yaw_rate_rad_s": "0"},
            {"bench_max_yaw_rate_rad_s": "0.151"},
            {"bench_max_yaw_rate_rad_s": "nan"},
            {"bench_stuck_timeout_s": "9.99"},
            {"bench_stuck_timeout_s": "121"},
            {"bench_stuck_timeout_s": "inf"},
            {"dry_run": "TRUE"},
            {"motor_command_enabled": 1},
        ):
            with self.subTest(updates=updates):
                with self.assertRaises(ValueError):
                    validate_bench_profile(profile(**updates), {})

    def test_launch_defaults_are_safe_and_field_profile_is_only_overridden(self):
        bench = BENCH.read_text(encoding="utf-8")
        real = REAL.read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument("dry_run", default_value="true")', bench)
        self.assertIn(
            'DeclareLaunchArgument("motor_command_enabled", default_value="false")',
            bench,
        )
        self.assertIn('"autonomy_max_speed_mps": LaunchConfiguration("bench_max_speed_mps")', bench)
        self.assertIn('"guided_max_forward_mps": LaunchConfiguration("bench_max_speed_mps")', bench)
        self.assertIn('"guided_max_reverse_mps": "0.0"', bench)
        self.assertIn('"guided_max_yaw_rate_rad_s": LaunchConfiguration(', bench)
        self.assertIn('DeclareLaunchArgument("autonomy_max_speed_mps", default_value="1.6")', real)
        self.assertIn('"max_vx_mps": ParameterValue(', real)
        self.assertIn('autonomy_max_speed_mps, value_type=float', real)
        self.assertIn('DeclareLaunchArgument("autonomy_stuck_timeout_s", default_value="3.0")', real)
        self.assertIn('"stuck_timeout_s": ParameterValue(', real)

    def test_bench_owns_an_explicit_camera_pipeline_only_when_enabled(self):
        bench = BENCH.read_text(encoding="utf-8")
        self.assertIn(
            'DeclareLaunchArgument("bench_camera_driver_enabled", default_value="true")',
            bench,
        )
        self.assertIn('package="v4l2_camera"', bench)
        self.assertIn(
            'DeclareLaunchArgument("camera_topic_type", default_value="raw")',
            bench,
        )
        self.assertNotIn('package="image_transport"', bench)
        self.assertIn('LaunchConfiguration("canonical_takeover_enabled")', bench)

    def test_real_vehicle_uses_repo_tty_single_owner_and_official_s2_defaults(self):
        bench = BENCH.read_text(encoding="utf-8")
        real = REAL.read_text(encoding="utf-8")
        self.assertIn('"tty_mavlink_router"', real)
        self.assertNotIn('"mavlink-routerd"', real)
        self.assertIn('FindPackagePrefix("ida_control")', real)
        self.assertIn('ExecuteProcess(', real)
        self.assertIn(
            'DeclareLaunchArgument("pixhawk_serial_port", default_value="")',
            real,
        )
        self.assertIn(
            'DeclareLaunchArgument("lidar_serial_port", default_value="")',
            real,
        )
        self.assertIn(
            'DeclareLaunchArgument("lidar_serial_baudrate", default_value="1000000")',
            real,
        )
        self.assertIn(
            'DeclareLaunchArgument("lidar_scan_mode", default_value="DenseBoost")',
            real,
        )
        self.assertIn('get_package_prefix("ida_control")', bench)
        self.assertIn('router.is_file()', bench)
        self.assertIn('Path(pixhawk).exists()', bench)
        self.assertIn('Path(lidar).exists()', bench)

    def test_installed_s2_calibration_is_persistent(self):
        text = PERCEPTION.read_text(encoding="utf-8")
        self.assertIn("angle_offset_deg: 180.0", text)
        self.assertIn("mirror_scan: true", text)
        self.assertIn("sensor_forward_offset_m: 0.57", text)

    def test_real_start_script_uses_current_packed_metadata_mailbox_names(self):
        text = START.read_text(encoding="utf-8")
        self.assertIn('mission_counts_param:="$MISSION_COUNTS_PARAM"', text)
        self.assertIn('mission_control_param:="$MISSION_CONTROL_PARAM"', text)
        self.assertNotIn("mission_p1_count_param:=", text)
        self.assertNotIn("mission_p2_count_param:=", text)

    def test_bench_start_script_preserves_explicit_motor_interlocks(self):
        text = BENCH_START.read_text(encoding="utf-8")
        self.assertIn("IDA_BENCH_PHYSICAL_SAFETY_ACK", text)
        self.assertIn("VEHICLE_RESTRAINED_MOTOR_AREA_CLEAR", text)
        self.assertIn('IDA_BENCH_DRY_RUN:-true', text)
        self.assertIn('IDA_MOTOR_COMMAND_ENABLED:-false', text)
        self.assertIn('bench_stuck_timeout_s:="$STUCK_TIMEOUT"', text)
        self.assertIn('bench_max_speed_mps:="$MAX_SPEED"', text)
        self.assertIn('bench_max_yaw_rate_rad_s:="$MAX_YAW_RATE"', text)
        self.assertIn(
            'class_names:="${IDA_MODEL_CLASS_NAMES:-orange,yellow}"', text
        )
        self.assertNotIn("ros2 service call", text)
        self.assertNotIn("ARM_IDA", text)


def p3_profile(**updates):
    """Restrained decision/telemetry P3 bench shape: motor path fully closed."""
    values = profile(
        bench_p3_only_enabled="true",
        bench_p3_start_parkur="3",
    )
    values.update(updates)
    return values


class P3BenchContractTests(unittest.TestCase):
    """validate_p3_bench_profile — direct-start P3 decision/telemetry bench."""

    def test_default_p3_profile_is_accepted_motor_closed(self):
        normalized = validate_p3_bench_profile(p3_profile(), {})
        self.assertTrue(normalized["bench_p3_only_enabled"])
        self.assertEqual(normalized["bench_p3_start_parkur"], 3)
        self.assertTrue(normalized["dry_run"])
        self.assertFalse(normalized["motor_command_enabled"])
        self.assertFalse(normalized["guided_mode_enabled"])
        self.assertFalse(normalized["canonical_takeover_enabled"])

    def test_start_parkur_bounds_are_enforced(self):
        for value in ("0", "4", "abc", "-1", "3.5"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    validate_p3_bench_profile(
                        p3_profile(bench_p3_start_parkur=value), {}
                    )
        for value in ("1", "2", "3"):
            with self.subTest(value=value):
                normalized = validate_p3_bench_profile(
                    p3_profile(bench_p3_start_parkur=value), {}
                )
                self.assertEqual(normalized["bench_p3_start_parkur"], int(value))

    def test_bench_enabled_requires_strict_boolean(self):
        for value in ("TRUE", 1, "yes", None):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    validate_p3_bench_profile(
                        p3_profile(bench_p3_only_enabled=value), {}
                    )

    def test_disabled_p3_profile_still_accepts_any_start_parkur(self):
        # Fail-closed: enabled=false iken start_parkur değeri bağlanmaz.
        normalized = validate_p3_bench_profile(
            p3_profile(
                bench_p3_only_enabled="false", bench_p3_start_parkur="2"
            ),
            {},
        )
        self.assertFalse(normalized["bench_p3_only_enabled"])
        self.assertEqual(normalized["bench_p3_start_parkur"], 2)

    def test_motor_path_without_exact_ack_is_rejected(self):
        # Motor yolu kapalı olmadan P3 bench kabul edilmez: motor path
        # açılmak istense bile fiziksel ack yoksa reddedilir.
        live = p3_profile(
            canonical_takeover_enabled="true",
            dry_run="false",
            mavlink_router_enabled="true",
            guided_mode_enabled="true",
            motor_command_enabled="true",
        )
        with self.assertRaises(ValueError):
            validate_p3_bench_profile(live, {})

    def test_enabled_rejects_live_motor_guided_or_dry_run_false(self):
        for updates in (
            {"dry_run": "false"},
            {"guided_mode_enabled": "true"},
            {"motor_command_enabled": "true"},
        ):
            with self.subTest(updates=updates):
                with self.assertRaises(ValueError):
                    validate_p3_bench_profile(p3_profile(**updates), {})

    def test_enabled_accepts_takeover_alone_but_rejects_live_motor_combo(self):
        # canonical_takeover tek başına motor yolunu AÇMAZ; kontrat kabul eder.
        normalized = validate_p3_bench_profile(
            p3_profile(canonical_takeover_enabled="true"), {}
        )
        self.assertTrue(normalized["canonical_takeover_enabled"])
        self.assertFalse(normalized["motor_command_enabled"])
        # Motor yolunu gerçekten açan kombinasyon ise fiziksel ack olmadan
        # (ve ack olsa bile dry_run=false/guided/motor P3 karar testine aykırı)
        # reddedilir.
        live = p3_profile(
            canonical_takeover_enabled="true",
            dry_run="false",
            mavlink_router_enabled="true",
            guided_mode_enabled="true",
            motor_command_enabled="true",
        )
        with self.assertRaises(ValueError):
            validate_p3_bench_profile(live, {})
        with self.assertRaises(ValueError):
            validate_p3_bench_profile(
                live, {"IDA_BENCH_PHYSICAL_SAFETY_ACK": BENCH_SAFETY_ACK}
            )

    def test_p3_bench_params_are_bench_only_not_field_classification(self):
        # Autonomy node tarafından declare edilen bench parametreleri bench
        # kontratına aittir; field classification testi bunları harici tutar.
        self.assertIn("bench_p3_only_enabled", BENCH_AUTONOMY_DEPLOYMENT_PARAMETERS)
        self.assertIn("bench_p3_start_parkur", BENCH_AUTONOMY_DEPLOYMENT_PARAMETERS)
        self.assertEqual(BENCH_P3_MIN_START_PARKUR, 1)
        self.assertEqual(BENCH_P3_MAX_START_PARKUR, 3)

    def test_p3_decision_launch_forwards_bench_params_and_closes_motor_path(self):
        p3 = P3_LAUNCH.read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument("bench_p3_only_enabled", default_value="true")', p3)
        self.assertIn('DeclareLaunchArgument("bench_p3_start_parkur", default_value="3")', p3)
        self.assertIn('DeclareLaunchArgument("motor_command_enabled", default_value="false")', p3)
        self.assertIn('DeclareLaunchArgument("guided_mode_enabled", default_value="false")', p3)
        self.assertIn('DeclareLaunchArgument("dry_run", default_value="true")', p3)
        self.assertIn("validate_p3_bench_profile", p3)
        self.assertIn('"bench_p3_only_enabled": LaunchConfiguration("bench_p3_only_enabled")', p3)
        self.assertIn('"bench_p3_start_parkur": LaunchConfiguration("bench_p3_start_parkur")', p3)
        # Motor komutu üretemeyen sıfır/boş portlar; ARM/RC override yok.
        self.assertNotIn("ARM", p3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
