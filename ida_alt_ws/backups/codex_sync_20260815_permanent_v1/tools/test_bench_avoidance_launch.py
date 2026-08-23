"""Unit/static safety tests for the restrained avoidance bench profile."""

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "ida_bringup"))

from ida_bringup.bench_contract import (  # noqa: E402
    BENCH_SAFETY_ACK,
    validate_bench_profile,
)


BENCH = ROOT / "src" / "ida_bringup" / "launch" / "bench_avoidance.launch.py"
REAL = ROOT / "src" / "ida_bringup" / "launch" / "real_vehicle.launch.py"
START = ROOT / "scripts" / "start_real.sh"
BENCH_START = ROOT / "scripts" / "start_bench_avoidance.sh"


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
        self.assertIn('DeclareLaunchArgument("autonomy_max_speed_mps", default_value="0.6")', real)
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
        self.assertIn(
            'class_names:="${IDA_MODEL_CLASS_NAMES:-orange,yellow}"', text
        )
        self.assertNotIn("ros2 service call", text)
        self.assertNotIn("ARM_IDA", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
