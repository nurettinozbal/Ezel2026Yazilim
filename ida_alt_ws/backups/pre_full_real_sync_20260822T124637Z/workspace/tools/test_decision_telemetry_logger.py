from __future__ import annotations

from pathlib import Path
import unittest

from tools.decision_telemetry_logger import compose_record, csv_row, extract_objects


ROOT = Path(__file__).resolve().parents[1]


class DecisionTelemetryLoggerTests(unittest.TestCase):
    def test_p2_right_obstacle_and_left_avoidance_are_explicit(self) -> None:
        latest = {
            "autonomy_state": {
                "state": "PARKUR_2_AVOIDANCE", "current_waypoint": 2,
                "action": "dwa_plan", "failsafe_reason": "",
            },
            "autonomy_debug": {
                "command": {"vx": 0.7, "yaw_rate": -0.45, "action": "dwa_plan"},
                "goal_body": [7.0, 1.5],
            },
            "limited_cmd": {"forward_mps": 0.65, "yaw_rate_rps": -0.4},
            "obstacles": {"obstacles": [{
                "id": "lidar-1", "forward_m": 2.0, "lateral_m": 0.8,
                "distance": 2.15, "color": "yellow", "hard_obstacle": True,
            }]},
            "mission_waypoints": {"waypoints": [
                {"lat": 1.0, "lon": 2.0, "parkur": 1},
                {"lat": 1.1, "lon": 2.1, "parkur": 1},
                {"lat": 1.2, "lon": 2.2, "parkur": 2},
            ]},
            "telemetry": {"motor_left_pwm": 1610, "motor_right_pwm": 1540},
        }
        record = compose_record(latest, {}, 100.25, 10.0)
        row = csv_row(record)
        self.assertEqual(row["parkur"], 2)
        self.assertEqual(row["target_side"], "SAG")
        self.assertEqual(row["limited_turn"], "SOL")
        self.assertEqual(row["left_pwm"], 1610)
        self.assertEqual(record["autonomy"]["phase"], "ENGELDEN_KACINMA")

    def test_raw_lidar_left_axis_is_converted_to_stack_right_axis(self) -> None:
        rows = extract_objects({"lidar_raw": {"clusters": [{
            "id": "raw-1", "forward_m": 3.0, "lateral_left_m": 1.0,
        }]}})
        self.assertEqual(rows[0]["lateral_right_m"], -1.0)
        self.assertEqual(rows[0]["side"], "SOL")

    def test_p1_waypoint_intent_is_recorded(self) -> None:
        latest = {
            "autonomy_state": {
                "state": "PARKUR_1_NAV", "current_waypoint": 0,
                "total_waypoints": 1,
            },
            "autonomy_debug": {
                "command": {"vx": 0.8, "yaw_rate": -0.2, "action": "waypoint_nav"},
                "goal_body": [8.0, -2.0],
            },
            "mission_waypoints": {"waypoints": [
                {"lat": 37.0, "lon": 32.0, "parkur": 1},
            ]},
        }
        record = compose_record(latest, {}, 1.0, 1.0)
        self.assertEqual(record["mission_intent"]["goal_side"], "SOL")
        self.assertEqual(record["mission_intent"]["parkur"], 1)
        self.assertEqual(record["autonomy"]["phase"], "WAYPOINT_TAKIP")

    def test_p3_yki_target_camera_lock_and_contact_phases(self) -> None:
        base = {
            "autonomy_state": {
                "state": "PARKUR_3_TARGET_LOCK", "current_waypoint": 0,
            },
            "target_color": {"target_color": "green"},
            "camera_p3": {"detections": [{
                "id": "cam-green", "color": "green", "confidence": 0.72,
                "bearing_deg": 12.0, "bbox_norm_x": 0.2, "bbox_size": 0.12,
            }]},
        }
        locked = {**base, "autonomy_debug": {"command": {
            "vx": 0.7, "yaw_rate": 0.25,
            "action": "target_lock color=green distance=2.0 center=0.2",
        }}}
        record = compose_record(locked, {}, 1.0, 1.0)
        self.assertEqual(record["engagement"]["requested_color"], "green")
        self.assertEqual(record["engagement"]["camera_match"]["side"], "SAG")
        self.assertEqual(record["autonomy"]["phase"], "HEDEF_KILIDI")
        self.assertEqual(record["planned_command"]["turn"], "SAG")

        contact = {**base, "autonomy_state": {"state": "ENGAGE", "current_waypoint": 0},
                   "autonomy_debug": {"command": {
                       "vx": 0.7, "yaw_rate": 0.0, "action": "engage_contact_window",
                   }}}
        self.assertEqual(
            compose_record(contact, {}, 2.0, 2.0)["autonomy"]["phase"],
            "TEMAS_PENCERESI",
        )

    def test_parkur_falls_back_to_autonomy_state_when_mission_late(self) -> None:
        """Geç abonelikte /mission/waypoints kaçarsa state adı parkuru doğrular."""
        latest = {
            "autonomy_state": {
                "state": "PARKUR_2_AVOIDANCE", "current_waypoint": 2,
                "total_waypoints": 4,
            },
            "autonomy_debug": {"command": {"vx": 0.6, "yaw_rate": -0.3}},
        }
        record = compose_record(latest, {}, 1.0, 1.0)
        intent = record["mission_intent"]
        self.assertEqual(intent["parkur"], 2)
        self.assertEqual(intent["total_waypoints"], 4)
        self.assertIsNone(intent["waypoint_lat"])
        self.assertIsNone(intent["waypoint_lon"])

    def test_parkur_falls_back_to_waypoint_index_budget(self) -> None:
        """Mission listesi yokken total_waypoints + current_waypoint tahmini."""
        latest = {
            "autonomy_state": {
                "state": "MISSION_READY", "current_waypoint": 3,
                "total_waypoints": 5,
            },
        }
        intent = compose_record(latest, {}, 1.0, 1.0)["mission_intent"]
        self.assertEqual(intent["parkur"], 2)
        # current_wp >= total -> P3 (hedef kümesi)
        intent = compose_record(
            {**latest, "autonomy_state": {
                **latest["autonomy_state"], "current_waypoint": 5,
            }}, {}, 1.0, 1.0
        )["mission_intent"]
        self.assertEqual(intent["parkur"], 3)

    def test_mission_waypoints_still_win_over_state_fallback(self) -> None:
        """Waypoint listesi varsa gerçek parkur/kordinatlar kullanılır."""
        latest = {
            "autonomy_state": {
                "state": "PARKUR_2_AVOIDANCE", "current_waypoint": 1,
                "total_waypoints": 3,
            },
            "mission_waypoints": {"waypoints": [
                {"lat": 37.0, "lon": 32.0, "parkur": 1},
                {"lat": 37.1, "lon": 32.1, "parkur": 2},
                {"lat": 37.2, "lon": 32.2, "parkur": 3},
            ]},
        }
        intent = compose_record(latest, {}, 1.0, 1.0)["mission_intent"]
        self.assertEqual(intent["parkur"], 2)
        self.assertAlmostEqual(intent["waypoint_lat"], 37.1)
        self.assertAlmostEqual(intent["waypoint_lon"], 32.1)

    def test_motor_pwm_reads_bridge_servo_output_fields(self) -> None:
        """mavsdk_bridge SERVO_OUTPUT_RAW -> motor_left/right_pwm kanıtı."""
        latest = {
            "autonomy_state": {"state": "MISSION_READY", "current_waypoint": 0},
            "telemetry": {"motor_left_pwm": 1620, "motor_right_pwm": 1500},
        }
        record = compose_record(latest, {}, 1.0, 1.0)
        motor = record["motor_output"]
        self.assertEqual(motor["left_pwm"], 1620)
        self.assertEqual(motor["right_pwm"], 1500)
        self.assertTrue(motor["measured"])
        self.assertEqual(motor["source"], "telemetry/state")
        self.assertEqual(csv_row(record)["left_pwm"], 1620)

    def test_tool_has_no_actuation_or_mavlink_surface(self) -> None:
        source = (ROOT / "tools" / "decision_telemetry_logger.py").read_text(
            encoding="utf-8"
        )
        for forbidden in (
            "create_publisher(", "create_client(", "pymavlink", "mavsdk",
            "subprocess", "os.system", "/mission/start", "/control/arm",
        ):
            self.assertNotIn(forbidden, source)
        cli = (ROOT / "scripts" / "ida_alt_ws").read_text(encoding="utf-8")
        self.assertIn("decision-log) shift; cmd_decision_log", cli)
        self.assertIn("ros2 param dump /ida_autonomy", cli)


if __name__ == "__main__":
    unittest.main()
