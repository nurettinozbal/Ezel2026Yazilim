"""Saf-Python (rclpy'siz) unittest'ler: ida_ws_gateway.

Jetson'da ROS2 Humble ile colcon test ya da dev makinesinde doğrudan
``python -m unittest`` ile çalışır (ament bağımlılığı yoktur).
"""

import os
import sys
import unittest

# Paket kökünü import yoluna ekle (colcon'dan bağımsız, dev makinesinde de çalışır).
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)
# ida_planning (contracts renk sabitleri) için planning paketinin iç klasörünü ekle.
# NOT: src/ida_planning dış klasörü namespace pakettir; gerçek modüller iç klasördedir.
_WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_PLANNING_ROOT = os.path.join(_WORKSPACE_ROOT, "ida_planning")
if _PLANNING_ROOT not in sys.path:
    sys.path.insert(0, _PLANNING_ROOT)

from ida_planning.contracts import COLORS, VALID_TARGET_COLORS

from ida_ws_gateway.dry_run_link import DryRunLink, parse_demo_waypoints
from ida_ws_gateway.mavlink_parser import (
    DEFAULT_COLOR_INT_MAP,
    MAV_COORD_SCALE,
    STATE_CODE_MAP,
    classify_parkur,
    encode_action,
    encode_log_count,
    encode_parkur_from_state,
    encode_perception_count,
    extract_named_and_mission,
    is_target_color_field,
    make_mission_waypoints,
    map_color_int,
    normalize_named_value_field,
    parse_mission_item_int,
    parse_named_value_int,
    round_to_deg7,
)


class TestNamedValueInt(unittest.TestCase):
    def test_valid_named_value(self) -> None:
        raw = {"mavpackettype": "NAMED_VALUE_INT", "param_id": "TARGET_COLOR", "value": 2}
        parsed = parse_named_value_int(raw)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed["param_id"], "TARGET_COLOR")
        self.assertEqual(parsed["value"], 2)

    def test_mavlink_alias_type(self) -> None:
        # pymavlink to_dict() "mavpackettype" üretir; alias'lar da kabul edilir.
        raw = {"msg_type": "NAMED_VALUE_INT", "param_id": "TARGET_COLOR", "value": "1"}
        parsed = parse_named_value_int(raw)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed["value"], 1)

    def test_wrong_type_returns_none(self) -> None:
        self.assertIsNone(parse_named_value_int({"mavpackettype": "HEARTBEAT", "value": 1}))
        self.assertIsNone(parse_named_value_int({"mavpackettype": "NAMED_VALUE_INT", "param_id": "X"}))
        self.assertIsNone(parse_named_value_int("garbage"))
        self.assertIsNone(parse_named_value_int(None))

    def test_target_col_truncated_char10_field(self) -> None:
        # Gerçek YKİ b"TARGET_COLOR" gönderir; MAVLink char[10] hatta kırpar.
        # pymavlink bunu bytes + \x00 dolgusuyla döndürebilir; normalize edilmelidir.
        raw = {"mavpackettype": "NAMED_VALUE_INT", "param_id": b"TARGET_COL\x00\x00", "value": 2}
        parsed = parse_named_value_int(raw)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed["param_id"], "TARGET_COL")
        self.assertTrue(is_target_color_field(parsed["param_id"]))
        self.assertTrue(is_target_color_field(b"TARGET_COLOR\x00\x00"))
        self.assertFalse(is_target_color_field("OTHER"))

    def test_real_yki_color_code_mapping_spec_compliant(self) -> None:
        # Mevcut üç kod korunur; 3=TURUNCU ve 5=SARI saha hedefleridir.
        self.assertEqual(
            DEFAULT_COLOR_INT_MAP,
            {"1": "red", "2": "green", "3": "orange", "4": "black", "5": "yellow"},
        )
        self.assertEqual(map_color_int(1), "red")
        self.assertEqual(map_color_int(2), "green")
        self.assertEqual(map_color_int(3), "orange")
        self.assertEqual(map_color_int(4), "black")
        self.assertEqual(map_color_int(5), "yellow")
        self.assertIsNone(map_color_int(99))

    def test_map_sourced_from_contracts(self) -> None:
        # Renk adları tek kaynaktan (contracts.COLORS) gelir; şartname renkleriyle uyumlu.
        self.assertEqual(DEFAULT_COLOR_INT_MAP["1"], COLORS["target_red"])
        self.assertEqual(DEFAULT_COLOR_INT_MAP["2"], COLORS["target_green"])
        self.assertEqual(DEFAULT_COLOR_INT_MAP["4"], COLORS["target_black"])
        self.assertEqual(DEFAULT_COLOR_INT_MAP["3"], COLORS["edge_buoy"])
        self.assertEqual(DEFAULT_COLOR_INT_MAP["5"], COLORS["obstacle_buoy"])
        # Eşleme değerleri geçerli hedef renkleri içinde olmalı.
        self.assertTrue(set(DEFAULT_COLOR_INT_MAP.values()) <= VALID_TARGET_COLORS)


class TestIdaAutonomyEncoding(unittest.TestCase):
    """İDA otonomi durumu -> NAMED_VALUE_INT kodlama (YKİ kontratıyla aynı)."""

    def test_state_code_map_matches_yki_contract(self):
        # YKİ services/autonomy_contract.py STATE_TO_CODE ile birebir.
        expected = {
            "WAIT_MISSION": 1,
            "MISSION_READY": 2,
            "PARKUR_1_NAV": 3,
            "PARKUR_2_AVOIDANCE": 4,
            "PARKUR_3_TARGET_LOCK": 5,
            "ENGAGE": 6,
            "FAILSAFE": 7,
            "COMPLETE": 8,
        }
        self.assertEqual(STATE_CODE_MAP, expected)

    def test_parkur_derived_from_state(self):
        self.assertEqual(encode_parkur_from_state("PARKUR_1_NAV"), 1)
        self.assertEqual(encode_parkur_from_state("PARKUR_2_AVOIDANCE"), 2)
        self.assertEqual(encode_parkur_from_state("PARKUR_3_TARGET_LOCK"), 3)
        self.assertEqual(encode_parkur_from_state("ENGAGE"), 3)
        self.assertEqual(encode_parkur_from_state("COMPLETE"), 0)  # bilinmeyen -> 0
        self.assertEqual(encode_parkur_from_state(""), 0)

    def test_action_prefix_matching(self):
        # Autonomy action string'leri float içerir; önek eşleşmesi gerekir.
        self.assertEqual(encode_action("idle"), 1)
        self.assertEqual(encode_action("hold target_color_missing"), 1)
        self.assertEqual(encode_action("waypoint distance=4.2 yaw_error=-12.3"), 2)
        self.assertEqual(encode_action("corridor bias"), 3)
        self.assertEqual(encode_action("avoid obstacle"), 4)
        self.assertEqual(encode_action("target_hold"), 5)
        self.assertEqual(encode_action("search_target_360 ts3_risk=0"), 5)
        self.assertEqual(encode_action("target_lock color=black distance=2.5"), 5)
        self.assertEqual(encode_action("target_confirming"), 5)
        self.assertEqual(encode_action("target_align ts3_risk=0"), 5)
        self.assertEqual(encode_action("target_loss_grace"), 5)
        self.assertEqual(encode_action("engage_target_lost_stop"), 5)
        self.assertEqual(encode_action("engage_alignment_lost_stop"), 5)
        self.assertEqual(encode_action("engage_contact_window"), 6)
        self.assertEqual(encode_action("failsafe telemetry_timeout"), 7)
        self.assertEqual(encode_action("mission_complete"), 8)
        self.assertEqual(encode_action(""), 0)
        self.assertEqual(encode_action("bilinmeyen_aksiyon"), 0)

    def test_perception_and_log_encodings(self):
        self.assertEqual(encode_perception_count(0), 100)
        self.assertEqual(encode_perception_count(2), 102)
        self.assertEqual(encode_perception_count(-1), 100)  # negatif -> 0 sayım
        self.assertEqual(encode_log_count(3), 103)
        self.assertEqual(encode_log_count(0), 100)
        self.assertEqual(encode_log_count(-2), 100)

    def test_all_field_names_are_within_char10(self):
        # Alan adları <= 10 karakter: MAVLink char[10] asla kırpmaz.
        # NOT: AUTONOMY_ST (11) kırpılırdı; AUTO_ST/AUTO_AC güvenli (test bunu korur).
        names = ["AUTO_ST", "AUTO_AC", "PARKUR", "PERC_DET", "PERC_OBS", "LOG_ACT", "LOG_CNT"]
        for name in names:
            self.assertLessEqual(len(name), 10, name)


class TestMissionItemInt(unittest.TestCase):
    def test_coordinate_round_trip(self) -> None:
        # 40.8630500 / 29.2599500 -> int32 x1e7 -> geri 7 ondalık.
        raw = {
            "mavpackettype": "MISSION_ITEM_INT",
            "x": int(round(40.86305 * MAV_COORD_SCALE)),
            "y": int(round(29.25995 * MAV_COORD_SCALE)),
            "command": 16,
        }
        wp = parse_mission_item_int(raw)
        self.assertIsNotNone(wp)
        assert wp is not None
        self.assertEqual(wp["lat"], round(40.86305, 7))
        self.assertEqual(wp["lon"], round(29.25995, 7))
        self.assertEqual(wp["parkur"], 16)

    def test_precision_seven_decimals(self) -> None:
        # 1e-7 derece çözünürlük: 7 ondalık korunur, kayıp yok.
        value = 40.8630521
        raw = {
            "mavpackettype": "MISSION_ITEM_INT",
            "x": int(round(value * MAV_COORD_SCALE)),
            "y": int(round(29.25995 * MAV_COORD_SCALE)),
            "command": 16,
        }
        wp = parse_mission_item_int(raw)
        self.assertIsNotNone(wp)
        assert wp is not None
        self.assertAlmostEqual(wp["lat"], 40.8630521, places=7)

    def test_invalid_item_returns_none(self) -> None:
        self.assertIsNone(parse_mission_item_int({"mavpackettype": "MISSION_ITEM_INT"}))
        self.assertIsNone(parse_mission_item_int({"mavpackettype": "HEARTBEAT", "x": 1, "y": 1}))
        self.assertIsNone(parse_mission_item_int({}))


class TestWaypointPipeline(unittest.TestCase):
    def test_make_mission_waypoints_parkur_classification(self) -> None:
        raw_items = [
            {"mavpackettype": "MISSION_ITEM_INT", "x": 408630500, "y": 292599500, "command": 16},
            {"mavpackettype": "MISSION_ITEM_INT", "x": 408631500, "y": 292600500, "command": 176},
            {"mavpackettype": "MISSION_ITEM_INT", "x": 408632500, "y": 292601500, "command": 177},
        ]
        waypoints = make_mission_waypoints(raw_items)
        self.assertEqual(len(waypoints), 3)
        self.assertEqual([w["parkur"] for w in waypoints], [1, 2, 3])
        self.assertEqual(waypoints[0]["lat"], 40.86305)

    def test_extract_named_and_mission_order_independent(self) -> None:
        raw_messages = [
            {"mavpackettype": "MISSION_ITEM_INT", "x": 408630500, "y": 292599500, "command": 16},
            {"mavpackettype": "NAMED_VALUE_INT", "param_id": "TARGET_COLOR", "value": 1},
            {"mavpackettype": "HEARTBEAT", "type": 0},
        ]
        target, waypoints = extract_named_and_mission(raw_messages)
        self.assertIsNotNone(target)
        assert target is not None
        self.assertEqual(target["value"], 1)
        self.assertEqual(len(waypoints), 1)

    def test_classify_parkur_fallback(self) -> None:
        self.assertEqual(classify_parkur(16), 1)
        self.assertEqual(classify_parkur(176), 2)
        self.assertEqual(classify_parkur(177), 3)
        self.assertEqual(classify_parkur(99), 1)  # bilinmeyen -> parkur 1

    def test_round_to_deg7(self) -> None:
        self.assertEqual(round_to_deg7(40.86305213), 40.8630521)
        self.assertTrue(isinstance(round_to_deg7("40.5"), float))


class TestDryRunLink(unittest.TestCase):
    def test_explicit_empty_demo_waypoints_disables_mission_seed(self):
        self.assertEqual(parse_demo_waypoints("[]"), [])

    def test_seeded_messages_round_trip(self) -> None:
        link = DryRunLink(
            demo_target_color="green",
            color_int_map={"1": "red", "2": "green", "4": "black"},
        )
        link.connect()
        batch = link.recv_batch()
        self.assertGreaterEqual(len(batch), 3)  # 1 renk + >=2 waypoint

        named = [m for m in batch if m["mavpackettype"] == "NAMED_VALUE_INT"]
        items = [m for m in batch if m["mavpackettype"] == "MISSION_ITEM_INT"]
        self.assertEqual(len(named), 1)
        self.assertEqual(named[0]["param_id"], "TARGET_COLOR")
        self.assertEqual(named[0]["value"], 2)  # green -> 2
        self.assertGreaterEqual(len(items), 2)
        # Yayınlanan waypoint'ler README formatına uygun.
        target, waypoints = extract_named_and_mission(batch)
        self.assertEqual(target["value"], 2)
        self.assertEqual(len(waypoints), len(items))

    def test_demo_unknown_color_not_seeded(self) -> None:
        # Tanımsız renk explicit haritada yoksa seed edilmez.
        link = DryRunLink(
            demo_target_color="blue",
            color_int_map={"1": "red", "2": "green", "4": "black"},
        )
        link.connect()
        batch = link.recv_batch()
        named = [m for m in batch if m["mavpackettype"] == "NAMED_VALUE_INT"]
        self.assertEqual(named, [])

    def test_batch_consumed_once(self) -> None:
        link = DryRunLink()
        link.connect()
        self.assertGreater(len(link.recv_batch()), 0)
        self.assertEqual(link.recv_batch(), [])

    def test_parse_demo_waypoints(self) -> None:
        parsed = parse_demo_waypoints('[{"lat": 40.86, "lon": 29.25, "parkur": 3}]')
        self.assertEqual(len(parsed), 1)
        self.assertEqual(parsed[0]["parkur"], 3)
        # Geçersiz JSON -> built-in demo.
        self.assertGreaterEqual(len(parse_demo_waypoints("not json")), 2)
        # Boş -> built-in demo.
        self.assertGreaterEqual(len(parse_demo_waypoints("")), 2)


if __name__ == "__main__":
    unittest.main()
