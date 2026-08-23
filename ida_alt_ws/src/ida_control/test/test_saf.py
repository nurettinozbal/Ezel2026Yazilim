"""Saf-Python (rclpy'siz) unittest'ler: ida_control.

mission_contract.mission_items_to_waypoints ve map_target_color saf fonksiyonlarını
test eder (MAVSDK MissionItem -> README waypoint kontratı; param/renk kodu ->
renk). Jetson'da ROS2 Humble ile colcon test ya da dev makinesinde
doğrudan ``python -m unittest`` ile çalışır (ament bağımlılığı yok).
"""

import os
import sys
import unittest

_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from ida_control.mission_contract import (
    COLOR_INT_MAP,
    MAV_CMD_DO_JUMP,
    MAV_CMD_DO_SET_MODE,
    MAV_CMD_NAV_WAYPOINT,
    MAV_COORD_SCALE,
    body_to_ned,
    map_target_color,
    mission_command_is_next,
    mission_items_to_waypoints,
    decode_mission_mailbox,
    exact_mission_control_token,
    next_mission_control,
    pack_mission_counts,
    pending_mission_control,
    unpack_mission_counts,
    ned_command_to_dict,
    set_position_target_local_ned_mask,
)
from ida_control.yki_status_contract import (
    autonomy_fields,
    logging_fields,
    perception_count,
)


class FakeMissionItem:
    """MAVSDK MissionItem'ın yalnız ihtiyaç duyulan alanlarını taklit eder."""

    def __init__(self, seq, x, y, command=MAV_CMD_NAV_WAYPOINT):
        self.seq = seq
        self.x = x
        self.y = y
        self.command = command


class TestMapTargetColor(unittest.TestCase):
    def test_real_yki_color_codes(self) -> None:
        # YKİ kontratı: KIRMIZI=1, YEŞİL=2, SİYAH=4 (MAVİ=3 boş — şartname).
        self.assertEqual(
            COLOR_INT_MAP,
            {1: "red", 2: "green", 3: "orange", 4: "black", 5: "yellow"},
        )
        self.assertEqual(map_target_color(1), "red")
        self.assertEqual(map_target_color(2), "green")
        self.assertEqual(map_target_color(4), "black")
        self.assertEqual(map_target_color(3), "orange")
        self.assertEqual(map_target_color(5), "yellow")
        self.assertIsNone(map_target_color(99))
        self.assertIsNone(map_target_color("abc"))


class TestMissionItemsToWaypoints(unittest.TestCase):
    def test_coordinate_round_trip(self) -> None:
        # 40.86305 / 29.25995 -> int32 x1e7 -> geri 7 ondalık.
        items = [
            FakeMissionItem(
                seq=0,
                x=int(round(40.86305 * MAV_COORD_SCALE)),
                y=int(round(29.25995 * MAV_COORD_SCALE)),
                command=MAV_CMD_NAV_WAYPOINT,
            )
        ]
        waypoints = mission_items_to_waypoints(items)
        self.assertEqual(waypoints, [{"lat": round(40.86305, 7), "lon": round(29.25995, 7), "parkur": 1}])

    def test_parkur_classification_from_command(self) -> None:
        items = [
            FakeMissionItem(seq=0, x=410000000, y=290000000, command=MAV_CMD_NAV_WAYPOINT),
            FakeMissionItem(seq=1, x=420000000, y=300000000, command=MAV_CMD_DO_SET_MODE),
            FakeMissionItem(seq=2, x=430000000, y=310000000, command=MAV_CMD_DO_JUMP),
            FakeMissionItem(seq=3, x=440000000, y=320000000, command=999),  # bilinmeyen -> 1
        ]
        parkurlar = [wp["parkur"] for wp in mission_items_to_waypoints(items)]
        self.assertEqual(parkurlar, [1, 2, 3, 1])

    def test_safe_nav_items_use_verified_boundary_counts(self) -> None:
        items = [
            FakeMissionItem(seq=i, x=410000000 + i, y=290000000 + i)
            for i in range(7)  # seq0 HOME + 6 gerçek waypoint
        ]
        waypoints = mission_items_to_waypoints(items, p1_count=4, p2_count=1)
        self.assertEqual([wp["parkur"] for wp in waypoints], [1, 1, 1, 1, 2, 3])

    def test_metadata_mode_removes_ardupilot_home_slot(self) -> None:
        items = [
            FakeMissionItem(seq=0, x=408630501, y=292599517),  # HOME
            FakeMissionItem(seq=1, x=408631400, y=292599517),  # gerçek WP1
            FakeMissionItem(seq=2, x=408633649, y=292603084),  # gerçek WP2
            FakeMissionItem(seq=3, x=408634548, y=292616165),  # gerçek WP3
        ]
        waypoints = mission_items_to_waypoints(items, p1_count=2, p2_count=1)
        self.assertEqual(len(waypoints), 3)
        self.assertEqual(waypoints[0]["lat"], 40.86314)
        self.assertEqual([wp["parkur"] for wp in waypoints], [1, 1, 2])

    def test_invalid_boundary_metadata_fails_closed(self) -> None:
        items = [FakeMissionItem(seq=0, x=410000000, y=290000000)]
        self.assertEqual(mission_items_to_waypoints(items, p1_count=0, p2_count=1), [])
        self.assertEqual(mission_items_to_waypoints(items, p1_count=1, p2_count=1), [])

    def test_items_sorted_by_seq(self) -> None:
        # Sıralı gelmeyen liste seq'e göre düzeltilmeli.
        items = [
            FakeMissionItem(seq=2, x=430000000, y=310000000),
            FakeMissionItem(seq=0, x=410000000, y=290000000),
            FakeMissionItem(seq=1, x=420000000, y=300000000),
        ]
        waypoints = mission_items_to_waypoints(items)
        self.assertEqual([wp["lat"] for wp in waypoints], [41.0, 42.0, 43.0])

    def test_invalid_coordinates_skipped(self) -> None:
        items = [
            FakeMissionItem(seq=0, x=410000000, y=290000000),
            FakeMissionItem(seq=1, x=0, y=0),          # (0,0) geçersiz sayılmaz ama sınırda
            FakeMissionItem(seq=2, x=999999999, y=0),  # lat 99.99 -> atlanır
        ]
        waypoints = mission_items_to_waypoints(items)
        lats = [wp["lat"] for wp in waypoints]
        self.assertNotIn(99.9999999, lats)

    def test_empty_input(self) -> None:
        self.assertEqual(mission_items_to_waypoints([]), [])


class TestMissionControlProtocol(unittest.TestCase):
    def test_single_mailbox_start_and_stop_transitions(self) -> None:
        self.assertEqual(next_mission_control(8_000_000.0, start=True), (8_000_001, 8_000_003))
        self.assertEqual(pending_mission_control(8_000_001.0), (8_000_001, True, 8_000_003))
        self.assertEqual(next_mission_control(8_000_003.0, start=False), (8_000_006, 8_000_008))
        self.assertEqual(pending_mission_control(8_000_006.0), (8_000_006, False, 8_000_008))
        self.assertIsNone(next_mission_control(8_000_001.0, start=False))

    def test_tokens_are_exact_finite_and_bounded(self) -> None:
        self.assertEqual(exact_mission_control_token(8_000_001.0), 8_000_001)
        self.assertEqual(decode_mission_mailbox(8_000_003), ("ack", 1, True))
        for value in (True, 1.2, 7_999_999, float("nan"), float("inf"), 16_000_001):
            self.assertIsNone(exact_mission_control_token(value))

    def test_sequence_wrap_is_explicit(self) -> None:
        self.assertEqual(next_mission_control(15_999_996.0, start=True), (8_000_001, 8_000_003))

    def test_live_sequence_gap_and_replay_are_rejected(self) -> None:
        self.assertTrue(mission_command_is_next(8_000_006, 1))
        self.assertFalse(mission_command_is_next(8_000_001, 1))
        self.assertFalse(mission_command_is_next(8_000_009, 1))
        self.assertTrue(mission_command_is_next(8_000_001, 1_999_999))

    def test_parkur_counts_share_one_float32_safe_parameter(self) -> None:
        self.assertEqual(pack_mission_counts(4, 1), 4005)
        self.assertEqual(unpack_mission_counts(4005.0), (4, 1))
        for value in (4005.5, 0, True, float("nan"), 1_002_002):
            self.assertIsNone(unpack_mission_counts(value))


class TestBodyToNed(unittest.TestCase):
    """Gövde hızı -> dünya NED dönüşümü (ArduRover GUIDED komut yolu).

    Formül ida_planning.geo.body_to_world ile birebir (sim telemetry_sim_node
    aynı dönüşümü kullanır). heading 0 = kuzey.
    """

    def test_heading_zero_forward_is_north(self) -> None:
        north, east = body_to_ned(0.8, 0.0, 0.0)
        self.assertAlmostEqual(north, 0.8, places=6)
        self.assertAlmostEqual(east, 0.0, places=6)

    def test_heading_90_forward_is_east(self) -> None:
        north, east = body_to_ned(0.8, 0.0, 90.0)
        self.assertAlmostEqual(north, 0.0, places=6)
        self.assertAlmostEqual(east, 0.8, places=6)

    def test_heading_180_forward_is_south(self) -> None:
        north, east = body_to_ned(0.8, 0.0, 180.0)
        self.assertAlmostEqual(north, -0.8, places=6)
        self.assertAlmostEqual(east, 0.0, places=6)

    def test_heading_45_forward_splits(self) -> None:
        north, east = body_to_ned(0.8, 0.0, 45.0)
        expected = 0.8 * (2 ** 0.5) / 2.0
        self.assertAlmostEqual(north, expected, places=6)
        self.assertAlmostEqual(east, expected, places=6)

    def test_vy_zero_preserved_differential(self) -> None:
        # vy=0 (diferansiyel): east bileşeni yalnız vx*sin(h) — dönüş kaynaklı.
        for heading in (0.0, 30.0, 90.0, 270.0):
            north, east = body_to_ned(0.8, 0.0, heading)
            self.assertAlmostEqual(north ** 2 + east ** 2, 0.8 ** 2, places=6)


class TestPositionTargetMask(unittest.TestCase):
    def test_default_mask_yaw_rate_enabled(self) -> None:
        # ArduRover speed + turn-rate dalını seçen proje maskesi.
        self.assertEqual(set_position_target_local_ned_mask(), 0x05E7)

    def test_disable_yaw_rate_mask_all(self) -> None:
        # yaw_rate devre dışı -> NED velocity-only.
        self.assertEqual(set_position_target_local_ned_mask(False), 0x0FC7)


class TestNedCommandToDict(unittest.TestCase):
    def test_basic_command(self) -> None:
        result = ned_command_to_dict(0.8, 0.0, 0.5, 0.0)
        self.assertAlmostEqual(result["vx_north"], 0.8, places=6)
        self.assertAlmostEqual(result["vy_east"], 0.0, places=6)
        self.assertEqual(result["yaw_rate"], 0.5)

    def test_heading_90(self) -> None:
        result = ned_command_to_dict(0.8, 0.0, 0.5, 90.0)
        self.assertAlmostEqual(result["vx_north"], 0.0, places=6)
        self.assertAlmostEqual(result["vy_east"], 0.8, places=6)


class TestYkiStatusContract(unittest.TestCase):
    def test_autonomy_fields_match_canonical_wire(self) -> None:
        self.assertEqual(
            autonomy_fields({"state": "PARKUR_2_AVOIDANCE", "action": "avoid dwa"}),
            {"AUTO_ST": 4, "AUTO_AC": 4, "PARKUR": 2},
        )

    def test_perception_and_logging_are_bounded(self) -> None:
        self.assertEqual(perception_count({"detections": [{}, {}]}, "detections"), 102)
        self.assertEqual(logging_fields({"active": True, "logger_count": 3}), {
            "LOG_ACT": 1, "LOG_CNT": 103,
        })
        self.assertEqual(logging_fields({"active": 1, "logger_count": -5}), {
            "LOG_ACT": 0, "LOG_CNT": 100,
        })

    def test_malformed_payloads_fail_closed(self) -> None:
        self.assertEqual(autonomy_fields("bad"), {})
        self.assertEqual(perception_count({"obstacles": "bad"}, "obstacles"), 100)
        self.assertEqual(logging_fields(None), {})


if __name__ == "__main__":
    unittest.main()
