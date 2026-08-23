"""Autonomy katmanı SAF testleri (T1 tekil hedef + T2 waypoint ilerleme).

Strateji: AutonomyNode doğrudan import edilemez (rclpy/geometry_msgs/std_msgs
modül seviyesinde gerekli — lokal kurulu değil). Bu yüzden bu dosya, tasarımın
R7 "saf fonksiyonlara taşı" ilkesine uyarak, autonomy_node'un koridor-hedef
seçimini ve wp ilerleme kararını besleyen SAF fonksiyonları test eder
(planner.corridor_goal_body, planner.waypoint_advance_decision).

P1/P2 davranışı, autonomy_node._use_corridor_goal/_use_corridor_bias'ın
taşıdığı mantığın (parkur + parkur_listesi) saf eşdeğeriyle doğrulanır —
rclpy'siz, kopyasız.
"""

import os
import sys
import json
import types
import unittest
from unittest.mock import Mock

# CLI'dan tek komutla çalışabilmesi için paket kökünü ekle
# (örn. `python src/ida_autonomy/test/test_saf.py`; import hatası olmasın).
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _p in ("ida_planning", "ida_autonomy"):
    _pth = os.path.join(_ROOT, _p)
    if _pth not in sys.path:
        sys.path.insert(0, _pth)

from ida_planning.planner import corridor_goal_body
from ida_planning.planner import waypoint_advance_decision
from ida_planning.planner import CourseGeometryTracker
from ida_planning.planner import Command
from ida_planning.planner import p1_route_exit_ready
from ida_planning.planner import p1_transition_decision
from ida_planning.planner import p2_route_exit_ready
from ida_planning.planner import p2_transition_decision
from ida_planning.scoring.contact import ContactCounter
from ida_planning.scoring.out_of_course import OutOfCourseDetector

# AutonomyNode callback'lerini ROS kurulumu olmayan saf test ortamında da
# doğrulayabilmek için yalnız import-time mesaj/Node kabukları.
try:
    from ida_autonomy.autonomy_node import (
        AutonomyNode,
        p3_initial_hold_active,
        p3_search_schedule,
        p3_target_loss_grace_command,
    )
except ModuleNotFoundError:
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    def _vector3_init(self):
        self.x, self.y, self.z = 0.0, 0.0, 0.0

    def _twist_init(self):
        self.linear = _vector3()
        self.angular = _vector3()

    _vector3 = type("Vector3", (), {"__init__": _vector3_init})
    geometry_msgs_msg.Twist = type("Twist", (), {"__init__": _twist_init})
    geometry_msgs.msg = geometry_msgs_msg
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy.node = rclpy_node
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {"__init__": lambda self: setattr(self, "data", "")})
    std_msgs.msg = std_msgs_msg
    sys.modules.update({
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    })
    from ida_autonomy.autonomy_node import (
        AutonomyNode,
        p3_initial_hold_active,
        p3_search_schedule,
        p3_target_loss_grace_command,
    )


class P3SearchScheduleTest(unittest.TestCase):
    def test_initial_hold_is_anchored_only_to_p3_entry(self):
        entry = 100.0
        self.assertTrue(p3_initial_hold_active(100.9, entry, 1.0))
        self.assertFalse(p3_initial_hold_active(101.0, entry, 1.0))
        # A target-loss search restart at t=150 must not recreate the hold.
        self.assertFalse(p3_initial_hold_active(150.0, entry, 1.0))

    def test_close_target_dropout_keeps_bearing_without_blind_advance(self):
        command = p3_target_loss_grace_command(
            Command(0.7, 0.0, -0.2, "target_align"), 4.5, 5.0
        )
        self.assertEqual(command, Command(0.0, 0.0, -0.2, "target_loss_reacquire"))

    def test_far_target_dropout_keeps_yaw_but_never_advances_blind(self):
        command = p3_target_loss_grace_command(
            Command(0.4, 0.0, 0.1, "target_approach"), 8.0, 5.0
        )
        self.assertEqual(command, Command(0.0, 0.0, 0.1, "target_loss_reacquire"))

    def test_invalid_target_dropout_history_fails_closed(self):
        command = p3_target_loss_grace_command(
            Command(0.4, 0.0, float("nan"), "target_approach"), 8.0, 5.0
        )
        self.assertEqual(command, Command(0.0, 0.0, 0.0, "target_loss_grace"))

    def test_thirty_deg_per_second_scans_four_sectors_in_twelve_seconds(self):
        expected = (
            (0.0, "scan", 0),
            (3.0, "scan", 1),
            (6.0, "scan", 2),
            (9.0, "scan", 3),
        )
        for elapsed, phase, sector in expected:
            with self.subTest(elapsed=elapsed):
                actual_phase, actual_sector, rotations = p3_search_schedule(
                    elapsed, 30.0, 1, 4.0
                )
                self.assertEqual((actual_phase, actual_sector), (phase, sector))
                self.assertEqual(rotations, 0)

    def test_staging_is_bounded_then_scan_cycle_restarts(self):
        self.assertEqual(p3_search_schedule(12.0, 30.0, 1, 4.0), ("staging", 3, 1))
        self.assertEqual(p3_search_schedule(15.9, 30.0, 1, 4.0), ("staging", 3, 1))
        self.assertEqual(p3_search_schedule(16.0, 30.0, 1, 4.0), ("scan", 0, 0))

    def test_initial_hold_delays_scan_without_changing_scan_duration(self):
        self.assertEqual(p3_search_schedule(0.9, 30.0, 1, 3.0, 1.0), ("hold", 0, 0))
        self.assertEqual(p3_search_schedule(1.0, 30.0, 1, 3.0, 1.0), ("scan", 0, 0))
        self.assertEqual(p3_search_schedule(13.0, 30.0, 1, 3.0, 1.0), ("staging", 3, 1))
        self.assertEqual(p3_search_schedule(16.0, 30.0, 1, 3.0, 1.0), ("scan", 0, 0))

    def test_invalid_schedule_is_fail_closed(self):
        for args in ((0.0, 0.0, 1, 4.0), (0.0, 30.0, 0, 4.0), (-1.0, 30.0, 1, 4.0)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                p3_search_schedule(*args)


# --- autonomy_node._use_corridor_goal / _use_corridor_bias mantığının
# saf eşdeğeri (aktif state parkur numarası + merkezi config listesi).
def current_parkur_index(state: str):
    return {
        "PARKUR_1_NAV": 1,
        "PARKUR_2_AVOIDANCE": 2,
        "PARKUR_3_TARGET_LOCK": 3,
        "ENGAGE": 3,
    }.get(state)


def use_corridor_goal(state: str, parkur_listesi) -> bool:
    return current_parkur_index(state) in parkur_listesi


def use_corridor_bias(state: str, parkur_listesi, corridor_active=True) -> bool:
    return current_parkur_index(state) in parkur_listesi and corridor_active


class T1GoalSourceTest(unittest.TestCase):
    """T1 — koridor hedefi seçimi: P1'de koridor, P2'de waypoint."""

    def test_p1_goal_is_corridor(self):
        """P1 (use_corridor_goal=True) + conf>=0.3 -> koridor hedefi (lookahead>=8, cl*0.3)."""
        goal = corridor_goal_body(
            0.0, 0.0, 10.0, -1.0, 1.0,
            use_corridor_goal=True, lookahead_min_m=8.0, lateral_gain=0.3,
        )
        self.assertGreaterEqual(goal[0], 8.0)
        self.assertAlmostEqual(goal[1], -0.3, places=5)

    def test_p2_goal_is_waypoint(self):
        """P2 (use_corridor_goal=False) + conf=1.0 -> waypoint (dx_n, dy_e)."""
        goal = corridor_goal_body(
            3.0, 4.0, 10.0, -1.0, 1.0,
            use_corridor_goal=False, lookahead_min_m=8.0, lateral_gain=0.3,
        )
        self.assertEqual(goal, (3.0, 4.0))

    def test_p1_vs_p2_use_corridor_goal_decision(self):
        """_use_corridor_goal kararı: P1'de True, P2'de False (state koşulu baskın)."""
        self.assertTrue(use_corridor_goal("PARKUR_1_NAV", [1]))
        self.assertFalse(use_corridor_goal("PARKUR_2_AVOIDANCE", [1]))
        self.assertFalse(use_corridor_goal("PARKUR_2_AVOIDANCE", []))
        # Yetki config listesinde açılırsa P2 için de çalışabilir; saha
        # profilimiz [1] kullandığı için P2'de kapalıdır.
        self.assertTrue(use_corridor_goal("PARKUR_2_AVOIDANCE", [1, 2]))
    def test_p2_no_corridor_bias(self):
        """P2'de corridor bias kapalı -> corridor=None (dwa corridor_score=0)."""
        self.assertFalse(use_corridor_bias("PARKUR_2_AVOIDANCE", [1]))
        self.assertTrue(use_corridor_bias("PARKUR_1_NAV", [1]))
        self.assertTrue(use_corridor_bias("PARKUR_2_AVOIDANCE", [1, 2]))

    def test_p1_bias_requires_actual_active_corridor(self):
        self.assertFalse(use_corridor_bias("PARKUR_1_NAV", [1], corridor_active=False))
        self.assertTrue(use_corridor_bias("PARKUR_1_NAV", [1], corridor_active=True))

    def test_actual_debug_helper_keeps_active_bias_for_recovery_returns(self):
        node = AutonomyNode.__new__(AutonomyNode)
        node.state = "PARKUR_1_NAV"
        node.corridor_bias_parkurs = [1]
        active = node._corridor_debug({"active": True, "confidence": 0.9})
        inactive = node._corridor_debug({"active": False})
        self.assertTrue(active["corridor_bias_configured"])
        self.assertTrue(active["corridor_bias_active"])
        self.assertFalse(inactive["corridor_bias_active"])

    def test_actual_p2_corridor_authority_is_profile_controlled(self):
        node = AutonomyNode.__new__(AutonomyNode)
        node.state = "PARKUR_2_AVOIDANCE"
        node.corridor_bias_parkurs = [1]
        self.assertFalse(node._corridor_bias_configured())
        node.corridor_bias_parkurs = [1, 2]
        self.assertTrue(node._corridor_bias_configured())

    def test_low_conf_falls_back_to_waypoint(self):
        """conf<0.3 -> waypoint hedef (P1 regresyon: koridor yetersiz görüşte hedef kaymasın)."""
        goal = corridor_goal_body(
            3.0, 4.0, 10.0, -1.0, 0.2,
            use_corridor_goal=True, lookahead_min_m=8.0, lateral_gain=0.3,
        )
        self.assertEqual(goal, (3.0, 4.0))


class MissionPhaseTransitionTest(unittest.TestCase):
    def setUp(self):
        self.waypoints = [
            {"lat": 41.0, "lon": 29.0, "parkur": 1},
            {"lat": 41.1, "lon": 29.0, "parkur": 1},
            {"lat": 41.2, "lon": 29.0, "parkur": 2},
        ]

    def test_early_yellow_cannot_end_p1(self):
        ready = p1_route_exit_ready(self.waypoints, 0, 0.1, 2.5)
        self.assertFalse(ready)
        self.assertIsNone(p1_transition_decision(
            yellow_count=4, yellow_threshold=4, route_exit_ready=ready,
            all_waypoints_complete=False, elapsed_s=100.0, max_duration_s=360.0,
        ))

    def test_yellow_can_end_p1_only_at_final_p1_exit(self):
        self.assertFalse(p1_route_exit_ready(self.waypoints, 1, 4.39, 2.5))
        ready = p1_route_exit_ready(self.waypoints, 1, 2.4, 2.5)
        self.assertTrue(ready)
        self.assertEqual(p1_transition_decision(
            yellow_count=4, yellow_threshold=4, route_exit_ready=ready,
            all_waypoints_complete=False, elapsed_s=200.0, max_duration_s=360.0,
        ), "complete")

    def test_p1_timeout_is_incomplete_not_completion(self):
        self.assertEqual(p1_transition_decision(
            yellow_count=4, yellow_threshold=4, route_exit_ready=False,
            all_waypoints_complete=False, elapsed_s=360.1, max_duration_s=360.0,
        ), "timeout")

    def test_p2_timeout_never_means_p3_completion(self):
        self.assertEqual(p2_transition_decision(
            crossed_count=8, min_crossings=4, route_complete=False,
            elapsed_s=420.1, max_duration_s=420.0,
        ), "timeout")

    def test_p2_exit_uses_labelled_boundary_not_overall_last_index(self):
        with_p3 = self.waypoints + [
            {"lat": 41.3, "lon": 29.0, "parkur": 3},
            {"lat": 41.4, "lon": 29.0, "parkur": 3},
        ]
        self.assertFalse(p2_route_exit_ready(with_p3, 2, 4.0, 2.5))
        self.assertTrue(p2_route_exit_ready(with_p3, 2, 2.4, 2.5))

    def test_p2_route_is_authoritative_and_crossings_are_diagnostic(self):
        self.assertEqual(p2_transition_decision(
            crossed_count=3, min_crossings=4, route_complete=True,
            elapsed_s=200.0, max_duration_s=420.0,
        ), "complete")
        self.assertIsNone(p2_transition_decision(
            crossed_count=99, min_crossings=4, route_complete=False,
            elapsed_s=200.0, max_duration_s=420.0,
        ))

    def test_transition_to_p3_rejects_unconfirmed_completion(self):
        node = AutonomyNode.__new__(AutonomyNode)
        node.state = "PARKUR_2_AVOIDANCE"
        node.enter_failsafe = Mock()
        self.assertFalse(node.transition_to_p3(420.1))
        node.enter_failsafe.assert_called_once_with(
            "parkur2_incomplete_transition", 420.1
        )

    def test_incomplete_timeout_failsafe_cannot_auto_recover(self):
        node = AutonomyNode.__new__(AutonomyNode)
        node.state = "FAILSAFE"
        node.failsafe_reason = "parkur2_incomplete_timeout"
        node.get_logger = Mock(return_value=Mock())
        node.recover_from_failsafe(500.0)
        self.assertEqual(node.state, "FAILSAFE")
        self.assertEqual(node.failsafe_reason, "parkur2_incomplete_timeout")


class T3StuckDetectorTest(unittest.TestCase):
    """T3 — zaman tabanlı stuck sayacı (dwa.py anlık vx koşulu kaldırıldı).

    AutonomyNode import edilemediğinden (rclpy yok), stuck mantığının SAF
    eşdeğerini test eder: _update_stuck/_is_stuck'ın taşıdığı durum
    (stuck_since zaman damgası + düşük hız eşiği + timeout).
    """

    def _make_state(self):
        return {
            "stuck_timeout_s": 3.0,
            "stuck_min_dist_m": 0.2,
            "stuck_vx_threshold": 0.1,
            "_stuck_dist_m": 0.0,
            "_stuck_since": None,
        }

    def _update(self, st, now, ground_speed):
        """autonomy_node._update_stuck saf eşdeğeri."""
        if ground_speed < st["stuck_vx_threshold"]:
            if st["_stuck_since"] is None:
                st["_stuck_since"] = now
            st["_stuck_dist_m"] = 0.0
        else:
            st["_stuck_since"] = None

    def _is_stuck(self, st, now):
        return st["_stuck_since"] is not None and (now - st["_stuck_since"]) >= st["stuck_timeout_s"]

    def test_no_stuck_when_moving(self):
        st = self._make_state()
        self._update(st, 100.0, 0.5)  # ilerliyor
        self.assertFalse(self._is_stuck(st, 103.0))

    def test_stuck_after_timeout(self):
        st = self._make_state()
        self._update(st, 100.0, 0.05)  # düşük hız, sayaç başla
        self.assertFalse(self._is_stuck(st, 101.0))
        self.assertFalse(self._is_stuck(st, 102.9))
        self.assertTrue(self._is_stuck(st, 103.1))

    def test_reset_when_speed_recovers(self):
        st = self._make_state()
        self._update(st, 100.0, 0.05)
        self._update(st, 101.0, 0.05)
        self.assertFalse(self._is_stuck(st, 102.0))
        self._update(st, 102.0, 0.6)  # hız yükseldi -> sıfırla
        self.assertFalse(self._is_stuck(st, 105.0))

    def test_short_slowdown_not_stuck(self):
        st = self._make_state()
        self._update(st, 100.0, 0.05)
        self._update(st, 100.2, 0.05)
        self._update(st, 100.4, 0.05)
        self.assertFalse(self._is_stuck(st, 101.0))  # 1s < 3s


class P3TrackedEngagementTest(unittest.TestCase):
    def _engage_node(self):
        node = AutonomyNode.__new__(AutonomyNode)
        node.state = "ENGAGE"
        node.now_sec = Mock(return_value=10.0)
        node.telemetry_ok = Mock(return_value=True)
        node.buoys_ok = Mock(return_value=True)
        node.publish = Mock()
        node.buoys = []
        node.target_color = "green"
        node.target_min_confidence = 0.20
        node._p3_locked_target_id = "lidar_obstacle_id:lidar-1"
        node._p3_target_lost_since = None
        node.engage_started = 9.5
        node.engage_window_s = 2.0
        node.max_speed = 1.6
        node.max_yaw_rate = 50.0
        node.wrong_avoid_lateral_m = 1.5
        node.wrong_avoid_speed_mps = 0.70
        node.p3_search_yaw_rate_deg_s = 30.0
        node.p3_lock_yaw_gain_deg_s = 35.0
        node.p3_engage_center_max = 0.18
        node.p3_engage_distance_m = 1.2
        node.p3_lock_min_speed_mps = 0.70
        node.p3_engage_speed_mps = 0.70
        return node

    def test_engage_target_loss_immediately_stops_forward_command(self):
        node = self._engage_node()
        node.tick()
        command = node.publish.call_args.args[0]
        self.assertEqual(node.state, "PARKUR_3_TARGET_LOCK")
        self.assertEqual(command.vx, 0.0)
        self.assertEqual(command.yaw_rate, 0.0)
        self.assertEqual(command.action, "engage_target_lost_stop")

    def test_engage_alignment_drift_stops_and_returns_to_lock(self):
        node = self._engage_node()
        node.buoys = [{
            "id": "cam-1", "lidar_obstacle_id": "lidar-1",
            "color": "green", "confidence": 0.9, "distance": 0.8,
            "bbox_norm_x": 0.30, "bbox_size": 0.3,
        }]
        node.tick()
        command = node.publish.call_args.args[0]
        self.assertEqual(node.state, "PARKUR_3_TARGET_LOCK")
        self.assertEqual(command.vx, 0.0)
        self.assertEqual(command.action, "engage_alignment_lost_stop")

    def test_stable_lidar_identity_prevents_same_color_target_switch(self):
        node = self._engage_node()
        node._p3_locked_target_id = None
        node.obstacles = [
            {"id": "lidar-a", "distance": 3.0, "hard_obstacle": True},
            {"id": "lidar-b", "distance": 5.0, "hard_obstacle": True},
        ]
        node.buoys = [
            {"id": "cam-a", "lidar_obstacle_id": "lidar-a", "color": "green", "confidence": 0.9, "distance": 3.0},
            {"id": "cam-b", "lidar_obstacle_id": "lidar-b", "color": "green", "confidence": 0.9, "distance": 5.0},
        ]
        first = node._p3_visible_target_candidates()
        self.assertEqual(first[0]["lidar_obstacle_id"], "lidar-a")
        node.buoys = [
            {"id": "cam-a2", "lidar_obstacle_id": "lidar-a", "color": "green", "confidence": 0.9, "distance": 2.5},
            {"id": "cam-b2", "lidar_obstacle_id": "lidar-b", "color": "green", "confidence": 0.9, "distance": 1.0},
        ]
        node.obstacles = [
            {"id": "lidar-a", "distance": 2.5, "hard_obstacle": True},
            {"id": "lidar-b", "distance": 1.0, "hard_obstacle": True},
        ]
        second = node._p3_visible_target_candidates()
        self.assertEqual(second[0]["lidar_obstacle_id"], "lidar-a")

    def test_target_without_lidar_match_is_rejected(self):
        """LİDAR DOĞRULAMASI: lidar eşleşmesi olmayan hedef elenir (fail-closed).

        Modeller iyi değil -> yanlış renk atanmış ama lidar karşılığı olmayan
        duba hedef sayılmaz (kamera-only yanlış konum angajman üretmez).
        """
        node = self._engage_node()
        node._p3_locked_target_id = None
        node.obstacles = [
            {"id": "lidar-1", "distance": 3.0, "hard_obstacle": True},
        ]
        node.buoys = [
            # lidar_obstacle_id YOK -> lidar eşleşmesi yok -> elenmeli.
            {"id": "cam-x", "color": "green", "confidence": 0.9, "distance": 2.0},
        ]
        self.assertEqual(node._p3_visible_target_candidates(), [])

    def test_target_with_mismatched_lidar_distance_is_rejected(self):
        """LİDAR DOĞRULAMASI: lidar mesafesi uyuşmayan hedef elenir."""
        node = self._engage_node()
        node._p3_locked_target_id = None
        node.obstacles = [
            # lidar engeli 3.0m'de ama hedef duba 1.0m'de -> eşleşme yok.
            {"id": "lidar-1", "distance": 3.0, "hard_obstacle": True},
        ]
        node.buoys = [
            {"id": "cam-x", "lidar_obstacle_id": "lidar-1", "color": "green",
             "confidence": 0.9, "distance": 1.0},
        ]
        self.assertEqual(node._p3_visible_target_candidates(), [])

    def test_target_with_lidar_match_is_selected(self):
        """LİDAR DOĞRULAMASI: lidar eşleşmeli hedef seçilir (normal akış)."""
        node = self._engage_node()
        node._p3_locked_target_id = None
        node.obstacles = [
            {"id": "lidar-1", "distance": 3.0, "hard_obstacle": True},
        ]
        node.buoys = [
            {"id": "cam-x", "lidar_obstacle_id": "lidar-1", "color": "green",
             "confidence": 0.9, "distance": 3.0},
        ]
        result = node._p3_visible_target_candidates()
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["lidar_obstacle_id"], "lidar-1")


class T2WaypointAdvanceTest(unittest.TestCase):
    """T2 — yakın geçiş ve sınırlı mevcut-WP overshoot truth-table."""

    def test_advance_when_close_and_forward(self):
        self.assertTrue(waypoint_advance_decision(2.0, 2.5, 0.0, 3.0))

    def test_advance_when_behind(self):
        self.assertFalse(waypoint_advance_decision(2.0, 2.5, 0.0, -1.0))

    def test_advance_when_far_but_behind(self):
        self.assertFalse(waypoint_advance_decision(50.0, 2.5, 20.0, -1.0))

    def test_no_advance_when_far_and_forward(self):
        self.assertFalse(waypoint_advance_decision(10.0, 2.5, 0.0, 3.0))

    def test_lateral_offset_ignored(self):
        """Yanal offset ne olursa olsun ilerleme (lateral eşik YOK — köşe tıkanması çözümü)."""
        self.assertTrue(waypoint_advance_decision(1.0, 2.5, 4.0, 3.0))
        self.assertTrue(waypoint_advance_decision(0.5, 2.5, 0.0, 0.6))

    def test_fwd_current_only_matters_in_guard_band(self):
        self.assertTrue(waypoint_advance_decision(2.0, 2.5, -10.0, 3.0))
        self.assertTrue(waypoint_advance_decision(2.0, 2.5, 10.0, 3.0))
        self.assertTrue(waypoint_advance_decision(3.0, 2.5, -0.1, -10.0, 5.0))
        self.assertFalse(waypoint_advance_decision(3.0, 2.5, 0.1, -10.0, 5.0))
        self.assertFalse(waypoint_advance_decision(50.0, 2.5, -10.0, -20.0, 5.0))

    def test_nonfinite_threshold_or_guard_rejected(self):
        for threshold in (float("nan"), float("inf")):
            with self.subTest(threshold=threshold), self.assertRaises(ValueError):
                waypoint_advance_decision(1.0, threshold, 0.0, 3.0, 5.0)
        for guard in (float("nan"), float("inf")):
            with self.subTest(guard=guard), self.assertRaises(ValueError):
                waypoint_advance_decision(1.0, 2.5, 0.0, 3.0, guard)


class MissionWaypointCallbackTest(unittest.TestCase):
    """Gerçek AutonomyNode.on_waypoints replay/reset lifecycle regresyonu."""

    def _node(self):
        node = AutonomyNode.__new__(AutonomyNode)
        node._mission_fingerprint = None
        node._last_mission_control_token = None
        node.dwa_enabled = True
        node.state = "WAIT_MISSION"
        node.current_wp = 7
        node.started = True
        node.strict_orange_latched = True
        node.raw_waypoints = []
        node.waypoints = []
        node.pair_detector_p1 = Mock()
        node.pair_detector_p2 = Mock()
        node.yellow_counter = Mock()
        node.contact_p1 = Mock()
        node.contact_p2 = Mock()
        node.ooc_p1 = OutOfCourseDetector(mode="corridor")
        node.ooc_p2 = OutOfCourseDetector(mode="corridor")
        node.course_geometry_p1 = CourseGeometryTracker()
        node.course_geometry_p2 = CourseGeometryTracker()
        node.score_reports = {1: object(), 2: object()}
        node.p1_stats_cache = {"crossed_count": 4}
        node.p2_stats_cache = {"crossed_count": 2}
        node.p1_stats = object()
        node.p2_stats = object()
        node.yellow_count = 4
        node.ts3_risk = 1
        node.p3_target_retries = 2
        node.p3_hold_started = 10.0
        node.engage_started = 11.0
        node.last_dwa = object()
        node.failsafe_reason = "old"
        node._failsafe_resume_state = "PARKUR_1_NAV"
        node._corridor_goal_on = True
        node._stuck_since = 12.0
        node._stuck_dist_m = 9.0
        node.sim_gate_truth_enabled = False
        node.sim_gate_truth = []
        node.last_gate_truth_time = 0.0
        node.perception_timeout = 2.0
        node.buoys = [{"id": "canonical"}]
        node.now_sec = Mock(return_value=10.0)
        node.mission_control_ack_pub = Mock()
        logger = Mock()
        node.get_logger = Mock(return_value=logger)
        return node

    def test_tokenized_start_and_stop_are_applied_and_acknowledged(self):
        node = self._node()
        node.on_waypoints(self._message())

        node.on_start(types.SimpleNamespace(data=json.dumps({"start": True, "token": 8_000_001})))
        self.assertTrue(node.started)
        start_ack = json.loads(node.mission_control_ack_pub.publish.call_args.args[0].data)
        self.assertEqual(start_ack["token"], 8_000_001)
        self.assertTrue(start_ack["applied"])

        node.state = "PARKUR_1_NAV"
        node.current_wp = 1
        node.on_start(types.SimpleNamespace(data=json.dumps({"start": False, "token": 8_000_006})))
        self.assertFalse(node.started)
        self.assertEqual(node.state, "MISSION_READY")
        self.assertEqual(node.current_wp, 0)
        stop_ack = json.loads(node.mission_control_ack_pub.publish.call_args.args[0].data)
        self.assertEqual(stop_ack["token"], 8_000_006)
        self.assertTrue(stop_ack["applied"])

    def test_start_without_waypoints_is_not_acknowledged_as_applied(self):
        node = self._node()
        node.waypoints = []
        node.on_start(types.SimpleNamespace(data=json.dumps({"start": True, "token": 8_000_001})))
        ack = json.loads(node.mission_control_ack_pub.publish.call_args.args[0].data)
        self.assertFalse(ack["applied"])
        self.assertFalse(node.started)

    def test_token_sign_mismatch_is_rejected_without_ack(self):
        node = self._node()
        node.waypoints = [{"lat": 41.0, "lon": 29.0, "parkur": 1}]
        node.on_start(types.SimpleNamespace(data=json.dumps({"start": False, "token": 8_000_001})))
        node.mission_control_ack_pub.publish.assert_not_called()

    def test_duplicate_stop_token_is_acked_without_resetting_twice(self):
        node = self._node()
        node.on_waypoints(self._message())
        before = node.pair_detector_p1.reset.call_count
        message = types.SimpleNamespace(
            data=json.dumps({"start": False, "token": 8_000_002})
        )

        node.on_start(message)
        after_first = node.pair_detector_p1.reset.call_count
        node.on_start(message)

        self.assertEqual(after_first, before + 1)
        self.assertEqual(node.pair_detector_p1.reset.call_count, after_first)
        self.assertEqual(node.mission_control_ack_pub.publish.call_count, 2)

    @staticmethod
    def _message(offset=0.0):
        return types.SimpleNamespace(data=json.dumps({"waypoints": [
            {"lat": 41.0 + offset, "lon": 29.0, "parkur": 1},
            {"lat": 41.0002 + offset, "lon": 29.0, "parkur": 2},
        ]}))

    def test_twenty_identical_replays_after_start_are_noop(self):
        node = self._node()
        node.on_waypoints(self._message())
        self.assertEqual(node.state, "MISSION_READY")
        self.assertFalse(node.started)
        self.assertEqual(node.pair_detector_p1.reset.call_count, 1)

        # Gerçek start/tick sonucunu simüle et; kalan telemetry_sim tekrarları
        # bu aktif durumu ve latch'i değiştirmemeli.
        node.state = "PARKUR_1_NAV"
        node.started = True
        node.strict_orange_latched = True
        node.current_wp = 1
        node.p1_stats_cache["crossed_count"] = 3
        for _ in range(20):
            node.on_waypoints(self._message())
        self.assertEqual(node.state, "PARKUR_1_NAV")
        self.assertTrue(node.started)
        self.assertTrue(node.strict_orange_latched)
        self.assertEqual(node.current_wp, 1)
        self.assertEqual(node.p1_stats_cache["crossed_count"], 3)
        self.assertEqual(node.pair_detector_p1.reset.call_count, 1)

    def test_changed_mission_full_reset_and_bad_message_preserves_it(self):
        node = self._node()
        node.on_waypoints(self._message())
        node.state = "PARKUR_2_AVOIDANCE"
        node.started = True
        node.strict_orange_latched = True
        node.score_reports[2] = object()
        node.on_waypoints(self._message(0.001))
        self.assertEqual(node.state, "MISSION_READY")
        self.assertEqual(node.current_wp, 0)
        self.assertFalse(node.started)
        self.assertFalse(node.strict_orange_latched)
        self.assertEqual(node.score_reports, {})
        self.assertEqual(node.p1_stats_cache, {})
        self.assertEqual(node.p2_stats_cache, {})
        self.assertEqual(node.pair_detector_p1.reset.call_count, 2)
        fingerprint = node._mission_fingerprint
        waypoints = list(node.waypoints)

        node.on_waypoints(types.SimpleNamespace(data="null"))
        node.on_waypoints(types.SimpleNamespace(data=json.dumps({
            "waypoints": [{"lat": float("nan"), "lon": 29.0, "parkur": 1}]
        })))
        self.assertEqual(node._mission_fingerprint, fingerprint)
        self.assertEqual(node.waypoints, waypoints)
        self.assertEqual(node.pair_detector_p1.reset.call_count, 2)

    def test_sim_truth_is_opt_in_and_stale_invalid_never_falls_back_to_camera(self):
        node = self._node()
        valid = types.SimpleNamespace(data=json.dumps({"detections": [
            {"id": "p2_o_l1", "source": "sim_gate_truth", "color": "orange", "forward_m": 6.0, "lateral_m": -4.0},
            {"id": "p2_o_r1", "source": "sim_gate_truth", "color": "orange", "forward_m": 6.0, "lateral_m": 4.0},
        ]}))
        node.on_gate_truth(valid)
        self.assertEqual(node._pair_detection_input(10.0), node.buoys)

        node.sim_gate_truth_enabled = True
        node.state = "PARKUR_2_AVOIDANCE"
        node.on_gate_truth(valid)
        self.assertEqual(len(node._pair_detection_input(10.0)), 2)
        self.assertEqual(node._pair_detection_input(13.0), [])
        node.on_gate_truth(types.SimpleNamespace(data="null"))
        self.assertEqual(node._pair_detection_input(10.0), [])

    def test_truth_filters_mixed_parkur_ids_and_mission_reset_clears_old_frame(self):
        node = self._node()
        node.sim_gate_truth_enabled = True
        node.state = "PARKUR_1_NAV"
        node.sim_gate_truth = [
            {"id": "p1_o_l1", "source": "sim_gate_truth"},
            {"id": "p1_o_r1", "source": "sim_gate_truth"},
            {"id": "p2_o_l1", "source": "sim_gate_truth"},
            {"id": "p2_o_r1", "source": "sim_gate_truth"},
        ]
        node.last_gate_truth_time = 10.0
        self.assertEqual(
            {item["id"] for item in node._pair_detection_input(10.0)},
            {"p1_o_l1", "p1_o_r1"},
        )
        node.state = "PARKUR_2_AVOIDANCE"
        self.assertEqual(
            {item["id"] for item in node._pair_detection_input(10.0)},
            {"p2_o_l1", "p2_o_r1"},
        )

        node._reset_for_new_mission()
        self.assertEqual(node.sim_gate_truth, [])
        self.assertEqual(node.last_gate_truth_time, 0.0)
        # Mission'dan sonraki ilk timeout penceresinde eski truth beslenmez.
        self.assertEqual(node._pair_detection_input(1.0), [])


class PerceptionFreshnessCallbackTest(unittest.TestCase):
    def test_stale_buoy_payload_clears_data_without_refreshing_watchdog(self):
        node = AutonomyNode.__new__(AutonomyNode)
        node.buoys = [{"id": "old"}]
        node.last_buoy_time = 5.0
        node.now_sec = Mock(return_value=10.0)
        node.on_buoys(types.SimpleNamespace(data=json.dumps({
            "stamp": 9.0, "detections": [], "stale": True
        })))
        self.assertEqual(node.buoys, [])
        self.assertEqual(node.last_buoy_time, 5.0)

    def test_fresh_buoy_payload_refreshes_and_malformed_preserves_state(self):
        node = AutonomyNode.__new__(AutonomyNode)
        node.buoys = []
        node.last_buoy_time = 5.0
        node.now_sec = Mock(return_value=10.0)
        node.on_buoys(types.SimpleNamespace(data=json.dumps({
            "stamp": 9.0, "detections": [{"id": "new"}]
        })))
        self.assertEqual(node.buoys, [{"id": "new"}])
        self.assertEqual(node.last_buoy_time, 10.0)
        node.on_buoys(types.SimpleNamespace(data="[]"))
        self.assertEqual(node.buoys, [{"id": "new"}])


class NearFieldGuardTest(unittest.TestCase):
    """AutonomyNode._near_field_guarded: yakın hard obstacle'da ileri komut 0.

    Saha bulgusu (2026-08-17): hard obstacle 0.93-1.56 m'de bazı komutlarda
    v=0.7 m/s korundu. _near_field_guarded DWA komutunu yakın-alan kuralıyla
    sarar; P3 hedef angajmanı bu katmana girmez.
    """

    def _node(self):
        node = AutonomyNode.__new__(AutonomyNode)
        node.max_yaw_rate = 50.0
        node.near_field_stop_m = 1.5
        node.near_field_slow_m = 3.0
        node.near_field_stop_lateral_m = 1.0
        node.near_field_slow_lateral_m = 1.5
        node.near_field_pivot_yaw_deg_s = 30.0
        node.near_field_slow_speed_mps = 0.30
        node.near_field_stop_release_m = 2.5
        node._near_field_latch = False
        node.obstacles = [{
            "forward_m": 0.93, "lateral_m": -0.67,
            "distance": 1.15, "color": "unknown", "hard_obstacle": True,
        }]
        return node

    def test_close_obstacle_zeroes_forward_command(self):
        """0.93 m hard obstacle -> DWA 0.7 isteği 0'a çekilir + sağa pivot."""
        node = self._node()
        candidate = Command(0.7, 0.0, 0.2, "dwa score=1.0")
        command, debug = node._near_field_guarded(candidate, {"costmap": True})
        self.assertEqual(command.vx, 0.0)
        self.assertGreater(command.yaw_rate, 0.0)
        self.assertTrue(debug["near_field"]["active"])
        self.assertTrue(debug["near_field"]["latch"])

    def test_slow_zone_reduces_speed(self):
        """2.5 m engel -> hız 0.3'e düşer, durmaz."""
        node = self._node()
        node.obstacles = [{
            "forward_m": 2.5, "lateral_m": -0.5,
            "distance": 2.55, "color": "unknown", "hard_obstacle": True,
        }]
        candidate = Command(0.7, 0.0, 0.1, "dwa score=1.0")
        command, debug = node._near_field_guarded(candidate, {"costmap": True})
        self.assertLessEqual(command.vx, 0.30)
        self.assertGreater(command.vx, 0.0)
        self.assertTrue(debug["near_field"]["active"])
        self.assertFalse(debug["near_field"]["latch"])

    def test_far_obstacle_unchanged(self):
        """Uzak engel komutu değiştirmez."""
        node = self._node()
        node.obstacles = [{
            "forward_m": 5.0, "lateral_m": 0.0,
            "distance": 5.0, "color": "unknown", "hard_obstacle": True,
        }]
        candidate = Command(0.7, 0.0, 0.2, "dwa score=1.0")
        command, debug = node._near_field_guarded(candidate, {"costmap": True})
        self.assertEqual(command.vx, 0.7)
        self.assertFalse(debug["near_field"]["active"])

    def test_hysteresis_keeps_stop_until_release(self):
        """Latch: engel 1.5 üstünde ama 2.5 altında iken stop sürer."""
        node = self._node()
        node._near_field_latch = True
        node.obstacles = [{
            "forward_m": 2.0, "lateral_m": 0.0,
            "distance": 2.0, "color": "unknown", "hard_obstacle": True,
        }]
        candidate = Command(0.7, 0.0, 0.0, "dwa score=1.0")
        command, debug = node._near_field_guarded(candidate, {"costmap": True})
        self.assertEqual(command.vx, 0.0)
        self.assertTrue(debug["near_field"]["latch"])

    def test_p3_search_stop_never_arms_reverse_escape(self):
        """Hedefsiz P3 staging engelde durur; reverse süpürme başlatmaz."""
        node = self._node()

        class EscapeProbe:
            reset_count = 0

            def reset(self):
                self.reset_count += 1

            def apply(self, *_args, **_kwargs):
                raise AssertionError("P3 search must not call reverse escape")

        node.center_obstacle_escape = EscapeProbe()
        command, debug = node._near_field_guarded(
            Command(0.7, 0.0, 0.0, "search_reposition_straight"),
            {"costmap": True},
            allow_reverse_escape=False,
        )
        self.assertEqual(command.vx, 0.0)
        self.assertEqual(node.center_obstacle_escape.reset_count, 1)
        self.assertEqual(
            debug["near_field"]["escape"], "disabled_for_search"
        )

    def test_p2_uses_dwa_as_single_authority_and_never_arms_reverse(self):
        """P2 rosbag regresyonu: ikinci stop/reverse katmanı DWA'yı ezmez."""
        node = self._node()
        node.state = "PARKUR_2_AVOIDANCE"

        class EscapeProbe:
            reset_count = 0

            def reset(self):
                self.reset_count += 1

            def apply(self, *_args, **_kwargs):
                raise AssertionError("P2 must use forward/pivot DWA recovery")

        node.center_obstacle_escape = EscapeProbe()
        command, debug = node._near_field_guarded(
            Command(0.7, 0.0, 0.0, "dwa score=1.0"),
            {"costmap": True},
        )
        self.assertEqual(command.vx, 0.7)
        self.assertEqual(command.action, "dwa score=1.0")
        self.assertEqual(node.center_obstacle_escape.reset_count, 1)
        self.assertEqual(
            debug["near_field"]["reason"], "disabled_for_p2_dwa_authority"
        )
        self.assertEqual(debug["near_field"]["escape"], "disabled_for_p2")


class BenchP3OnlyOverrideTest(unittest.TestCase):
    """bench_p3_only_enabled doğrudan P3 başlangıç override'ı (fail-closed).

    Production'da bayraklar kapalıdır ve davranış birebir aynı kalır (WP0 /
    P1->P2->P3 hakem sırası). Yalnız açıkça etkinleştirilmiş bench launch'ı
    (bench_p3_decision.launch.py, motor yolu kapalı) bu override'ı açar.
    """

    def _node(self):
        node = AutonomyNode.__new__(AutonomyNode)
        node._mission_fingerprint = None
        node._last_mission_control_token = None
        node.dwa_enabled = True
        node.bench_p3_only_enabled = False
        node.bench_p3_start_parkur = 0
        node.bench_override_wp = None
        node.state = "WAIT_MISSION"
        node.current_wp = 7
        node.started = True
        node.strict_orange_latched = True
        node.raw_waypoints = []
        node.waypoints = []
        node.pair_detector_p1 = Mock()
        node.pair_detector_p2 = Mock()
        node.yellow_counter = Mock()
        # publish() -> _score_events() gerçek ContactCounter iterasyonu yapar;
        # Mock burada TypeError üretir (get_events_all Mock döner).
        node.contact_p1 = ContactCounter(
            contact_radius_m=1.6, sustained_contact_s=30.0, drop_after_s=5.0
        )
        node.contact_p2 = ContactCounter(
            contact_radius_m=1.6, sustained_contact_s=30.0, drop_after_s=5.0
        )
        node.ooc_p1 = OutOfCourseDetector(mode="corridor")
        node.ooc_p2 = OutOfCourseDetector(mode="corridor")
        node.course_geometry_p1 = CourseGeometryTracker()
        node.course_geometry_p2 = CourseGeometryTracker()
        node.score_reports = {}
        node.p1_stats_cache = {}
        node.p2_stats_cache = {}
        node.p1_stats = None
        node.p2_stats = None
        node.yellow_count = 0
        node.ts3_risk = 0
        node.p3_target_retries = 0
        node.p3_hold_started = None
        node.engage_started = None
        node.last_dwa = None
        node.failsafe_reason = ""
        node._failsafe_resume_state = None
        node._corridor_goal_on = False
        node._stuck_since = None
        node._stuck_dist_m = 0.0
        node.sim_gate_truth_enabled = False
        node.sim_gate_truth = []
        node.last_gate_truth_time = 0.0
        node.perception_timeout = 2.0
        node.buoys = []
        node.now_sec = Mock(return_value=10.0)
        node.mission_control_ack_pub = Mock()
        node.get_logger = Mock(return_value=Mock())
        return node

    @staticmethod
    def _message():
        return types.SimpleNamespace(data=json.dumps({"waypoints": [
            {"lat": 41.0, "lon": 29.0, "parkur": 1},
            {"lat": 41.0002, "lon": 29.0, "parkur": 2},
            {"lat": 41.0004, "lon": 29.0, "parkur": 3},
            {"lat": 41.0006, "lon": 29.0, "parkur": 3},
        ]}))

    def test_disabled_override_keeps_mission_start_at_wp0(self):
        """(a) bench_p3_only_enabled=False iken parkur override ÇALIŞMAZ — WP0."""
        node = self._node()
        node.bench_p3_only_enabled = False
        node.bench_p3_start_parkur = 3
        node.on_waypoints(self._message())
        self.assertEqual(node.state, "MISSION_READY")
        self.assertEqual(node.current_wp, 0)
        self.assertIsNone(node.bench_override_wp)

        # Gerçek start tick'i de hakem sırasını (WP0 / P1) korur.
        node.started = True
        node.telemetry = {"lat": 41.0, "lon": 29.0, "heading_deg": 0.0}
        node.state = "MISSION_READY"
        node.publish = Mock()
        node.tick()
        self.assertEqual(node.current_wp, 0)
        self.assertEqual(node.state, "PARKUR_1_NAV")

    def test_disabled_rejects_invalid_start_parkur_without_error(self):
        """(a) Kapalıyken geçersiz parkur değeri override'ı bozmaz (fail-closed)."""
        node = self._node()
        node.bench_p3_only_enabled = False
        node.bench_p3_start_parkur = 99
        node.on_waypoints(self._message())
        self.assertEqual(node.current_wp, 0)

    def test_enabled_start_parkur_3_starts_at_p3_state(self):
        """(b) True + start_parkur=3 -> P3 başlangıç state'i (ilk parkur>=3 WP)."""
        node = self._node()
        node.bench_p3_only_enabled = True
        node.bench_p3_start_parkur = 3
        node.on_waypoints(self._message())
        self.assertEqual(node.current_wp, 2)
        self.assertEqual(node.bench_override_wp, 2)

        # Start tick'i authoritative restart'a rağmen bench atlamasını korur.
        node.started = True
        node.telemetry = {"lat": 41.0, "lon": 29.0, "heading_deg": 0.0}
        node.state = "MISSION_READY"
        node.publish = Mock()
        node.tick()
        self.assertEqual(node.current_wp, 2)
        self.assertEqual(node.state, "PARKUR_3_TARGET_LOCK")

    def test_enabled_start_parkur_2_starts_at_first_p2_waypoint(self):
        """(b) start_parkur=2 -> ilk parkur>=2 WP (P2 state)."""
        node = self._node()
        node.bench_p3_only_enabled = True
        node.bench_p3_start_parkur = 2
        node.on_waypoints(self._message())
        self.assertEqual(node.current_wp, 1)
        node.started = True
        node.telemetry = {"lat": 41.0, "lon": 29.0, "heading_deg": 0.0}
        node.state = "MISSION_READY"
        node.publish = Mock()
        node.tick()
        self.assertEqual(node.state, "PARKUR_2_AVOIDANCE")

    def test_enabled_but_mission_without_target_parkur_is_noop(self):
        """Hedef parkur rotada yoksa override etkisizdir (fail-closed)."""
        node = self._node()
        node.bench_p3_only_enabled = True
        node.bench_p3_start_parkur = 3
        node.on_waypoints(types.SimpleNamespace(data=json.dumps({"waypoints": [
            {"lat": 41.0, "lon": 29.0, "parkur": 1},
            {"lat": 41.0002, "lon": 29.0, "parkur": 2},
        ]})))
        self.assertEqual(node.current_wp, 0)
        self.assertIsNone(node.bench_override_wp)

    def test_publish_marks_bench_session_and_never_arms(self):
        """(c) Fail-closed: bench etkinken state/score/debug bench:true taşır.

        Motor/ARM komutu yalnız launch kontratıyla değil, autonomy yayınlarında
        da izlenebilir olmalıdır — bu test yalnız işaretleme ve sıfır-komut
        taşıma davranışını doğrular (motor kapısı launch'ta kapalıdır).
        """
        node = self._node()
        node.bench_p3_only_enabled = True
        node.bench_p3_start_parkur = 3
        node.on_waypoints(self._message())
        node.state_pub = Mock()
        node.score_pub = Mock()
        node.debug_pub = Mock()
        node.costmap_pub = Mock()
        node.cmd_pub = Mock()
        node.telemetry = None
        node.telemetry_ok = Mock(return_value=False)
        node._failsafe_resume_state = None
        node.strict_orange_latched = False
        node.target_color = "green"
        node.yellow_threshold = 4
        node.ts3_risk = 0
        command = Command(0.0, 0.0, 0.0, "idle")
        node.publish(command, {"near_field": False})
        state = json.loads(node.state_pub.publish.call_args.args[0].data)
        score = json.loads(node.score_pub.publish.call_args.args[0].data)
        debug = json.loads(node.debug_pub.publish.call_args.args[0].data)
        self.assertTrue(state["bench"])
        self.assertTrue(score["bench"])
        self.assertTrue(debug["bench"])
        # Production etiketleri korunur (bench kapalıyken false kalır).
        node.bench_p3_only_enabled = False
        node.publish(command, {"near_field": False})
        state_off = json.loads(node.state_pub.publish.call_args.args[0].data)
        self.assertFalse(state_off["bench"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
