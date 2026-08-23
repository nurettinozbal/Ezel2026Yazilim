"""Mission state machine for the IDA autonomy stack.

States: WAIT_MISSION, MISSION_READY, PARKUR_1_NAV, PARKUR_2_AVOIDANCE,
PARKUR_3_TARGET_LOCK, ENGAGE, COMPLETE, FAILSAFE.

All time handling uses the ROS2 clock (self.get_clock().now()) so the state
machine stays deterministic and testable; wall-clock time is never used.
"""

import math
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ida_planning.contracts import VALID_TARGET_COLORS, dumps, loads
from ida_planning.behavior import (
    CenterObstacleEscapeController,
    MODE_ALIGN,
    MODE_NEAR_STOP,
    MODE_RECOVERY,
    MODE_SLOW,
    NavigationBehaviorArbiter,
    NavigationDecision,
    motion_expected_for_stuck,
    nearest_forward_hard_obstacle_m,
)
from ida_planning.costmap import CostMap
from ida_planning.dwa import DEFAULT_ACCEL, DEFAULT_SIM, DEFAULT_W, DwaCommand, DwaPlanner
from ida_planning.geo import (
    bearing_deg,
    clamp,
    haversine_m,
    latlon_to_local_m,
    local_m_to_latlon,
    normalize_angle_deg,
)
from ida_planning.planner import (
    Command,
    CourseGeometryStatus,
    CourseGeometryTracker,
    PairCrossingDetector,
    YellowGateCounter,
    apply_corridor_bias,
    apply_obstacle_avoidance,
    build_corridor_info,
    build_route_corridor_info,
    corridor_goal_body,
    course_geometry_parkur,
    is_last_waypoint_index,
    mission_waypoint_fingerprint,
    near_field_command,
    p1_route_exit_ready,
    p1_transition_decision,
    p2_route_exit_ready,
    p2_transition_decision,
    sanitize_mission_waypoints,
    split_parkur_routes,
    target_engagement_command,
    task_timeout_expired,
    update_strict_orange_latch,
    waypoint_advance_decision,
    waypoint_guidance_index,
    waypoint_command,
    wrong_target_risk,
)
from ida_planning.speed_safety import speed_for_safety_radius
from ida_planning.scoring.contact import ContactCounter
from ida_planning.scoring.out_of_course import OutOfCourseDetector
from ida_planning.scoring.score import ScoreCalculator

MISSION_MAILBOX_BASE = 8_000_000
MISSION_CONTROL_MAX_SEQUENCE = 1_999_999
MISSION_INCOMPLETE_FAILSAFE_REASONS = frozenset({
    "parkur1_incomplete_timeout",
    "parkur2_incomplete_timeout",
    "parkur2_incomplete_transition",
})


def decode_mission_command_token(value: Any) -> tuple[int, bool] | None:
    """Decode only SCR_USER6 command states; ACK/invalid values are rejected."""
    if isinstance(value, bool):
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    token = int(round(numeric)) if math.isfinite(numeric) else -1
    maximum = MISSION_MAILBOX_BASE + 4 * MISSION_CONTROL_MAX_SEQUENCE
    if abs(numeric - token) > 1e-6 or not MISSION_MAILBOX_BASE <= token <= maximum:
        return None
    residue = (token - MISSION_MAILBOX_BASE) % 4
    if residue not in {1, 2}:
        return None
    return token, residue == 1


def p3_initial_hold_active(now_s: float, entered_at_s: float, hold_s: float) -> bool:
    """Return whether the one P3-entry hold window is still active.

    Search-cycle and target-loss timestamps are intentionally absent, so
    reacquisition can never restart this hold.
    """

    values = (now_s, entered_at_s, hold_s)
    if not all(math.isfinite(float(value)) for value in values) or hold_s < 0.0:
        raise ValueError("P3 initial hold values must be finite and hold non-negative")
    elapsed = float(now_s) - float(entered_at_s)
    # ROS clock rollback is fail-safe: remain stopped until time catches up.
    return elapsed < 0.0 or elapsed < float(hold_s)


def p3_scan_progress(
    accumulated_deg: float, last_heading_deg: float, heading_deg: float
) -> tuple[float, float]:
    """Accumulate actual positive rotation; wave-driven reversal removes progress."""

    values = (accumulated_deg, last_heading_deg, heading_deg)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("P3 scan headings must be finite")
    delta = normalize_angle_deg(float(heading_deg) - float(last_heading_deg))
    return max(0.0, float(accumulated_deg) + delta), float(heading_deg) % 360.0


def p3_target_loss_grace_command(
    last_command: Optional[Command],
    last_distance_m: Optional[float],
    align_distance_m: float,
) -> Command:
    """Retain a bounded last bearing during a brief P3 model dropout."""

    if last_command is None:
        return Command(0.0, 0.0, 0.0, "target_loss_grace")
    try:
        # Distance inputs remain part of the public helper contract so old
        # callers fail closed on malformed state, but target loss never keeps
        # blind forward thrust.  The last bounded yaw is enough to bridge a
        # short detector dropout without producing the observed sweep motion.
        distance = float(last_distance_m)
        align_distance = float(align_distance_m)
        yaw = float(last_command.yaw_rate)
    except (TypeError, ValueError, OverflowError):
        return Command(0.0, 0.0, 0.0, "target_loss_grace")
    if not all(math.isfinite(value) for value in (distance, align_distance, yaw)):
        return Command(0.0, 0.0, 0.0, "target_loss_grace")
    return Command(0.0, 0.0, yaw, "target_loss_reacquire")


def costmap_inflation_speed_for_state(
    state: str, plan_speed_mps: float, p2_speed_cap_mps: float
) -> float:
    """Select the speed used only for costmap obstacle inflation.

    DWA already sweeps every candidate trajectory with the vehicle footprint.
    In P2, applying the full stopping-distance inflation to every obstacle as
    well can close a valid eight-metre corridor.  The field profile therefore
    bounds only P2's *global* inflation speed; P1/P3 behavior is unchanged.
    A zero cap retains the fixed hull + base safety radius while DWA continues
    to enforce collision-free motion over its configured time horizon.
    """

    speed = float(plan_speed_mps)
    cap = float(p2_speed_cap_mps)
    if not math.isfinite(speed) or not math.isfinite(cap):
        raise ValueError("costmap inflation speeds must be finite")
    speed = max(0.0, speed)
    cap = max(0.0, cap)
    if str(state) == "PARKUR_2_AVOIDANCE":
        return min(speed, cap)
    return speed


class AutonomyNode(Node):
    def __init__(self) -> None:
        super().__init__("ida_autonomy")

        self.declare_parameter("loop_hz", 10.0)
        self.declare_parameter("waypoint_threshold_m", 2.5)
        self.declare_parameter("waypoint_overshoot_guard_m", 5.0)
        # B4: default'lar autonomy.yaml ile sync — YAML yüklenemezse güvenli/tuned
        # değerler devreye girer (eski kod 0.8 ile araç %33 hızlı gidebiliyordu).
        self.declare_parameter("max_speed_mps", 0.6)
        self.declare_parameter("max_yaw_rate_deg_s", 50.0)
        self.declare_parameter("yaw_kp_deg_s_per_deg", 0.55)
        self.declare_parameter("telemetry_timeout_s", 2.0)
        self.declare_parameter("perception_timeout_s", 2.0)
        self.declare_parameter("obstacle_avoid_distance_m", 8.0)
        self.declare_parameter("target_min_confidence", 0.20)
        self.declare_parameter("p3_search_yaw_rate_deg_s", 30.0)
        self.declare_parameter("p3_search_scan_degrees", 360.0)
        self.declare_parameter("p3_search_initial_hold_s", 1.0)
        self.declare_parameter("p3_search_reposition_m", 2.0)
        self.declare_parameter("p3_search_transit_speed_mps", 0.30)
        self.declare_parameter("p3_search_reposition_arrival_m", 0.50)
        self.declare_parameter("p3_search_anchor_acceptance_m", 1.50)
        # P2 bitişinden sonra hakem görev listesine waypoint eklemeden, son
        # rota doğrultusunda kontrollü P3 yaklaşma başlangıcı.
        self.declare_parameter("p2_exit_advance_m", 5.0)
        self.declare_parameter("p2_exit_advance_speed_mps", 0.70)
        self.declare_parameter("p2_exit_align_latch_release_deg", 150.0)
        self.declare_parameter("p3_lock_confirm_s", 0.0)
        self.declare_parameter("p3_lock_confirm_frames", 1)
        self.declare_parameter("p3_target_loss_grace_s", 0.80)
        # Son lidar-fused hedef koordinatını bu süre boyunca sanal hedef olarak
        # koru. Kısa kamera kaybı P3 arama/anchor otoritesini devralamaz.
        self.declare_parameter("p3_target_memory_timeout_s", 20.0)
        self.declare_parameter("p3_lock_yaw_gain_deg_s", 35.0)
        self.declare_parameter("p3_engage_center_max", 0.18)
        self.declare_parameter("p3_engage_distance_m", 1.2)
        self.declare_parameter("p3_lock_min_speed_mps", 0.18)
        self.declare_parameter("p3_engage_speed_mps", 0.18)
        self.declare_parameter("p3_align_distance_m", 5.0)
        self.declare_parameter("p3_approach_speed_mps", 0.40)
        self.declare_parameter("p3_approach_yaw_max_deg_s", 12.0)
        # Ayrıcalıklı gate truth yalnız sim launch override'ıyla açılır;
        # gerçek araç/canonical kamera hattında varsayılan kapalıdır.
        self.declare_parameter("sim_gate_truth_enabled", False)

        # BENCH/TEST-ONLY P3 direct-start mechanism. Default fail-closed:
        # bench_p3_only_enabled=False ile davranış production ile birebir
        # aynıdır ve P1->P2->P3 hakem sırası korunur. Yalnız açıkça
        # etkinleştirilmiş bench launch'ı (bench_p3_decision.launch.py) bu
        # bayrağı açar; motor yolu o launch'ta kapalıdır (dry_run, guided=off,
        # motor_command_enabled=false). Bu bir DEPLOYMENT parametresidir —
        # field_profile.yaml tuning'i değildir.
        self.declare_parameter("bench_p3_only_enabled", False)
        self.declare_parameter("bench_p3_start_parkur", 0)

        # Parkur geçiş eşikleri (İş 2 tasarımı).
        self.declare_parameter("p2_min_pair_crossings", 2)
        self.declare_parameter("p1_max_duration_s", 360.0)
        self.declare_parameter("p2_max_duration_s", 420.0)
        self.declare_parameter("p3_hold_timeout_s", 60.0)

        # Sarı kapı geçiş sayacı parametreleri.
        self.declare_parameter("yellow_sighting_threshold", 4)
        self.declare_parameter("yellow_sighting_decay", 2)
        self.declare_parameter("yellow_sighting_range_m", 15.0)
        self.declare_parameter("yellow_sighting_forward_m", 1.0)
        self.declare_parameter("yellow_sighting_lateral_m", 6.0)
        self.declare_parameter("yellow_sighting_min_confidence", 0.35)

        # Duba ikilisi geçiş detektörü parametreleri.
        self.declare_parameter("pair_max_lateral_m", 15.0)
        self.declare_parameter("pair_min_lateral_gap_m", 1.0)
        self.declare_parameter("pair_cross_trigger_forward_m", 3.0)
        self.declare_parameter("pair_cross_complete_forward_m", -0.5)
        self.declare_parameter("pair_min_confidence", 0.35)
        self.declare_parameter("pair_max_age_s", 20.0)
        self.declare_parameter("pair_max_forward_gap_m", 3.0)
        self.declare_parameter("pair_max_crossing_gap_s", 2.0)

        # Parkur 3 (hedef angajmanı) parametreleri.
        self.declare_parameter("wrong_avoid_lateral_m", 1.5)
        self.declare_parameter("wrong_avoid_speed_mps", 0.35)
        self.declare_parameter("engage_window_s", 2.0)

        # Failsafe.
        self.declare_parameter("failsafe_recovery_enabled", True)
        self.declare_parameter("failsafe_recovery_s", 5.0)

        # Puan ceza modülü (çarpma + parkur dışı + puan hesabı).
        self.declare_parameter("score_kd1", 5.0)
        self.declare_parameter("score_kd2", 5.0)
        self.declare_parameter("score_ed2", 5.0)
        self.declare_parameter("score_ooc_mode", "waypoint_bbox")
        self.declare_parameter("score_half_width_m", 8.0)
        self.declare_parameter("score_sustained_contact_s", 30.0)
        self.declare_parameter("score_contact_radius_m", 1.6)
        self.declare_parameter("score_drop_after_s", 5.0)
        self.declare_parameter("score_include_uav_bonus", True)

        # Gerçek rota geometrisi yalnız durum/puan gözlemi içindir. Turuncu
        # dubaların sert costmap maliyeti ayrı, mission boyunca latch'li kalır.
        self.declare_parameter("course_geometry_enter_m", 4.5)
        self.declare_parameter("course_geometry_exit_m", 5.5)

        # Costmap + DWA (çakışma #15 çözümü: P1/P2 planlaması costmap maliyetinde).
        self.declare_parameter("dwa_enabled", True)
        self.declare_parameter("nn_sort_enabled", False)
        self.declare_parameter("costmap_size_m", 30.0)
        self.declare_parameter("costmap_cell_m", 0.25)
        self.declare_parameter("costmap_bot_radius_m", 0.6)
        self.declare_parameter("costmap_safety_m", 1.0)   # B4: YAML ile sync (güvenlik marjı)
        self.declare_parameter("dwa_recovery_vx", 0.70)   # Saha itki eşiği; YAML ile sync
        self.declare_parameter("dwa_recovery_yaw", 25.0)  # B4: YAML ile sync
        self.declare_parameter("dwa_w_obstacle", DEFAULT_W["obstacle"])
        self.declare_parameter("dwa_w_heading", DEFAULT_W["heading"])
        self.declare_parameter("dwa_w_progress", DEFAULT_W["progress"])
        self.declare_parameter("dwa_w_speed", DEFAULT_W["speed"])
        self.declare_parameter("dwa_w_smooth", DEFAULT_W["smooth"])
        self.declare_parameter("dwa_w_unknown", DEFAULT_W["unknown"])
        self.declare_parameter("dwa_w_corridor", DEFAULT_W["corridor"])
        self.declare_parameter("dwa_w_avoid", DEFAULT_W["avoid"])
        self.declare_parameter("dwa_vx_steps", 7)
        self.declare_parameter("dwa_yaw_steps", 17)   # B4: YAML ile sync (119 aday)
        self.declare_parameter("dwa_sim_time", DEFAULT_SIM["sim_time_s"])
        self.declare_parameter("dwa_sim_step", DEFAULT_SIM["sim_step_s"])
        self.declare_parameter("dwa_accel_max", DEFAULT_ACCEL["max_accel_mps2"])
        self.declare_parameter("dwa_slow_vx", 0.70)   # Saha itki eşiği; YAML ile sync
        self.declare_parameter("dwa_min_drive_vx", 0.70)
        self.declare_parameter("dwa_min_turn_rate_deg_s", 20.0)
        self.declare_parameter("dwa_recovery_latch_ticks", 12)
        self.declare_parameter("dwa_align_before_drive_deg", 42.0)
        self.declare_parameter("dwa_align_yaw_rate_deg_s", 30.0)
        self.declare_parameter("dwa_align_parkurs", [1])
        # DWA kaçınma yön histerezisi: sabit engelde pivot/ilerlemeli geçişleri
        # bastırır; yön değişimi kararlı kanıt (N tick) ister.
        self.declare_parameter("dwa_avoid_direction_latch_ticks", 8)
        self.declare_parameter("p2_dwa_max_yaw_rate_deg_s", 22.0)
        # P2 DWA her adayı zaman ufkunda gövde iziyle zaten tarar. Global
        # costmap şişirmesine tekrar tam hız durma mesafesi eklemek dar koridoru
        # iki kez küçültür. Bu tavan yalnız o ikinci (global) katmanı sınırlar.
        self.declare_parameter("p2_costmap_inflation_speed_cap_mps", 0.0)

        # Yakın-alan güvenliği (saha bulgusu 2026-08-17): hard obstacle 0.93-1.56 m
        # mesafedeyken ileri komut sıfırlanmalı. Gerçek gövde geometrisi
        # (0.76 m genişlik / 1.18 m uzunluk -> yarım uzunluk 0.59 m) ve
        # lidar offseti (0.57 m önde, sllidar_bridge'de gövde frame'e taşınır)
        # dikkate alınır. DWA skorlamasından bağımsız kesin kuraldır (P1/P2).
        self.declare_parameter("near_field_stop_m", 1.5)
        self.declare_parameter("near_field_slow_m", 3.0)
        self.declare_parameter("near_field_stop_lateral_m", 1.0)
        self.declare_parameter("near_field_slow_lateral_m", 1.5)
        self.declare_parameter("near_field_pivot_yaw_deg_s", 30.0)
        self.declare_parameter("near_field_slow_speed_mps", 0.30)
        self.declare_parameter("near_field_stop_release_m", 2.5)
        # P3 hedef dışı engelleri için ayrı son yaklaşma zarfı. Doğrulanmış
        # angajman hedefi zaten bu near-field girdisinden çıkarılır.
        self.declare_parameter("p3_near_field_stop_m", 1.0)
        self.declare_parameter("p3_near_field_stop_release_m", 1.8)
        self.declare_parameter("near_field_escape_trigger_s", 1.5)
        self.declare_parameter("near_field_escape_reverse_s", 1.5)
        self.declare_parameter("near_field_escape_cooldown_s", 1.0)
        self.declare_parameter("near_field_escape_reverse_mps", 0.20)
        self.declare_parameter("near_field_escape_yaw_deg_s", 30.0)
        self.declare_parameter("near_field_escape_rear_stop_m", 1.5)
        # Varsayılan 0: bilinen-iyi sim davranışı değişmez. Saha profili,
        # reverse sonrası açık koridora kontrollü ters-yaw çıkış yayı ekler.
        self.declare_parameter("near_field_escape_forward_commit_s", 0.0)
        self.declare_parameter("near_field_escape_forward_commit_mps", 0.0)
        self.declare_parameter("near_field_escape_forward_commit_yaw_deg_s", 0.0)
        self.declare_parameter("near_field_heading_align_speed_mps", 0.20)
        self.declare_parameter("near_field_heading_align_max_yaw_deg_s", 12.0)

        # P1/P2 tek davranış hakemi. DWA/align/recovery/near-field yalnız aday
        # üretir; bu katman tek tick'te tek davranış seçer ve son güvenlik
        # zarfını uygular. Tüm saha ayarları autonomy.yaml'dan gelir.
        self.declare_parameter("nav_align_min_dwell_s", 0.6)
        self.declare_parameter("nav_recovery_min_dwell_s", 1.0)
        self.declare_parameter("nav_slow_min_dwell_s", 0.4)
        self.declare_parameter("nav_final_high_yaw_deg_s", 22.5)
        self.declare_parameter("nav_final_obstacle_stop_m", 2.0)
        self.declare_parameter("nav_rotational_sweep_radius_m", 1.0)
        self.declare_parameter("nav_rear_sweep_escape_speed_mps", 0.20)

        # Tekil hedef kaynağı (T1): koridor-hedef override'ı ve koridor bias'ı
        # parkur bazlı kontrolü. Varsayılanlar P1'i kapsar ([1]); P2'de hedef
        # her zaman waypoint olur ve DWA skorlama bias'ı kapanır (çifte uygulama
        # yok — sarı engel üstüne hedef düşmez). Yetki yalnız merkezi
        # profil listesinden verilir; saha profilinde P2 bilerek kapalıdır.
        # WAYPOINT BİRİNCİL HEDEF: corridor_goal_parkurs default [0] -> koridor
        # hedef override'ı KAPALI (hedef her zaman waypoint). Koridor bilgisi
        # yalnız DWA skor bias'ı (corridor_bias_parkurs) olarak kullanılır.
        # ROS2 Humble boş listeyi BYTE_ARRAY olarak yorumlar. Saha profilindeki
        # INTEGER_ARRAY [0] override'ı bu nedenle node'u açılışta düşürüyordu.
        # [0] hiçbir gerçek parkurla eşleşmez ve aynı "kapalı" semantiğini korur.
        self.declare_parameter("corridor_goal_parkurs", [0])
        self.declare_parameter("corridor_bias_parkurs", [1])
        self.declare_parameter("corridor_goal_lookahead_min_m", 8.0)
        self.declare_parameter("corridor_goal_lateral_gain", 0.3)
        # P2'nin GN4->GN5 mission doğrusu kamera modelinden bağımsız yumuşak
        # merkezleme + dışarı doğru aday veto zarfıdır. Hedef yine waypoint'tir.
        self.declare_parameter("p2_route_corridor_enabled", True)
        self.declare_parameter("p2_route_corridor_lookahead_m", 8.0)
        self.declare_parameter("p2_route_corridor_half_width_m", 4.0)
        self.declare_parameter("p2_route_corridor_confidence", 1.0)

        # Hıza bağlı güvenlik (idaws'tan): durma mesafesi = v*t + v^2/2a.
        # costmap_safety_m taban güvenlik mesafesidir; bu parametreler hız
        # arttıkça şişirme yarıçapını büyütür (costmap.set_speed) ve DWA hız
        # ölçeğini sensör menziline göre sınırlar (governor).
        self.declare_parameter("safety_base_m", 0.5)
        self.declare_parameter("safety_max_m", 10.0)
        self.declare_parameter("reaction_time_s", 1.0)
        self.declare_parameter("max_decel_mps2", 1.5)
        self.declare_parameter("max_speed_scale", 1.0)

        # T3: Zaman tabanlı stuck dedektörü (dwa.py anlık vx koşulu kaldırıldı).
        # Araç bu süre boyunca < stuck_min_dist_m yol almadıysa recovery tetiklenir.
        # Anlık hız düşüşü (approach/yellow brake) takılma SAYILMAZ — sürekli
        # hareketsizlik gerekir (yanlış recovery spin'i önler, dur-devam kırılır).
        self.declare_parameter("stuck_timeout_s", 3.0)
        self.declare_parameter("stuck_min_dist_m", 0.2)
        self.declare_parameter("stuck_vx_threshold", 0.1)

        self.loop_hz = float(self.get_parameter("loop_hz").value)
        self.waypoint_threshold_m = float(self.get_parameter("waypoint_threshold_m").value)
        self.waypoint_overshoot_guard_m = float(
            self.get_parameter("waypoint_overshoot_guard_m").value
        )
        if not math.isfinite(self.waypoint_threshold_m) or self.waypoint_threshold_m <= 0.0:
            raise ValueError("waypoint_threshold_m sonlu ve > 0 olmalı")
        if (
            not math.isfinite(self.waypoint_overshoot_guard_m)
            or self.waypoint_overshoot_guard_m < self.waypoint_threshold_m
        ):
            raise ValueError(
                "waypoint_overshoot_guard_m sonlu ve waypoint_threshold_m'den "
                "küçük olmamalı"
            )
        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_deg_s").value)
        self.yaw_kp = float(self.get_parameter("yaw_kp_deg_s_per_deg").value)
        self.telemetry_timeout = float(self.get_parameter("telemetry_timeout_s").value)
        self.perception_timeout = float(self.get_parameter("perception_timeout_s").value)
        self.obstacle_avoid_distance = float(self.get_parameter("obstacle_avoid_distance_m").value)
        self.target_min_confidence = float(self.get_parameter("target_min_confidence").value)
        self.p3_search_yaw_rate_deg_s = float(
            self.get_parameter("p3_search_yaw_rate_deg_s").value
        )
        self.p3_search_scan_degrees = float(
            self.get_parameter("p3_search_scan_degrees").value
        )
        self.p3_search_initial_hold_s = float(
            self.get_parameter("p3_search_initial_hold_s").value
        )
        self.p3_search_reposition_m = float(
            self.get_parameter("p3_search_reposition_m").value
        )
        self.p3_search_transit_speed_mps = float(
            self.get_parameter("p3_search_transit_speed_mps").value
        )
        self.p3_search_reposition_arrival_m = float(
            self.get_parameter("p3_search_reposition_arrival_m").value
        )
        self.p3_search_anchor_acceptance_m = float(
            self.get_parameter("p3_search_anchor_acceptance_m").value
        )
        self.p2_exit_advance_m = float(
            self.get_parameter("p2_exit_advance_m").value
        )
        self.p2_exit_advance_speed_mps = float(
            self.get_parameter("p2_exit_advance_speed_mps").value
        )
        self.p2_exit_align_latch_release_deg = float(
            self.get_parameter("p2_exit_align_latch_release_deg").value
        )
        self.p3_lock_confirm_s = float(self.get_parameter("p3_lock_confirm_s").value)
        self.p3_lock_confirm_frames = int(
            self.get_parameter("p3_lock_confirm_frames").value
        )
        self.p3_target_loss_grace_s = float(
            self.get_parameter("p3_target_loss_grace_s").value
        )
        self.p3_target_memory_timeout_s = float(
            self.get_parameter("p3_target_memory_timeout_s").value
        )
        self.p3_lock_yaw_gain_deg_s = float(
            self.get_parameter("p3_lock_yaw_gain_deg_s").value
        )
        self.p3_engage_center_max = float(
            self.get_parameter("p3_engage_center_max").value
        )
        self.p3_engage_distance_m = float(
            self.get_parameter("p3_engage_distance_m").value
        )
        self.p3_lock_min_speed_mps = float(
            self.get_parameter("p3_lock_min_speed_mps").value
        )
        self.p3_engage_speed_mps = float(
            self.get_parameter("p3_engage_speed_mps").value
        )
        self.p3_align_distance_m = float(
            self.get_parameter("p3_align_distance_m").value
        )
        self.p3_approach_speed_mps = float(
            self.get_parameter("p3_approach_speed_mps").value
        )
        self.p3_approach_yaw_max_deg_s = float(
            self.get_parameter("p3_approach_yaw_max_deg_s").value
        )
        p3_finite = (
            self.target_min_confidence,
            self.p3_search_yaw_rate_deg_s,
            self.p3_search_scan_degrees,
            self.p3_search_reposition_m,
            self.p3_search_transit_speed_mps,
            self.p3_search_reposition_arrival_m,
            self.p3_search_anchor_acceptance_m,
            self.p3_lock_confirm_s,
            self.p3_target_loss_grace_s,
            self.p3_target_memory_timeout_s,
            self.p3_lock_yaw_gain_deg_s,
            self.p3_engage_center_max,
            self.p3_engage_distance_m,
            self.p3_lock_min_speed_mps,
            self.p3_engage_speed_mps,
            self.p3_align_distance_m,
            self.p3_approach_speed_mps,
            self.p3_approach_yaw_max_deg_s,
            self.p3_search_initial_hold_s,
            self.p2_exit_advance_m,
            self.p2_exit_advance_speed_mps,
            self.p2_exit_align_latch_release_deg,
        )
        if not all(math.isfinite(value) for value in p3_finite):
            raise ValueError("P3 target parameters must all be finite")
        if not 0.0 <= self.target_min_confidence <= 1.0:
            raise ValueError("target_min_confidence must be within [0, 1]")
        if not 0.0 < self.p3_search_yaw_rate_deg_s <= self.max_yaw_rate:
            raise ValueError("p3_search_yaw_rate_deg_s must be within (0, max_yaw_rate]")
        if not 300.0 <= self.p3_search_scan_degrees <= 400.0:
            raise ValueError("p3_search_scan_degrees must be within [300, 400]")
        if not 0.0 <= self.p3_search_initial_hold_s <= 10.0:
            raise ValueError("p3_search_initial_hold_s must be within [0, 10]")
        if not 0.5 <= self.p3_search_reposition_m <= 5.0:
            raise ValueError("p3_search_reposition_m must be within [0.5, 5]")
        if not 0.0 < self.p3_search_transit_speed_mps <= self.max_speed:
            raise ValueError("p3_search_transit_speed_mps must be within (0, max_speed]")
        if not 0.1 <= self.p3_search_reposition_arrival_m <= self.p3_search_reposition_m:
            raise ValueError("p3_search_reposition_arrival_m is invalid")
        if not 0.5 <= self.p3_search_anchor_acceptance_m <= 3.0:
            raise ValueError("p3_search_anchor_acceptance_m must be within [0.5, 3]")
        if not 0.0 <= self.p2_exit_advance_m <= 20.0:
            raise ValueError("p2_exit_advance_m must be within [0, 20]")
        if not 0.0 < self.p2_exit_advance_speed_mps <= self.max_speed:
            raise ValueError(
                "p2_exit_advance_speed_mps must be within (0, max_speed]"
            )
        if not 90.0 <= self.p2_exit_align_latch_release_deg <= 180.0:
            raise ValueError(
                "p2_exit_align_latch_release_deg must be within [90, 180]"
            )
        if not 0.0 <= self.p3_lock_confirm_s <= 10.0:
            raise ValueError("p3_lock_confirm_s must be within [0, 10]")
        if not 1 <= self.p3_lock_confirm_frames <= 100:
            raise ValueError("p3_lock_confirm_frames must be within [1, 100]")
        if not 0.0 <= self.p3_target_loss_grace_s <= 4.0:
            raise ValueError("p3_target_loss_grace_s must be within [0, 4]")
        if not self.p3_target_loss_grace_s <= self.p3_target_memory_timeout_s <= 60.0:
            raise ValueError(
                "p3_target_memory_timeout_s must be within "
                "[p3_target_loss_grace_s, 60]"
            )
        if not 0.0 < self.p3_lock_yaw_gain_deg_s <= 360.0:
            raise ValueError("p3_lock_yaw_gain_deg_s must be within (0, 360]")
        if not 0.0 < self.p3_engage_center_max <= 1.0:
            raise ValueError("p3_engage_center_max must be within (0, 1]")
        if not 0.0 < self.p3_engage_distance_m <= 50.0:
            raise ValueError("p3_engage_distance_m must be within (0, 50]")
        if not 0.0 < self.p3_lock_min_speed_mps <= self.max_speed:
            raise ValueError("p3_lock_min_speed_mps must be within (0, max_speed]")
        if not 0.0 < self.p3_engage_speed_mps <= self.max_speed:
            raise ValueError("p3_engage_speed_mps must be within (0, max_speed]")
        if not self.p3_engage_distance_m < self.p3_align_distance_m <= 50.0:
            raise ValueError(
                "p3_align_distance_m must exceed engage distance and be <= 50"
            )
        if not 0.0 < self.p3_approach_speed_mps <= self.max_speed:
            raise ValueError("p3_approach_speed_mps must be within (0, max_speed]")
        if not 0.0 < self.p3_approach_yaw_max_deg_s <= self.max_yaw_rate:
            raise ValueError(
                "p3_approach_yaw_max_deg_s must be within (0, max_yaw_rate]"
            )
        self.sim_gate_truth_enabled = bool(
            self.get_parameter("sim_gate_truth_enabled").value
        )
        self.bench_p3_only_enabled = bool(
            self.get_parameter("bench_p3_only_enabled").value
        )
        self.bench_p3_start_parkur = int(
            self.get_parameter("bench_p3_start_parkur").value
        )
        if self.bench_p3_only_enabled and not 1 <= self.bench_p3_start_parkur <= 3:
            raise ValueError(
                "bench_p3_start_parkur must be in [1, 3] when bench_p3_only_enabled=true"
            )

        self.p2_min_pair_crossings = int(self.get_parameter("p2_min_pair_crossings").value)
        self.p1_max_duration_s = float(self.get_parameter("p1_max_duration_s").value)
        self.p2_max_duration_s = float(self.get_parameter("p2_max_duration_s").value)
        self.p3_hold_timeout_s = float(self.get_parameter("p3_hold_timeout_s").value)
        for name, value in (
            ("p1_max_duration_s", self.p1_max_duration_s),
            ("p2_max_duration_s", self.p2_max_duration_s),
            ("p3_hold_timeout_s", self.p3_hold_timeout_s),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and >= 0 (0 disables it)")

        self.yellow_sighting_threshold = int(self.get_parameter("yellow_sighting_threshold").value)
        self.yellow_sighting_decay = float(self.get_parameter("yellow_sighting_decay").value)
        self.yellow_sighting_range_m = float(self.get_parameter("yellow_sighting_range_m").value)
        self.yellow_sighting_forward_m = float(self.get_parameter("yellow_sighting_forward_m").value)
        self.yellow_sighting_lateral_m = float(self.get_parameter("yellow_sighting_lateral_m").value)
        self.yellow_sighting_min_confidence = float(
            self.get_parameter("yellow_sighting_min_confidence").value
        )

        self.pair_max_lateral_m = float(self.get_parameter("pair_max_lateral_m").value)
        self.pair_min_lateral_gap_m = float(self.get_parameter("pair_min_lateral_gap_m").value)
        self.pair_cross_trigger_forward_m = float(self.get_parameter("pair_cross_trigger_forward_m").value)
        self.pair_cross_complete_forward_m = float(self.get_parameter("pair_cross_complete_forward_m").value)
        self.pair_min_confidence = float(self.get_parameter("pair_min_confidence").value)
        self.pair_max_age_s = float(self.get_parameter("pair_max_age_s").value)
        self.pair_max_forward_gap_m = float(
            self.get_parameter("pair_max_forward_gap_m").value
        )
        self.pair_max_crossing_gap_s = float(
            self.get_parameter("pair_max_crossing_gap_s").value
        )

        self.wrong_avoid_lateral_m = float(self.get_parameter("wrong_avoid_lateral_m").value)
        self.wrong_avoid_speed_mps = float(self.get_parameter("wrong_avoid_speed_mps").value)
        self.engage_window_s = float(self.get_parameter("engage_window_s").value)

        self.failsafe_recovery_enabled = bool(self.get_parameter("failsafe_recovery_enabled").value)
        self.failsafe_recovery_s = float(self.get_parameter("failsafe_recovery_s").value)

        self.course_geometry_enter_m = float(
            self.get_parameter("course_geometry_enter_m").value
        )
        self.course_geometry_exit_m = float(
            self.get_parameter("course_geometry_exit_m").value
        )

        # Costmap + DWA (P1/P2 tick planlaması; P3'te devre dışı).
        self.dwa_enabled = bool(self.get_parameter("dwa_enabled").value)
        nn_sort_requested = bool(self.get_parameter("nn_sort_enabled").value)
        self.nn_sort_enabled = False
        if nn_sort_requested:
            self.get_logger().warn(
                "nn_sort_enabled ignored: authoritative mission waypoint order is required"
            )
        # Hıza bağlı güvenlik parametreleri (costmap şişirme + DWA governor).
        self.safety_base_m = float(self.get_parameter("safety_base_m").value)
        self.safety_max_m = float(self.get_parameter("safety_max_m").value)
        self.reaction_time_s = float(self.get_parameter("reaction_time_s").value)
        self.max_decel_mps2 = float(self.get_parameter("max_decel_mps2").value)
        self.max_speed_scale = float(self.get_parameter("max_speed_scale").value)
        self.p2_costmap_inflation_speed_cap_mps = float(
            self.get_parameter("p2_costmap_inflation_speed_cap_mps").value
        )
        if (
            not math.isfinite(self.p2_costmap_inflation_speed_cap_mps)
            or not 0.0 <= self.p2_costmap_inflation_speed_cap_mps <= self.max_speed
        ):
            raise ValueError(
                "p2_costmap_inflation_speed_cap_mps must be within "
                "[0, max_speed_mps]"
            )
        self.costmap = CostMap(
            size_m=float(self.get_parameter("costmap_size_m").value),
            cell_m=float(self.get_parameter("costmap_cell_m").value),
            bot_radius_m=float(self.get_parameter("costmap_bot_radius_m").value),
            safety_m=float(self.get_parameter("costmap_safety_m").value),
            safety_base_m=self.safety_base_m,
            safety_max_m=self.safety_max_m,
            reaction_time_s=self.reaction_time_s,
            max_decel_mps2=self.max_decel_mps2,
        )
        self.dwa = DwaPlanner(
            max_speed_mps=self.max_speed,
            max_yaw_rate_deg_s=self.max_yaw_rate,
            vx_steps=int(self.get_parameter("dwa_vx_steps").value),
            yaw_steps=int(self.get_parameter("dwa_yaw_steps").value),
            sim_time_s=float(self.get_parameter("dwa_sim_time").value),
            sim_step_s=float(self.get_parameter("dwa_sim_step").value),
            accel_limits={
                "max_accel_mps2": float(self.get_parameter("dwa_accel_max").value),
                "max_decel_mps2": 1.2,
                "max_yaw_accel_deg_s2": 180.0,  # 90 -> 180: dönüş ivmesi açık (keskin dönüş)
            },
            w={
                "obstacle": float(self.get_parameter("dwa_w_obstacle").value),
                "heading": float(self.get_parameter("dwa_w_heading").value),
                "progress": float(self.get_parameter("dwa_w_progress").value),
                "speed": float(self.get_parameter("dwa_w_speed").value),
                "smooth": float(self.get_parameter("dwa_w_smooth").value),
                "unknown": float(self.get_parameter("dwa_w_unknown").value),
                "corridor": float(self.get_parameter("dwa_w_corridor").value),
                "avoid": float(self.get_parameter("dwa_w_avoid").value),
            },
            recovery_vx=float(self.get_parameter("dwa_recovery_vx").value),
            recovery_yaw_deg_s=float(self.get_parameter("dwa_recovery_yaw").value),
            unknown_slow_vx=float(self.get_parameter("dwa_slow_vx").value),
            min_drive_speed_mps=float(self.get_parameter("dwa_min_drive_vx").value),
            min_turn_rate_deg_s=float(self.get_parameter("dwa_min_turn_rate_deg_s").value),
            recovery_latch_ticks=int(self.get_parameter("dwa_recovery_latch_ticks").value),
            align_before_drive_deg=float(self.get_parameter("dwa_align_before_drive_deg").value),
            align_yaw_rate_deg_s=float(self.get_parameter("dwa_align_yaw_rate_deg_s").value),
            avoid_direction_latch_ticks=int(
                self.get_parameter("dwa_avoid_direction_latch_ticks").value
            ),
            # CostMap engeli zaten bot_radius + dinamik safety kadar şişirir.
            # Burada aynı yarıçapı tekrar süpürmek güvenliği iki kez sayardı.
            bot_radius_m=0.0,
            cell_m=float(self.get_parameter("costmap_cell_m").value),
            n_cells=int(round(float(self.get_parameter("costmap_size_m").value)
                              / float(self.get_parameter("costmap_cell_m").value))),
            max_speed_scale=self.max_speed_scale,
        )
        self.dwa_slow_vx = float(self.get_parameter("dwa_slow_vx").value)
        self.p2_dwa_max_yaw_rate_deg_s = float(
            self.get_parameter("p2_dwa_max_yaw_rate_deg_s").value
        )
        if not 0.0 < self.p2_dwa_max_yaw_rate_deg_s < float(
            self.get_parameter("nav_final_high_yaw_deg_s").value
        ):
            raise ValueError(
                "p2_dwa_max_yaw_rate_deg_s must be positive and below "
                "nav_final_high_yaw_deg_s"
            )
        # Yakın-alan güvenlik eşikleri (P1/P2 DWA navigasyonu; P3 hedef angajmanı
        # kendi mesafe/merkezleme kurallarına tabidir ve bu katmandan etkilenmez).
        self.near_field_stop_m = float(self.get_parameter("near_field_stop_m").value)
        self.near_field_slow_m = float(self.get_parameter("near_field_slow_m").value)
        self.near_field_stop_lateral_m = float(
            self.get_parameter("near_field_stop_lateral_m").value
        )
        self.near_field_slow_lateral_m = float(
            self.get_parameter("near_field_slow_lateral_m").value
        )
        self.near_field_pivot_yaw_deg_s = float(
            self.get_parameter("near_field_pivot_yaw_deg_s").value
        )
        self.near_field_slow_speed_mps = float(
            self.get_parameter("near_field_slow_speed_mps").value
        )
        self.near_field_stop_release_m = float(
            self.get_parameter("near_field_stop_release_m").value
        )
        self.p3_near_field_stop_m = float(
            self.get_parameter("p3_near_field_stop_m").value
        )
        self.p3_near_field_stop_release_m = float(
            self.get_parameter("p3_near_field_stop_release_m").value
        )
        self.near_field_heading_align_speed_mps = float(
            self.get_parameter("near_field_heading_align_speed_mps").value
        )
        self.near_field_heading_align_max_yaw_deg_s = float(
            self.get_parameter("near_field_heading_align_max_yaw_deg_s").value
        )
        near_field_values = (
            self.near_field_stop_m,
            self.near_field_slow_m,
            self.near_field_stop_lateral_m,
            self.near_field_slow_lateral_m,
            self.near_field_pivot_yaw_deg_s,
            self.near_field_slow_speed_mps,
            self.near_field_stop_release_m,
            self.p3_near_field_stop_m,
            self.p3_near_field_stop_release_m,
            self.near_field_heading_align_speed_mps,
            self.near_field_heading_align_max_yaw_deg_s,
        )
        if not all(math.isfinite(value) for value in near_field_values):
            raise ValueError("near-field safety parameters must all be finite")
        if not 0.0 < self.near_field_stop_m <= self.near_field_stop_release_m:
            raise ValueError(
                "near_field_stop_m must be in (0, near_field_stop_release_m]"
            )
        if not 0.0 < self.p3_near_field_stop_m <= self.p3_near_field_stop_release_m:
            raise ValueError(
                "p3_near_field_stop_m must be in "
                "(0, p3_near_field_stop_release_m]"
            )
        if self.p3_near_field_stop_release_m > self.near_field_slow_m:
            raise ValueError(
                "p3_near_field_stop_release_m cannot exceed near_field_slow_m"
            )
        if self.near_field_slow_lateral_m < self.near_field_stop_lateral_m:
            raise ValueError(
                "near_field_slow_lateral_m cannot be below "
                "near_field_stop_lateral_m"
            )
        self.nav_final_obstacle_stop_m = float(
            self.get_parameter("nav_final_obstacle_stop_m").value
        )
        if self.nav_final_obstacle_stop_m > self.near_field_stop_m:
            raise ValueError(
                "nav_final_obstacle_stop_m cannot exceed near_field_stop_m; "
                "otherwise a zero-vx/zero-yaw dead zone is created"
            )
        if self.near_field_heading_align_speed_mps < 0.0:
            raise ValueError("near_field_heading_align_speed_mps must be non-negative")
        if not 0.0 < self.near_field_heading_align_max_yaw_deg_s <= self.max_yaw_rate:
            raise ValueError(
                "near_field_heading_align_max_yaw_deg_s must be in (0, max_yaw_rate]"
            )
        self._near_field_latch = False
        self.center_obstacle_escape = CenterObstacleEscapeController(
            trigger_s=float(self.get_parameter("near_field_escape_trigger_s").value),
            reverse_s=float(self.get_parameter("near_field_escape_reverse_s").value),
            cooldown_s=float(self.get_parameter("near_field_escape_cooldown_s").value),
            reverse_speed_mps=float(
                self.get_parameter("near_field_escape_reverse_mps").value
            ),
            yaw_deg_s=float(self.get_parameter("near_field_escape_yaw_deg_s").value),
            rear_stop_m=float(
                self.get_parameter("near_field_escape_rear_stop_m").value
            ),
            forward_commit_s=float(
                self.get_parameter("near_field_escape_forward_commit_s").value
            ),
            forward_commit_speed_mps=float(
                self.get_parameter("near_field_escape_forward_commit_mps").value
            ),
            forward_commit_yaw_deg_s=float(
                self.get_parameter(
                    "near_field_escape_forward_commit_yaw_deg_s"
                ).value
            ),
        )
        self.nav_arbiter = NavigationBehaviorArbiter(
            min_dwell_s={
                MODE_ALIGN: float(self.get_parameter("nav_align_min_dwell_s").value),
                MODE_RECOVERY: float(
                    self.get_parameter("nav_recovery_min_dwell_s").value
                ),
                MODE_SLOW: float(self.get_parameter("nav_slow_min_dwell_s").value),
                # Near-field stop release is governed by distance hysteresis;
                # adding a time dwell would delay a newly clear path.
                MODE_NEAR_STOP: 0.0,
            },
            high_yaw_deg_s=float(
                self.get_parameter("nav_final_high_yaw_deg_s").value
            ),
            obstacle_stop_m=float(
                self.nav_final_obstacle_stop_m
            ),
            forward_accel_mps2=float(
                self.get_parameter("dwa_accel_max").value
            ),
            lateral_clearance_m=self.near_field_stop_lateral_m,
            rotational_sweep_radius_m=float(
                self.get_parameter("nav_rotational_sweep_radius_m").value
            ),
            rear_sweep_escape_speed_mps=float(
                self.get_parameter("nav_rear_sweep_escape_speed_mps").value
            ),
        )
        # Tekil hedef kaynağı parametre değerleri (YAML override; tip list-of-int).
        self.corridor_goal_parkurs: List[int] = list(self.get_parameter("corridor_goal_parkurs").value)
        self.corridor_bias_parkurs: List[int] = list(self.get_parameter("corridor_bias_parkurs").value)
        self.dwa_align_parkurs: List[int] = list(
            self.get_parameter("dwa_align_parkurs").value
        )
        self.corridor_goal_lookahead_min_m = float(self.get_parameter("corridor_goal_lookahead_min_m").value)
        self.corridor_goal_lateral_gain = float(self.get_parameter("corridor_goal_lateral_gain").value)
        self.p2_route_corridor_enabled = bool(
            self.get_parameter("p2_route_corridor_enabled").value
        )
        self.p2_route_corridor_lookahead_m = float(
            self.get_parameter("p2_route_corridor_lookahead_m").value
        )
        self.p2_route_corridor_half_width_m = float(
            self.get_parameter("p2_route_corridor_half_width_m").value
        )
        self.p2_route_corridor_confidence = float(
            self.get_parameter("p2_route_corridor_confidence").value
        )
        if (
            not math.isfinite(self.p2_route_corridor_lookahead_m)
            or self.p2_route_corridor_lookahead_m <= 0.0
            or not math.isfinite(self.p2_route_corridor_half_width_m)
            or self.p2_route_corridor_half_width_m <= 0.0
            or not math.isfinite(self.p2_route_corridor_confidence)
            or not 0.0 <= self.p2_route_corridor_confidence <= 1.0
        ):
            raise ValueError("P2 route corridor parameters are invalid")
        # T3: stuck sayacı durum alanları (zaman tabanlı; ground_speed integrali).
        self.stuck_timeout_s = float(self.get_parameter("stuck_timeout_s").value)
        self.stuck_min_dist_m = float(self.get_parameter("stuck_min_dist_m").value)
        self.stuck_vx_threshold = float(self.get_parameter("stuck_vx_threshold").value)
        self._stuck_dist_m = 0.0          # son sıfırlamadan bu yana ölçülen ilerleme
        self._stuck_since: Optional[float] = None   # sürekli düşük-hız başlangıcı (ROS zaman)
        # Koridor hedefi histerezisi: conf>=0.7'de devreye girer, <0.5'e düşmeden
        # çıkmaz. Hedefin tick'ten tick'e zıplamasını (yalpalama) önler.
        self._corridor_goal_on = False
        # B6: slow_planner KALICI örnek — handle_costmap_navigation her slow tick'te
        # yeni DwaPlanner kuruyordu (cache boş -> ~120ms/tick, hit oranı 0).
        # Aynı parametrelerle bir kez kurulur; hız ölçeği her tick ayarlanır.
        self.dwa_slow = DwaPlanner(
            max_speed_mps=self.dwa_slow_vx,
            max_yaw_rate_deg_s=self.max_yaw_rate,
            vx_steps=int(self.get_parameter("dwa_vx_steps").value),
            yaw_steps=int(self.get_parameter("dwa_yaw_steps").value),
            sim_time_s=float(self.get_parameter("dwa_sim_time").value),
            sim_step_s=float(self.get_parameter("dwa_sim_step").value),
            accel_limits={
                "max_accel_mps2": float(self.get_parameter("dwa_accel_max").value),
                "max_decel_mps2": 1.2,
                "max_yaw_accel_deg_s2": 180.0,
            },
            w={
                "obstacle": float(self.get_parameter("dwa_w_obstacle").value),
                "heading": float(self.get_parameter("dwa_w_heading").value),
                "progress": float(self.get_parameter("dwa_w_progress").value),
                "speed": float(self.get_parameter("dwa_w_speed").value),
                "smooth": float(self.get_parameter("dwa_w_smooth").value),
                "unknown": float(self.get_parameter("dwa_w_unknown").value),
                "corridor": float(self.get_parameter("dwa_w_corridor").value),
                "avoid": float(self.get_parameter("dwa_w_avoid").value),
            },
            recovery_vx=float(self.get_parameter("dwa_recovery_vx").value),
            recovery_yaw_deg_s=float(self.get_parameter("dwa_recovery_yaw").value),
            unknown_slow_vx=self.dwa_slow_vx,
            min_drive_speed_mps=float(self.get_parameter("dwa_min_drive_vx").value),
            min_turn_rate_deg_s=float(self.get_parameter("dwa_min_turn_rate_deg_s").value),
            recovery_latch_ticks=int(self.get_parameter("dwa_recovery_latch_ticks").value),
            align_before_drive_deg=float(self.get_parameter("dwa_align_before_drive_deg").value),
            align_yaw_rate_deg_s=float(self.get_parameter("dwa_align_yaw_rate_deg_s").value),
            avoid_direction_latch_ticks=int(
                self.get_parameter("dwa_avoid_direction_latch_ticks").value
            ),
            bot_radius_m=0.0,
            cell_m=float(self.get_parameter("costmap_cell_m").value),
            n_cells=int(round(float(self.get_parameter("costmap_size_m").value)
                              / float(self.get_parameter("costmap_cell_m").value))),
            max_speed_scale=1.0,
        )
        # Hız governor'ı durumu (idaws:302-331): sensör menzili (safety_max_m)
        # hıza bağlı güvenlik yarıçapını aşıyorsa DWA hız ölçeği düşürülür.
        # Kapalı çevrim integral: ölü bant 0.25 m/s, gain 0.03, clamp [0.05, 1.0].
        self._v_max_sensor = speed_for_safety_radius(
            self.safety_max_m,
            self.safety_base_m,
            self.reaction_time_s,
            self.max_decel_mps2,
        )
        # Başlangıç ölçeği: nominal tavan sensör sınırının altındaysa 1.0'da
        # kalır (B1); aşarsa sensör sınırına çekilir (min ile tavanı aşmaz).
        # 4.045/0.8 = 5.06 -> min(1.0, 5.06) = 1.0: nominal 0.8 zaten güvenli.
        self._speed_scale_target = min(
            1.0, self._v_max_sensor / max(0.01, self.max_speed)
        )
        self._speed_warned = False
        self.get_logger().info(
            f"v_max_sensor={self._v_max_sensor:.2f} m/s, "
            f"max_speed={self.max_speed:.2f} m/s, "
            f"governor_scale={self._speed_scale_target:.2f}"
        )
        self.last_dwa: Optional[DwaCommand] = None
        self.costmap_pub = self.create_publisher(String, "/planning/costmap", 10)  # canlı harita — volatile OK
        self._costmap_pub_div = 0  # costmap yayın bölücüsü (her 4. tick'te)

        self.state = "WAIT_MISSION"
        self.waypoints: List[Dict[str, Any]] = []
        self.raw_waypoints: List[Dict[str, Any]] = []  # B11: orijinal hakem noktaları (NN için)
        self.bench_override_wp: Optional[int] = None
        self._mission_fingerprint: Optional[tuple] = None
        self._last_mission_control_token: Optional[int] = None
        self.current_wp = 0
        self.target_color = ""
        self.started = False
        # GÜVENLİK DURUMU: P1 navigasyonunun başladığı tick'te True olur;
        # geometri dışına çıkmak bunu ASLA çözmez ve P2 boyunca korunur.
        # Yalnız yeni mission/reset temizler.
        self.strict_orange_latched = False
        self.course_geometry_p1 = CourseGeometryTracker(
            self.course_geometry_enter_m, self.course_geometry_exit_m
        )
        self.course_geometry_p2 = CourseGeometryTracker(
            self.course_geometry_enter_m, self.course_geometry_exit_m
        )
        self.course_geometry_status = CourseGeometryStatus(False, None, None)
        self.engage_started: Optional[float] = None
        self._p3_target_seen_since: Optional[float] = None
        self._p3_target_seen_frames = 0
        self._p3_target_last_buoy_seq = -1
        self._p3_locked_target_id: Optional[str] = None
        # Hedef yalnız lock süre/kare sözleşmesini sağladıktan sonra lidar
        # near-field engel listesinden ayrılabilir. Böylece tek hatalı renk
        # karesi gerçek bir engeli güvenlik katmanından çıkaramaz.
        self._p3_target_confirmed_latched: bool = False
        self._p3_target_lost_since: Optional[float] = None
        self._p3_last_tracking_command: Optional[Command] = None
        self._p3_last_target_distance_m: Optional[float] = None
        # Son lidar-fused hedef gözleminin dünya koordinatı. Kısa kamera/model
        # kesintisinde ham motor komutu tekrarlanmaz; sabit dubaya ait bu geçici
        # hedefe bearing her tick güncel GPS/heading ile yeniden hesaplanır.
        self._p3_last_target_fix: Optional[tuple[float, float]] = None
        # P3 tek arama otoritesi: anchor scan -> 2 m reposition -> scout scan
        # -> anchor return. Dış mission listesine waypoint eklenmez.
        self._p3_search_anchor: Optional[tuple[float, float]] = None
        self._p3_search_scout: Optional[tuple[float, float]] = None
        self._p3_search_reposition_bearing_deg: Optional[float] = None
        self._p3_search_phase: str = "anchor_scan"
        self._p3_scan_last_heading_deg: Optional[float] = None
        self._p3_scan_accum_deg: float = 0.0
        # Dış state/MAVLink sözleşmesini değiştirmeyen P2→P3 alt fazı.
        self._p2_exit_advance_origin: Optional[tuple[float, float]] = None
        self._p2_exit_advance_bearing_deg: Optional[float] = None
        self._p2_exit_align_direction = 0.0
        self._p3_reacquisition_started: bool = False
        # Başka bir sert engel P3 hareket komutunu eziyorsa üst durum makinesi
        # hedef kayıp/engage sürelerini ilerletmez. Engel kalkınca aynı hedef
        # niyeti kaldığı yerden devam eder; anchor araması yanlışlıkla başlamaz.
        self._p3_safety_pause_active: bool = False
        self._p3_safety_pause_started: Optional[float] = None
        # Initial hold yalnız P3'e ilk girişte uygulanır. Hedef kaybı arama
        # döngüsünü sıfırlayabilir fakat bu parkur-giriş zamanını sıfırlamaz.
        self._p3_initial_hold_started: Optional[float] = None
        self._buoy_frame_seq = 0
        self.parkur_start_time: float = 0.0
        self.p3_hold_started: Optional[float] = None
        self.failsafe_reason: str = ""
        self._failsafe_resume_state: Optional[str] = None
        self.failsafe_entered_time: float = 0.0
        # B12: P3 hedef renk eksikliği failsafe retry sayacı — 3 denemeden sonra
        # COMPLETE (renk hiç gelmezse 60s+5s sonsuz döngü 20dk süreyi tüketirdi).
        self.p3_target_retries: int = 0
        self.p3_target_max_retries: int = 3
        self.ts3_risk: int = 0
        self.p1_stats_cache: Dict[str, Any] = {}
        self.p2_stats_cache: Dict[str, Any] = {}
        self.p1_stats: Optional[Any] = None
        self.p2_stats: Optional[Any] = None
        # Puan ceza modülü anlık durumu (publish'te JSON'a serilir).
        self.score_reports: Dict[int, Any] = {}
        self.yellow_count: int = 0
        self.yellow_threshold: int = self.yellow_sighting_threshold

        self.pair_detector_p1 = PairCrossingDetector(
            max_pair_lateral_m=self.pair_max_lateral_m,
            min_lateral_gap_m=self.pair_min_lateral_gap_m,
            cross_trigger_forward_m=self.pair_cross_trigger_forward_m,
            cross_complete_forward_m=self.pair_cross_complete_forward_m,
            min_confidence=self.pair_min_confidence,
            max_age_s=self.pair_max_age_s,
            max_pair_forward_gap_m=self.pair_max_forward_gap_m,
            max_pair_crossing_gap_s=self.pair_max_crossing_gap_s,
            allow_sim_truth_geometry=self.sim_gate_truth_enabled,
        )
        self.pair_detector_p2 = PairCrossingDetector(
            max_pair_lateral_m=self.pair_max_lateral_m,
            min_lateral_gap_m=self.pair_min_lateral_gap_m,
            cross_trigger_forward_m=self.pair_cross_trigger_forward_m,
            cross_complete_forward_m=self.pair_cross_complete_forward_m,
            min_confidence=self.pair_min_confidence,
            max_age_s=self.pair_max_age_s,
            max_pair_forward_gap_m=self.pair_max_forward_gap_m,
            max_pair_crossing_gap_s=self.pair_max_crossing_gap_s,
            allow_sim_truth_geometry=self.sim_gate_truth_enabled,
        )

        # Puan ceza modülü: temas + parkur dışı sayaçları P1/P2 için ayrı,
        # puan hesaplayıcı tek (parkur argümanıyla hangi formülün kullanılacağını
        # seçer). Sayaçlar parkur geçişlerinde reset edilir (P1 çarpmaları P2'ye
        # sızmaz). Puan hesabı navigasyonu ASLA engellemez (estimated=true).
        self.contact_p1 = ContactCounter(
            contact_radius_m=float(self.get_parameter("score_contact_radius_m").value),
            sustained_contact_s=float(self.get_parameter("score_sustained_contact_s").value),
            drop_after_s=float(self.get_parameter("score_drop_after_s").value),
        )
        self.contact_p2 = ContactCounter(
            contact_radius_m=float(self.get_parameter("score_contact_radius_m").value),
            sustained_contact_s=float(self.get_parameter("score_sustained_contact_s").value),
            drop_after_s=float(self.get_parameter("score_drop_after_s").value),
        )
        self.ooc_p1 = OutOfCourseDetector(
            mode=str(self.get_parameter("score_ooc_mode").value),
            half_width_m=float(self.get_parameter("score_half_width_m").value),
        )
        self.ooc_p2 = OutOfCourseDetector(
            mode=str(self.get_parameter("score_ooc_mode").value),
            half_width_m=float(self.get_parameter("score_half_width_m").value),
        )
        self.score_calc = ScoreCalculator(
            kd1=float(self.get_parameter("score_kd1").value),
            kd2=float(self.get_parameter("score_kd2").value),
            ed2=float(self.get_parameter("score_ed2").value),
            include_uav_bonus=bool(self.get_parameter("score_include_uav_bonus").value),
        )
        self.score_pub = self.create_publisher(String, "/autonomy/score", 10)
        self.yellow_counter = YellowGateCounter(
            threshold=self.yellow_sighting_threshold,
            decay=self.yellow_sighting_decay,
            range_m=self.yellow_sighting_range_m,
            forward_m=self.yellow_sighting_forward_m,
            lateral_m=self.yellow_sighting_lateral_m,
            min_confidence=self.yellow_sighting_min_confidence,
        )

        self.telemetry: Optional[Dict[str, Any]] = None
        self.buoys: List[Dict[str, Any]] = []
        self.obstacles: List[Dict[str, Any]] = []
        self.sim_gate_truth: List[Dict[str, Any]] = []
        self.last_telemetry_time = 0.0
        self.last_buoy_time = 0.0
        self.last_obstacle_time = 0.0
        self.last_gate_truth_time = 0.0

        self.create_subscription(String, "/mission/waypoints", self.on_waypoints, 10)
        self.create_subscription(String, "/mission/start", self.on_start, 10)
        self.create_subscription(String, "/mission/target_color", self.on_target_color, 10)
        self.create_subscription(String, "/telemetry/state", self.on_telemetry, 20)
        self.create_subscription(String, "/perception/buoys", self.on_buoys, 20)
        self.create_subscription(String, "/perception/obstacles", self.on_obstacles, 20)
        self.create_subscription(
            String, "/perception_sim/gate_truth", self.on_gate_truth, 20
        )

        self.cmd_pub = self.create_publisher(Twist, "/autonomy/cmd_vel_body", 10)
        self.mission_control_ack_pub = self.create_publisher(
            String, "/mission/control_ack", 10
        )
        self.state_pub = self.create_publisher(String, "/autonomy/state", 10)
        self.debug_pub = self.create_publisher(String, "/autonomy/debug", 10)

        self.create_timer(1.0 / self.loop_hz, self.tick)
        self.get_logger().info("IDA autonomy state machine started")

    # --- Yardımcılar ---------------------------------------------------------

    def now_sec(self) -> float:
        """ROS2 clock'un saniye cinsinden zamanı (deterministik tick mantığı)."""
        return self.get_clock().now().nanoseconds / 1e9

    def _score_events(self, now: float) -> List[Dict[str, Any]]:
        """Puan ceza modülünden itiraz adayı olayları üretir (events_logger için).

        Contact olayları kapanınca (sustained/closed) yayınlanır: ``counted``
        her 30 sn'lik SÜREKLİ temas bloğunda +2 artar (1 blok -> 2, 2 blok -> 4;
        şartname 840-843). Parkur dışı olaylar ``times_out`` arttığında yazılır;
        ``detail`` ayrıca yazılmazsa olay adayı yine de tek satır üretir
        (events.csv izleme).

        Determinizm: ``now`` her yayında self.now_sec()'ten gelir; olay zamanı
        ``t`` olayın kapanış/başlangıç anıdır.
        """
        events: List[Dict[str, Any]] = []
        # contact: kapalı (bitmiş) olaylar — parkur geçişinde close_stale ile
        # kapatılır, P1/P2 puan dökümüne katılır.
        for counter, parkur in ((self.contact_p1, 1), (self.contact_p2, 2)):
            for event in counter.get_events_all():
                if not event.closed:
                    continue
                detail = (
                    f"counted={event.counted} sustained={event.sustained}"
                    f" source={event.source} color={event.color}"
                    f" start_t={event.start_t:.1f} last_t={event.last_t:.1f}"
                )
                events.append(
                    {"t": event.last_t, "type": "contact", "parkur": parkur, "key": event.key, "detail": detail}
                )
        # out_of_course: parkur dışına çıkış olayı YALNIZ times_out değiştiğinde
        # üretilir (O2) — her tick tekrarlanan satır üretilmez; events_logger geç
        # başlasa bile değişim anındaki tek olay yayınlanır.
        for detector, parkur in ((self.ooc_p1, 1), (self.ooc_p2, 2)):
            if detector.times_out_changed() and detector.times_out > 0:
                events.append(
                    {
                        "t": now,
                        "type": "out_of_course",
                        "parkur": parkur,
                        "key": "outside",
                        "detail": f"times_out={detector.times_out} cumulative_out_s={detector.cumulative_out_s:.1f}",
                    }
                )
        return events

    def telemetry_ok(self, now: float) -> bool:
        return self.telemetry is not None and now - self.last_telemetry_time <= self.telemetry_timeout

    def buoys_ok(self, now: float) -> bool:
        return now - self.last_buoy_time <= self.perception_timeout

    @staticmethod
    def _p3_target_identity(item: Dict[str, Any]) -> Optional[str]:
        """Prefer the stable lidar track over the frame-local camera ID."""
        for key in ("lidar_obstacle_id", "id"):
            value = item.get(key)
            if value is not None and str(value).strip():
                return f"{key}:{str(value).strip()}"
        return None

    def _p3_visible_target_candidates(self) -> List[Dict[str, Any]]:
        candidates = [
            item for item in self.buoys
            if isinstance(item, dict)
            and str(item.get("color", "")).lower() == self.target_color
            and float(item.get("confidence", 0.0)) >= self.target_min_confidence
        ]
        if self._p3_locked_target_id is not None:
            return [
                item for item in candidates
                if self._p3_target_identity(item) == self._p3_locked_target_id
            ]
        if not candidates:
            return []
        # LİDAR DOĞRULAMASI (fail-closed): hedef duba, self.obstacles içinde
        # aynı lidar_obstacle_id'li bir hard engelle eşleşmeli. Modeller iyi
        # olmadığından fusion yanlış renk atayabilir; yalnız renkli ama lidar
        # karşılığı olmayan (veya mesafesi uyuşmayan) bir duba hedef SAYILMAZ.
        # Kamera-only yanlış konum angajman üretmemesi kuralının uygulaması.
        lidar_by_id: Dict[str, Dict[str, Any]] = {}
        for obs in getattr(self, "obstacles", []) or []:
            if not isinstance(obs, dict):
                continue
            obs_id = obs.get("lidar_obstacle_id", obs.get("id"))
            if obs_id is not None:
                lidar_by_id[str(obs_id)] = obs
        verified: List[Dict[str, Any]] = []
        for item in candidates:
            lidar_id = item.get("lidar_obstacle_id")
            if lidar_id is None:
                continue  # lidar eşleşmesi yok -> hedef olamaz (fail-closed)
            obs = lidar_by_id.get(str(lidar_id))
            if obs is None:
                continue  # obstacles'ta karşılığı yok -> hayali eşleşme
            # Mesafe tutarlılığı: hedef duba ile lidar engeli aynı fiziksel
            # noktada olmalı (1.0 m tolerans — fusion yanlış eşleştirirse).
            try:
                dist_item = float(item.get("distance", 99.0))
                dist_obs = float(obs.get("distance", 99.0))
            except (TypeError, ValueError):
                continue
            if abs(dist_item - dist_obs) > 1.0:
                continue
            verified.append(item)
        if not verified:
            return []
        chosen = min(verified, key=lambda item: float(item.get("distance", 99.0)))
        identity = self._p3_target_identity(chosen)
        if identity is not None:
            self._p3_locked_target_id = identity
        return [chosen]

    def _p3_tracking_buoys(
        self, selected_targets: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """Keep wrong-color risk inputs but quarantine other same-color tracks."""
        return selected_targets + [
            item for item in self.buoys
            if isinstance(item, dict)
            and str(item.get("color", "")).lower() != self.target_color
        ]

    @staticmethod
    def _p3_obstacle_ids(item: Dict[str, Any]) -> set[str]:
        """Return every stable lidar identity carried by one fused object."""

        identities: set[str] = set()
        if not isinstance(item, dict):
            return identities
        for key in ("lidar_obstacle_id", "id"):
            value = item.get(key)
            if value is not None and str(value).strip():
                identities.add(str(value).strip())
        return identities

    def _p3_selected_target_obstacle_ids(
        self, selected_targets: List[Dict[str, Any]]
    ) -> set[str]:
        """Obstacle IDs that belong to the confirmed engagement target.

        The target is removed only from the P3 near-field input. It remains in
        the fused obstacle stream for lidar verification, logging and every
        other parkur. Confirmation is deliberately latched first so a one-frame
        color error cannot whitelist a physical obstacle.
        """

        if not getattr(self, "_p3_target_confirmed_latched", False):
            return set()
        identities: set[str] = set()
        for target in selected_targets:
            identities.update(self._p3_obstacle_ids(target))
        locked = getattr(self, "_p3_locked_target_id", None)
        if locked is not None:
            text = str(locked).strip()
            if ":" in text:
                text = text.split(":", 1)[1]
            if text:
                identities.add(text)
        return identities

    def _p3_non_target_obstacles(
        self, selected_targets: List[Dict[str, Any]]
    ) -> tuple[List[Dict[str, Any]], set[str]]:
        """Keep every hard obstacle except the confirmed contact target."""

        ignored = self._p3_selected_target_obstacle_ids(selected_targets)
        if not ignored:
            return list(getattr(self, "obstacles", []) or []), ignored
        filtered = [
            obstacle
            for obstacle in (getattr(self, "obstacles", []) or [])
            if self._p3_obstacle_ids(obstacle).isdisjoint(ignored)
        ]
        return filtered, ignored

    def _update_p3_safety_pause(self, active: bool, now: float) -> None:
        """Freeze P3 phase timers while a non-target obstacle owns motion."""

        was_active = bool(getattr(self, "_p3_safety_pause_active", False))
        started = getattr(self, "_p3_safety_pause_started", None)
        if active:
            if not was_active or started is None:
                self._p3_safety_pause_started = now
            self._p3_safety_pause_active = True
            return
        if was_active and started is not None:
            paused_s = max(0.0, now - float(started))
            # ENGAGE contact penceresi gerçek hareket süresidir; güvenlik
            # pivotunda geçen süre görevi tamamlamış sayılmaz.
            if self.engage_started is not None:
                self.engage_started += paused_s
            # Defensive: çağıran dal eski bir loss başlangıcını koruduysa aynı
            # süreyi ona da ekle. Normal P3 akışı pause boyunca loss'u başlatmaz.
            if self._p3_target_lost_since is not None:
                self._p3_target_lost_since += paused_s
        self._p3_safety_pause_active = False
        self._p3_safety_pause_started = None

    def _p3_target_memory_expired(self, now: float) -> bool:
        """True only after the remembered fused target exceeded its TTL.

        ``p3_target_loss_grace_s`` smooths the immediate camera dropout;
        this longer timeout owns the decision to abandon the remembered world
        coordinate and return to the P3 search anchor. Safety pauses shift the
        loss timestamp, so obstacle avoidance never consumes this budget.
        """

        lost_since = getattr(self, "_p3_target_lost_since", None)
        if lost_since is None:
            return False
        return now - float(lost_since) > self.p3_target_memory_timeout_s

    def _p3_guard_tracking_command(
        self,
        candidate: Command,
        debug: Dict[str, Any],
        selected_targets: List[Dict[str, Any]],
        now: float,
    ) -> tuple[Command, Dict[str, Any], bool]:
        """Apply one P3 safety owner without treating the target as a hazard."""

        obstacles, ignored = self._p3_non_target_obstacles(selected_targets)
        guarded, debug = self._near_field_guarded(
            candidate,
            debug,
            allow_reverse_escape=False,
            obstacles_override=obstacles,
        )
        near_field = debug.get("near_field", {})
        paused = bool(near_field.get("active", False))
        self._update_p3_safety_pause(paused, now)
        debug["p3_safety_pause"] = {
            "active": paused,
            "started": self._p3_safety_pause_started,
            "ignored_target_obstacle_ids": sorted(ignored),
            "remaining_obstacle_count": len(obstacles),
        }
        return guarded, debug, paused

    def obstacles_ok(self, now: float) -> bool:
        return now - self.last_obstacle_time <= self.perception_timeout

    def _refresh_course_geometry(self) -> CourseGeometryStatus:
        """Aktif parkurun gerçek polyline durumunu güncelle.

        Bu bilgi yalnız telemetri/debug/puanlama gözlemidir; sert turuncu
        güvenlik latch'ini değiştirmez.
        """

        telemetry_fresh = self.telemetry is not None and self.telemetry_ok(self.now_sec())
        active_parkur = course_geometry_parkur(
            self.state,
            telemetry_fresh=telemetry_fresh,
            failsafe_resume_state=self._failsafe_resume_state,
        )
        if self.telemetry is None or active_parkur is None:
            self.course_geometry_status = CourseGeometryStatus(False, None, None)
            return self.course_geometry_status
        if active_parkur == 1:
            tracker = self.course_geometry_p1
        else:
            tracker = self.course_geometry_p2
        self.course_geometry_status = tracker.update(
            self.telemetry.get("lat"), self.telemetry.get("lon")
        )
        return self.course_geometry_status

    def _course_geometry_debug(self) -> Dict[str, Any]:
        status = self._refresh_course_geometry()
        return {
            "strict_orange_latched": self.strict_orange_latched,
            "inside_course_geometry": status.inside,
            "course_geometry_valid": status.valid,
            "distance_from_course_m": (
                round(status.distance_m, 3) if status.distance_m is not None else None
            ),
            # Geriye uyumluluk: eski alan artık latch değil, gerçek geometrinin
            # deprecated alias'ıdır. Geometri bilinmiyorsa null kalır.
            "in_corridor": status.inside,
        }

    def is_last_p2_wp(self) -> bool:
        """Son P2 waypoint'ine ulaşılıp ulaşılmadığı (geçiş koşulu için).

        current_wp bir sonraki hedef waypoint'in index'idir; geriye yalnızca son
        waypoint kaldıysa (current_wp >= len - 1) şart sağlanmış sayılır. Parkur
        alanı kontrolü YAPILMAZ: P2 tamamlama koşulu yalnızca rota pozisyonuna
        bağlıdır (tasarım kararı). Saf mantık planner.is_last_waypoint_index'tedir.
        """
        return is_last_waypoint_index(self.current_wp, len(self.waypoints))

    def transition_to_p2(self, now: float) -> None:
        """PARKUR_1_NAV -> PARKUR_2_AVOIDANCE geçişi: detektörleri sıfırla.

        Puan ceza modülünde P1 sayaçları (contact_p1/ooc_p1) P1 puan dökümünü
        üretir ve reset edilir; P2 sayaçları temiz başlar. P1 çarpmaları P2'ye
        sızmaz.
        """
        self.state = "PARKUR_2_AVOIDANCE"
        self.strict_orange_latched = update_strict_orange_latch(
            self.strict_orange_latched, self.state
        )
        self.parkur_start_time = now
        self.pair_detector_p2.reset()
        self.yellow_counter.reset()
        self.contact_p1.close_stale(now)
        self.score_reports[1] = self.score_calc.report(
            parkur=1,
            g1=int(self.p1_stats_cache.get("crossed_count", 0)),
            contact1=self.contact_p1.total_counted(),
            ooc1=self.ooc_p1.times_out,
        )
        # ooc_p1 burada resetlenmez: pending times_out olayı aynı tick'in
        # publish/_score_events adımında tüketilmelidir.
        self.contact_p2.reset()
        self.ooc_p2.reset()
        self._clear_p2_exit_advance()
        self.get_logger().info("Transition to PARKUR_2_AVOIDANCE")

    def _clear_p2_exit_advance(self) -> None:
        self._p2_exit_advance_origin = None
        self._p2_exit_advance_bearing_deg = None
        self._p2_exit_align_direction = 0.0

    def _p2_terminal_route_bearing_deg(self) -> float:
        """Son hakem rota parçasının yönünü döndür.

        Mission listesine yeni waypoint eklenmez. Son P2 noktası ile ondan
        önceki farklı hakem noktası kullanılır; tek noktalı rotada mevcut
        araç heading'i kontrollü fallback'tir.
        """

        route = self.raw_waypoints or self.waypoints
        last_p2 = None
        for index, waypoint in enumerate(route):
            if int(waypoint.get("parkur", 1)) == 2:
                last_p2 = index
        if last_p2 is not None:
            end = route[last_p2]
            for index in range(last_p2 - 1, -1, -1):
                start = route[index]
                if haversine_m(
                    float(start["lat"]), float(start["lon"]),
                    float(end["lat"]), float(end["lon"]),
                ) > 0.05:
                    return bearing_deg(
                        float(start["lat"]), float(start["lon"]),
                        float(end["lat"]), float(end["lon"]),
                    )
        return float(self.telemetry.get("heading_deg", 0.0)) % 360.0

    def begin_p2_exit_advance(
        self, now: float, *, p2_completed: bool = False
    ) -> bool:
        """Latch P2 completion and start the internal 5 m exit sub-phase."""

        if self.state != "PARKUR_2_AVOIDANCE" or not p2_completed:
            self.enter_failsafe("parkur2_incomplete_transition", now)
            return False
        if self.p2_exit_advance_m <= 0.0:
            return self.transition_to_p3(now, p2_completed=True)
        if self.telemetry is None:
            self.enter_failsafe("parkur2_incomplete_transition", now)
            return False

        start_lat = float(self.telemetry["lat"])
        start_lon = float(self.telemetry["lon"])
        route_bearing = self._p2_terminal_route_bearing_deg()
        self._p2_exit_advance_origin = (start_lat, start_lon)
        self._p2_exit_advance_bearing_deg = route_bearing
        initial_error = normalize_angle_deg(
            route_bearing - float(self.telemetry["heading_deg"])
        )
        self._p2_exit_align_direction = (
            1.0 if initial_error > 0.0 else (-1.0 if initial_error < 0.0 else 0.0)
        )
        self.nav_arbiter.reset()
        self._stuck_since = None
        self.get_logger().info(
            "P2 route complete; internal exit advance started: "
            f"distance={self.p2_exit_advance_m:.1f}m "
            f"bearing={route_bearing:.1f}deg"
        )
        return True

    def handle_p2_exit_advance(
        self, now: float
    ) -> tuple[Command, Dict[str, Any]]:
        """Drive the internal P2 exit leg without mutating mission waypoints."""

        origin = self._p2_exit_advance_origin
        route_bearing = self._p2_exit_advance_bearing_deg
        if self.telemetry is None or origin is None or route_bearing is None:
            self.enter_failsafe("parkur2_incomplete_transition", now)
            return Command(0.0, 0.0, 0.0, "failsafe p2_exit_unavailable"), {}

        displacement = latlon_to_local_m(
            origin[0], origin[1],
            float(self.telemetry["lat"]), float(self.telemetry["lon"]),
        )
        bearing_rad = math.radians(route_bearing)
        progress_m = (
            displacement.x * math.cos(bearing_rad)
            + displacement.y * math.sin(bearing_rad)
        )
        remaining_m = max(0.0, self.p2_exit_advance_m - progress_m)
        debug: Dict[str, Any] = {
            "p2_exit_advance": {
                "active": True,
                "progress_m": progress_m,
                "remaining_m": remaining_m,
                "target_m": self.p2_exit_advance_m,
                "route_bearing_deg": route_bearing,
            }
        }
        if progress_m >= self.p2_exit_advance_m:
            self.transition_to_p3(now, p2_completed=True)
            return Command(0.0, 0.0, 0.0, "p2_exit_advance_complete"), debug

        yaw_error = normalize_angle_deg(
            route_bearing - float(self.telemetry["heading_deg"])
        )
        # Tam arka eksende ±180° aynı fiziksel yönü ifade eder; küçük pose
        # gürültüsü işareti her tick değiştirirse iki motor sağ/sol arasında
        # salınır. Çıkış başında seçilen dönüş yönünü arka belirsizlik
        # bölgesinden çıkana kadar koru, sonra normal en kısa-yol hatasına dön.
        latch_release = getattr(
            self, "p2_exit_align_latch_release_deg", 150.0
        )
        latched_direction = getattr(self, "_p2_exit_align_direction", 0.0)
        if abs(yaw_error) >= latch_release and latched_direction != 0.0:
            yaw_error = math.copysign(abs(yaw_error), latched_direction)
        elif abs(yaw_error) < latch_release and abs(yaw_error) > 1e-6:
            self._p2_exit_align_direction = math.copysign(1.0, yaw_error)
        yaw_rate_deg_s = clamp(
            yaw_error * self.yaw_kp, -self.max_yaw_rate, self.max_yaw_rate
        )
        aligned = abs(yaw_error) <= 35.0
        candidate = Command(
            self.p2_exit_advance_speed_mps if aligned else 0.0,
            0.0,
            math.radians(yaw_rate_deg_s),
            "p2_exit_advance" if aligned else "p2_exit_align",
        )
        yaw_error_rad = math.radians(yaw_error)
        goal_body = (
            remaining_m * math.cos(yaw_error_rad),
            remaining_m * math.sin(yaw_error_rad),
        )
        command, debug, _ = self._arbitrate_navigation_candidate(
            candidate, debug, goal_body=goal_body, now=now
        )
        return command, debug

    def _initialize_p3_search_anchor(
        self, reposition_bearing_deg: Optional[float] = None
    ) -> bool:
        """Capture the P3 entry position without mutating the judge mission."""

        if self.telemetry is None:
            return False
        try:
            lat = float(self.telemetry["lat"])
            lon = float(self.telemetry["lon"])
            heading = float(self.telemetry["heading_deg"])
            bearing = (
                heading if reposition_bearing_deg is None
                else float(reposition_bearing_deg)
            ) % 360.0
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
        if not all(math.isfinite(value) for value in (lat, lon, heading, bearing)):
            return False
        bearing_rad = math.radians(bearing)
        scout = local_m_to_latlon(
            lat,
            lon,
            self.p3_search_reposition_m * math.cos(bearing_rad),
            self.p3_search_reposition_m * math.sin(bearing_rad),
        )
        self._p3_search_anchor = (lat, lon)
        self._p3_search_scout = scout
        self._p3_search_reposition_bearing_deg = bearing
        self._start_p3_scan("anchor_scan")
        return True

    def _start_p3_scan(self, phase: str) -> None:
        if phase not in {"anchor_scan", "scout_scan"}:
            raise ValueError("invalid P3 scan phase")
        self._p3_search_phase = phase
        self._p3_scan_accum_deg = 0.0
        self._p3_scan_last_heading_deg = (
            float(self.telemetry["heading_deg"]) % 360.0
            if self.telemetry is not None
            else None
        )

    def _set_p3_return_to_anchor(self) -> None:
        self._p3_search_phase = "return_anchor"
        self._p3_scan_accum_deg = 0.0
        self._p3_scan_last_heading_deg = None

    def _p3_transit_command(
        self, target: tuple[float, float], action: str
    ) -> tuple[Command, float]:
        forward_m, right_m = self.goal_body_forward_m(
            {"lat": target[0], "lon": target[1]}
        )
        distance_m = math.hypot(forward_m, right_m)
        bearing_error_deg = math.degrees(math.atan2(right_m, forward_m))
        yaw_rate_deg_s = clamp(
            bearing_error_deg * self.yaw_kp,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )
        aligned = forward_m > 0.0 and abs(bearing_error_deg) <= 35.0
        return Command(
            self.p3_search_transit_speed_mps if aligned else 0.0,
            0.0,
            math.radians(yaw_rate_deg_s),
            action if aligned else f"{action}_align",
        ), distance_m

    def _p3_anchor_search_command(
        self, debug: Dict[str, Any]
    ) -> tuple[Command, Dict[str, Any]]:
        """Single P3 search authority based on measured heading and geometry."""

        if self.telemetry is None:
            return Command(0.0, 0.0, 0.0, "p3_search_no_telemetry"), debug
        if self._p3_search_anchor is None or self._p3_search_scout is None:
            if not self._initialize_p3_search_anchor():
                return Command(0.0, 0.0, 0.0, "p3_search_anchor_invalid"), debug

        phase = self._p3_search_phase
        heading = float(self.telemetry["heading_deg"]) % 360.0
        if phase in {"anchor_scan", "scout_scan"}:
            if self._p3_scan_last_heading_deg is None:
                self._p3_scan_last_heading_deg = heading
            self._p3_scan_accum_deg, self._p3_scan_last_heading_deg = p3_scan_progress(
                self._p3_scan_accum_deg,
                self._p3_scan_last_heading_deg,
                heading,
            )
            if self._p3_scan_accum_deg >= self.p3_search_scan_degrees:
                if phase == "anchor_scan":
                    self._p3_search_phase = "reposition"
                else:
                    self._set_p3_return_to_anchor()
                return Command(0.0, 0.0, 0.0, f"p3_{phase}_complete"), debug
            command = Command(
                0.0,
                0.0,
                math.radians(self.p3_search_yaw_rate_deg_s),
                f"p3_{phase} actual={self._p3_scan_accum_deg:.1f}",
            )
        elif phase == "reposition":
            assert self._p3_search_scout is not None
            command, distance_m = self._p3_transit_command(
                self._p3_search_scout, "p3_reposition"
            )
            if distance_m <= self.p3_search_reposition_arrival_m:
                self._start_p3_scan("scout_scan")
                return Command(0.0, 0.0, 0.0, "p3_reposition_complete"), debug
            command, debug = self._near_field_guarded(
                command, debug, allow_reverse_escape=False
            )
            # A hard obstacle at the scout point must not start a reverse loop;
            # scan safely from the current reachable viewpoint instead.
            if command.vx <= 0.0 and bool(debug.get("near_field", {}).get("active")):
                self._start_p3_scan("scout_scan")
                command = Command(0.0, 0.0, command.yaw_rate, "p3_reposition_blocked_scan")
        elif phase == "return_anchor":
            assert self._p3_search_anchor is not None
            command, distance_m = self._p3_transit_command(
                self._p3_search_anchor, "p3_return_anchor"
            )
            if distance_m <= self.p3_search_anchor_acceptance_m:
                self._start_p3_scan("anchor_scan")
                return Command(0.0, 0.0, 0.0, "p3_anchor_reached"), debug
            command, debug = self._near_field_guarded(
                command, debug, allow_reverse_escape=False
            )
        else:
            self._set_p3_return_to_anchor()
            command = Command(0.0, 0.0, 0.0, "p3_search_phase_recovered")

        debug["p3_search"] = {
            "phase": self._p3_search_phase,
            "scan_actual_deg": round(self._p3_scan_accum_deg, 2),
            "scan_required_deg": self.p3_search_scan_degrees,
            "anchor": self._p3_search_anchor,
            "scout": self._p3_search_scout,
        }
        return command, debug

    def transition_to_p3(self, now: float, *, p2_completed: bool = False) -> bool:
        """Enter P3 only through a positively confirmed P2 completion."""
        if self.state != "PARKUR_2_AVOIDANCE" or not p2_completed:
            self.enter_failsafe("parkur2_incomplete_transition", now)
            return False
        exit_bearing = self._p2_exit_advance_bearing_deg
        self._clear_p2_exit_advance()
        self.state = "PARKUR_3_TARGET_LOCK"
        self.parkur_start_time = now
        self.pair_detector_p1.reset()
        self.pair_detector_p2.reset()
        self.contact_p1.close_stale(now)
        self.contact_p2.close_stale(now)
        self.score_reports[2] = self.score_calc.report(
            parkur=2,
            g2=int(self.p2_stats_cache.get("crossed_count", 0)),
            contact2=self.contact_p2.total_counted(),
            ooc2=self.ooc_p2.times_out,
        )
        # P2 pending OOC olayı publish edilene kadar korunur. Tüm sayaçlar
        # yalnız maddi olarak yeni mission kurulurken resetlenir.
        self.p3_hold_started = None
        self.engage_started = None
        self._p3_target_seen_since = None
        self._p3_target_seen_frames = 0
        self._p3_target_last_buoy_seq = -1
        self._p3_locked_target_id = None
        self._p3_target_confirmed_latched = False
        self._p3_target_lost_since = None
        self._p3_last_tracking_command = None
        self._p3_last_target_distance_m = None
        self._p3_last_target_fix = None
        self._p3_search_anchor = None
        self._p3_search_scout = None
        self._p3_search_reposition_bearing_deg = None
        self._p3_search_phase = "anchor_scan"
        self._p3_scan_last_heading_deg = None
        self._p3_scan_accum_deg = 0.0
        self._p3_reacquisition_started = False
        self._p3_safety_pause_active = False
        self._p3_safety_pause_started = None
        self._p3_initial_hold_started = now
        if not self._initialize_p3_search_anchor(exit_bearing):
            self.enter_failsafe("p3_search_anchor_unavailable", now)
            return False
        self.get_logger().info("Transition to PARKUR_3_TARGET_LOCK")
        return True

    def enter_failsafe(self, reason: str, now: float) -> None:
        """FAILSAFE durumuna geç; reason'ı sakla (DEBUG/state JSON'a yazılır)."""
        self._failsafe_resume_state = (
            self.state
            if self.state in {"PARKUR_1_NAV", "PARKUR_2_AVOIDANCE"}
            else None
        )
        self.state = "FAILSAFE"
        self.failsafe_reason = reason
        self.failsafe_entered_time = now
        self.get_logger().warn(f"FAILSAFE: {reason}")

    # --- Topic geri çağrıları -------------------------------------------------

    def _interpolate_waypoints(
        self, waypoints: List[Dict[str, Any]], step_m: float = 5.0
    ) -> List[Dict[str, Any]]:
        """Hakem waypoint'leri arasına step_m aralıklı mikro waypoint'ler ekler.

        Ardışık her (lat1, lon1)-(lat2, lon2) çifti için mesafe haversine_m ile
        hesaplanır ve ara nokta sayısı count = int(distance / step_m) olur.
        Ara noktalar t = k / (count + 1) oranlarında lat/lon üzerinde doğrusal
        interpolasyonla üretilir; böylece ardışık nokta aralığı her parçada
        step_m'den küçük kalır (5m aralık hedefi). Orijinal waypoint'ler aynen
        korunur; ara noktalar parçası oldukları doğru parçasının başlangıç
        waypoint'ini kopyalayıp yalnızca lat/lon'u değiştirir, dolayısıyla
        "parkur" alanı ve diğer alanlar korunur. P1/P2 geçiş mantığı bozulmaz:
        parkur değişimi yalnızca orijinal waypoint'te (kendi parkur değeriyle)
        gerçekleşir.

        Edge durumlar: boş liste -> [] ; tek waypoint -> kopya ; distance
        NaN/inf veya step_m <= 0 ise noktalar olduğu gibi kopyalanır
        (interpolasyon atlanır); distance < step_m ise araya nokta girmez.
        """
        if not waypoints:
            return []
        if step_m <= 0.0:
            return list(waypoints)
        interpolated: List[Dict[str, Any]] = []
        for idx in range(len(waypoints) - 1):
            start = waypoints[idx]
            end = waypoints[idx + 1]
            lat1, lon1 = float(start["lat"]), float(start["lon"])
            lat2, lon2 = float(end["lat"]), float(end["lon"])
            interpolated.append(dict(start))  # orijinal başlangıç noktası korunur
            distance = haversine_m(lat1, lon1, lat2, lon2)
            if not math.isfinite(distance) or distance < step_m:
                continue
            count = int(distance / step_m)
            for k in range(1, count + 1):
                t = k / (count + 1)
                point = dict(start)  # parkur ve diğer alanlar segment başından kopyalanır
                point["lat"] = lat1 + (lat2 - lat1) * t
                point["lon"] = lon1 + (lon2 - lon1) * t
                interpolated.append(point)
        if waypoints:
            interpolated.append(dict(waypoints[-1]))  # son orijinal nokta korunur
        return interpolated

    def _reset_for_new_mission(self) -> None:
        """Yeni ve maddi olarak farklı mission için tüm runtime durumunu temizle."""

        self.state = "MISSION_READY"
        self.current_wp = 0
        self.bench_override_wp = None
        self.started = False
        self.strict_orange_latched = update_strict_orange_latch(
            self.strict_orange_latched, self.state, new_mission=True
        )
        self.pair_detector_p1.reset()
        self.pair_detector_p2.reset()
        self.yellow_counter.reset()
        self.contact_p1.reset()
        self.contact_p2.reset()
        self.ooc_p1.reset()
        self.ooc_p2.reset()
        self.course_geometry_p1.reset_state()
        self.course_geometry_p2.reset_state()
        self.course_geometry_status = CourseGeometryStatus(False, None, None)
        self.score_reports.clear()
        self.p1_stats_cache.clear()
        self.p2_stats_cache.clear()
        self.p1_stats = None
        self.p2_stats = None
        self.yellow_count = 0
        self.ts3_risk = 0
        self.p3_target_retries = 0
        self.p3_hold_started = None
        self.engage_started = None
        self._p3_target_seen_since = None
        self._p3_target_seen_frames = 0
        self._p3_target_last_buoy_seq = -1
        self._p3_locked_target_id = None
        self._p3_target_confirmed_latched = False
        self._p3_target_lost_since = None
        self._p3_last_tracking_command = None
        self._p3_last_target_distance_m = None
        self._p3_last_target_fix = None
        self._p3_search_anchor = None
        self._p3_search_scout = None
        self._p3_search_reposition_bearing_deg = None
        self._p3_search_phase = "anchor_scan"
        self._p3_scan_last_heading_deg = None
        self._p3_scan_accum_deg = 0.0
        self._p3_reacquisition_started = False
        self._p3_safety_pause_active = False
        self._p3_safety_pause_started = None
        self._p3_initial_hold_started = None
        self._clear_p2_exit_advance()
        self.last_dwa = None
        self.failsafe_reason = ""
        self._failsafe_resume_state = None
        self._corridor_goal_on = False
        self._stuck_since = None
        self._stuck_dist_m = 0.0
        self._near_field_latch = False
        center_escape = getattr(self, "center_obstacle_escape", None)
        if center_escape is not None:
            center_escape.reset()
        nav_arbiter = getattr(self, "nav_arbiter", None)
        if nav_arbiter is not None:
            nav_arbiter.reset()
        self.sim_gate_truth = []
        self.last_gate_truth_time = 0.0

    def on_waypoints(self, msg: String) -> None:
        payload = loads(msg.data, None)
        valid = sanitize_mission_waypoints(payload)
        if valid is None:
            self.get_logger().warn("Invalid mission waypoints ignored; current mission preserved")
            return
        fingerprint = mission_waypoint_fingerprint(valid)
        if fingerprint == self._mission_fingerprint:
            # telemetry_sim ilk saniyede aynı mission'ı 20 kez yayınlar.
            # Replay aktif state/latch/sayaçları ASLA sıfırlamamalıdır.
            return

        # Aday rota runtime durumuna dokunmadan tamamen hazırlanır; böylece
        # bozuk mesaj mevcut iyi mission'ı yarım/boş rotaya dönüştüremez.
        candidate_raw = [dict(w) for w in valid]
        candidate_waypoints = (
            [dict(w) for w in valid]
            if self.dwa_enabled
            else self._interpolate_waypoints(valid, step_m=5.0)
        )
        p1_route, p2_route = split_parkur_routes(candidate_raw)

        self._reset_for_new_mission()
        self.raw_waypoints = candidate_raw
        self.waypoints = candidate_waypoints
        self.course_geometry_p1.set_route(p1_route)
        self.course_geometry_p2.set_route(p2_route)
        self.course_geometry_status = CourseGeometryStatus(False, None, None)
        self.ooc_p1.set_course(p1_route)
        self.ooc_p2.set_course(p2_route)
        self._mission_fingerprint = fingerprint
        self._apply_bench_p3_override()
        self.get_logger().info(
            f"Loaded {len(valid)} mission waypoints "
            f"(interpolated to {len(self.waypoints)})"
        )

    def _apply_bench_p3_override(self) -> None:
        """Bench-only direct P3 start: mission yüklendiğinde başlangıç WP'sini atla.

        Yalnız bench_p3_only_enabled=True VE bench_p3_start_parkur 1-3 aralığında
        iken etkindir (açıkça etkinleştirilmiş bench launch'ı — production'da bu
        bayraklar kapalıdır, fail-closed). current_wp, parkur >= start_parkur olan
        ilk waypoint'e atlanır; parkur 3 için doğrudan P3 state'i doğar. Bu
        test/karar oturumudur: ARM/motor/mod komutu üretilmez (motor yolu bench
        launch'ında kapalıdır). Hakem P1->P2->P3 sırası değiştirilmez.
        """
        # __new__ ile kurulan test node'larında nitelikler eksik olabilir;
        # eksikse fail-closed davran (production __init__ her zaman ayarlar).
        if not getattr(self, "bench_p3_only_enabled", False):
            return
        if not 1 <= self.bench_p3_start_parkur <= 3:
            self.get_logger().error(
                "Bench P3 override ignored: bench_p3_start_parkur "
                f"({self.bench_p3_start_parkur}) 1-3 dışında"
            )
            return
        if not self.waypoints:
            self.get_logger().warn("Bench P3 override ignored: no accepted waypoints")
            return
        first_parkur = 1
        try:
            first_parkur = int(self.waypoints[0].get("parkur", 1))
        except (TypeError, ValueError):
            pass
        start_at = 0
        for idx, wp in enumerate(self.waypoints):
            try:
                if int(wp.get("parkur", 1)) >= self.bench_p3_start_parkur:
                    start_at = idx
                    break
            except (TypeError, ValueError):
                continue
        # Mission yalnızca parkur < start_parkur içeriyorsa override etkisizdir:
        # başlangıç noktası varsayılan olarak kalır, atlama yapılmaz.
        if start_at == 0 and first_parkur < self.bench_p3_start_parkur:
            self.get_logger().warn(
                "Bench P3 override ignored: mission has no waypoint at or beyond "
                f"parkur {self.bench_p3_start_parkur}"
            )
            return
        self.current_wp = start_at
        self.bench_override_wp = start_at
        self.get_logger().warn(
            f"BENCH P3-ONLY override: direct start at waypoint {start_at} "
            f"(parkur >= {self.bench_p3_start_parkur}); motor yolu kapalı "
            "(dry_run, guided=off, motor_command_enabled=false)"
        )

    def on_start(self, msg: String) -> None:
        payload = loads(msg.data, None)
        should_start = msg.data.strip().lower() in {"start", "true", "1"}
        token: int | None = None
        if isinstance(payload, dict):
            should_start = bool(payload.get("start", payload.get("data", False)))
            raw_token = payload.get("token")
            if raw_token is not None:
                decoded = decode_mission_command_token(raw_token)
                if decoded is None or decoded[1] != should_start:
                    return
                token = decoded[0]
        applied = True
        duplicate = token is not None and token == self._last_mission_control_token
        if duplicate:
            self.get_logger().info(f"Duplicate mission token ACK replay: {token}")
        elif should_start:
            if not self.waypoints:
                applied = False
                self.started = False
                self.get_logger().warn("Mission start ignored: no accepted waypoints")
            else:
                self.started = True
                self.get_logger().info("Mission start received")
        elif token is not None:
            # STOP yeni START'ta rotanın başından ve temiz sayaçlarla
            # başlamak üzere runtime'ı sıfırlar; mission verisini korur.
            self._reset_for_new_mission()
            self.get_logger().info("Mission stop received; autonomy reset to MISSION_READY")

        if token is not None and applied:
            self._last_mission_control_token = token

        if token is not None:
            ack = String()
            ack.data = dumps({
                "stamp": self.now_sec(),
                "token": token,
                "start": should_start,
                "applied": applied,
                "state": self.state,
            })
            self.mission_control_ack_pub.publish(ack)

    def on_target_color(self, msg: String) -> None:
        payload = loads(msg.data, None)
        if isinstance(payload, dict):
            color = str(payload.get("target_color", payload.get("color", ""))).lower()
        else:
            color = msg.data.strip().lower()
        if not color:
            return
        # Şartname uyumu: yalnızca siyah/kırmızı/yeşil (RAL 9005/3026/6037) geçerli.
        # "blue" (MAVİ yedeği kaldırıldı) ve bilinmeyen renkler reddedilir ve loglanır.
        if color not in VALID_TARGET_COLORS:
            self.get_logger().warn(
                f"Invalid target color ignored: '{color}' (valid: {sorted(VALID_TARGET_COLORS)})"
            )
            return
        self.target_color = color
        self.get_logger().info(f"Target color set: {self.target_color}")

    def on_telemetry(self, msg: String) -> None:
        payload = loads(msg.data, None)
        if isinstance(payload, dict) and {"lat", "lon", "heading_deg"} <= payload.keys():
            self.telemetry = payload
            self.last_telemetry_time = self.now_sec()

    def on_buoys(self, msg: String) -> None:
        payload = loads(msg.data, None)
        if not isinstance(payload, dict):
            return
        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            return
        self.buoys = detections
        # Fusion/camera canary bayat bir payload yayınlayabilir. Bayat mesaj son
        # güvenilir tespitleri temizler ama perception freshness'i yenilemez.
        if not bool(payload.get("stale", False)):
            self.last_buoy_time = self.now_sec()
        # Saf replay/test harness'leri callback'i __init__ olmadan da çağırabilir;
        # production'da alan init edilir, burada eksikse güvenli 0'dan başlat.
        self._buoy_frame_seq = getattr(self, "_buoy_frame_seq", 0) + 1

    def on_obstacles(self, msg: String) -> None:
        payload = loads(msg.data, {})
        obstacles = payload.get("obstacles", [])
        self.obstacles = obstacles if isinstance(obstacles, list) else []
        # "stale: true" (sllidar_bridge canary bayat veri tekrarı): algı kaynağı
        # susmuş demektir — last_obstacle_time GÜNCELLENMEZ, böylece
        # obstacle_timeout failsafe'i tetiklenir (bayat engelle araç sürülmez).
        if not bool(payload.get("stale", False)):
            self.last_obstacle_time = self.now_sec()

    def on_gate_truth(self, msg: String) -> None:
        """Sim-only stable pair truth; canonical algı topic'lerine karışmaz."""

        if not self.sim_gate_truth_enabled:
            return
        payload = loads(msg.data, None)
        if not isinstance(payload, dict) or bool(payload.get("stale", False)):
            self.sim_gate_truth = []
            return
        detections = payload.get("detections")
        if not isinstance(detections, list) or not all(
            isinstance(item, dict) for item in detections
        ):
            self.sim_gate_truth = []
            return
        self.sim_gate_truth = detections
        self.last_gate_truth_time = self.now_sec()

    def _pair_detection_input(self, now: float) -> List[Dict[str, Any]]:
        if not self.sim_gate_truth_enabled:
            return self.buoys
        if now - self.last_gate_truth_time > self.perception_timeout:
            return []
        if self.state == "PARKUR_1_NAV":
            prefix = "p1_o_"
        elif self.state == "PARKUR_2_AVOIDANCE":
            prefix = "p2_o_"
        else:
            return []
        return [
            item for item in self.sim_gate_truth
            if item.get("source") == "sim_gate_truth"
            and str(item.get("id", "")).lower().startswith(prefix)
        ]

    # --- Durum makinesi -------------------------------------------------------

    def tick(self) -> None:
        now = self.now_sec()
        command = Command(0.0, 0.0, 0.0, "idle")
        debug: Dict[str, Any] = {}

        if self.state in {"WAIT_MISSION", "MISSION_READY"}:
            if self.started and self.waypoints:
                if self.state == "MISSION_READY" and self.telemetry is not None:
                    # Hakem sırası authoritative'dir. Runtime NN reordering,
                    # parkur sınırı/geometri/OOC rotasını ayıramaz. Tek istisna
                    # açıkça etkinleştirilmiş bench P3-only oturumudur: WP0'a
                    # sıfırlama, on_waypoints'te hesaplanan bench atlamasını
                    # korur (motor yolu bench launch'ında kapalıdır).
                    if not getattr(self, "bench_p3_only_enabled", False):
                        self.current_wp = 0
                    elif self.bench_override_wp is not None:
                        self.current_wp = self.bench_override_wp
                self.state = self.parkur_state_for_current_wp()
                if self.state == "PARKUR_3_TARGET_LOCK":
                    self._p3_initial_hold_started = now
                    self._p3_search_anchor = None
                    self._p3_search_scout = None
                    self._p3_search_reposition_bearing_deg = None
                    self._p3_search_phase = "anchor_scan"
                    self._p3_scan_last_heading_deg = None
                    self._p3_scan_accum_deg = 0.0
                    self._initialize_p3_search_anchor()
                    self._p3_reacquisition_started = False
                # İlk costmap tick'i daha sonraki tick'tedir; latch burada
                # kurulunca o ilk hesap bile turuncuyu sert engel kabul eder.
                self.strict_orange_latched = update_strict_orange_latch(
                    self.strict_orange_latched, self.state
                )
                self.parkur_start_time = now
            self.publish(command, debug)
            return

        # Aktif durumlarda telemetri timeout -> FAILSAFE.
        if self.state not in {"COMPLETE", "FAILSAFE"}:
            if not self.telemetry_ok(now):
                self.enter_failsafe("telemetry_timeout", now)
                command = Command(0.0, 0.0, 0.0, "failsafe telemetry_timeout")
                self.publish(command, debug)
                return

        # FAILSAFE: otomatik iyileşme kontrolü (telemetri/algı geri geldiyse).
        if self.state == "FAILSAFE":
            if (
                self.failsafe_recovery_enabled
                and self.failsafe_reason not in MISSION_INCOMPLETE_FAILSAFE_REASONS
                and now - self.failsafe_entered_time > self.failsafe_recovery_s
                and self.telemetry_ok(now)
            ):
                if self.failsafe_reason in {"target_color_missing", "target_perception_timeout"}:
                    if self.buoys_ok(now):
                        self.recover_from_failsafe(now)
                elif self.failsafe_reason == "obstacle_timeout":
                    if self.obstacles_ok(now):
                        self.recover_from_failsafe(now)
                else:  # telemetry_timeout ve diğerleri
                    self.recover_from_failsafe(now)
            if self.state == "FAILSAFE":
                command = Command(0.0, 0.0, 0.0, f"failsafe {self.failsafe_reason}")
                self.publish(command, debug)
                return

        if self.state == "PARKUR_1_NAV":
            self.p1_stats = self.pair_detector_p1.update(
                self._pair_detection_input(now), now
            )
            p1_stats = self.p1_stats
            self.p1_stats_cache = {
                "crossed_count": p1_stats.crossed_count,
                "kd_estimate": p1_stats.kd_estimate,
                "ratio": p1_stats.ratio,
            }
            self.yellow_count = self.yellow_counter.update(self.buoys, now)
            # Puan ceza modülü (P1): temas + parkur dışı izleme; puan dökümü.
            self.contact_p1.update(self.buoys, self.obstacles, 1, now)
            if self.telemetry is not None:
                self.ooc_p1.update(
                    float(self.telemetry.get("lat", 0.0)),
                    float(self.telemetry.get("lon", 0.0)),
                    float(self.telemetry.get("ground_speed", 0.0)),
                    1,
                    now,
                )
            self.score_reports[1] = self.score_calc.report(
                parkur=1,
                g1=p1_stats.crossed_count,
                contact1=self.contact_p1.total_counted(),
                ooc1=self.ooc_p1.times_out,
            )
            p1_transition = p1_transition_decision(
                yellow_count=self.yellow_count,
                yellow_threshold=self.yellow_threshold,
                route_exit_ready=p1_route_exit_ready(
                    self.waypoints,
                    self.current_wp,
                    self.distance_to_current_wp(),
                    self.waypoint_threshold_m,
                ),
                all_waypoints_complete=self.current_wp >= len(self.waypoints),
                elapsed_s=now - self.parkur_start_time,
                max_duration_s=self.p1_max_duration_s,
            )
            if p1_transition == "complete":
                self.transition_to_p2(now)
            elif p1_transition == "timeout":
                self.enter_failsafe("parkur1_incomplete_timeout", now)
                command = Command(0.0, 0.0, 0.0, "failsafe parkur1_incomplete_timeout")
            if self.state == "PARKUR_1_NAV":
                # P1 de lidar tabanlı DWA kullanır. Fusion/lidar akışı kesilmişken
                # boş costmap'i "yol açık" sayıp ilerlemek güvenli değildir.
                if not self.obstacles_ok(now):
                    self.enter_failsafe("obstacle_timeout", now)
                    command = Command(0.0, 0.0, 0.0, "failsafe obstacle_timeout")
                    self.publish(command, debug)
                    return
                if self.dwa_enabled:
                    command, debug = self.handle_costmap_navigation()
                else:
                    # Yedek zincir (saha testi A/B): corridor bias + obstacle avoidance.
                    command, debug = self.handle_waypoint_navigation(use_corridor=True)
            else:
                # Az önce P2'ye geçtiysek komut idle kalsın.
                self.publish(command, debug)
                return

        elif self.state == "PARKUR_2_AVOIDANCE":
            self.p2_stats = self.pair_detector_p2.update(
                self._pair_detection_input(now), now
            )
            p2_stats = self.p2_stats
            self.p2_stats_cache = {
                "crossed_count": p2_stats.crossed_count,
                "kd_estimate": p2_stats.kd_estimate,
                "ratio": p2_stats.ratio,
            }
            # Puan ceza modülü (P2): temas (turuncu kenar + sarı engel) + parkur
            # dışı izleme; puan dökümü.
            self.contact_p2.update(self.buoys, self.obstacles, 2, now)
            if self.telemetry is not None:
                self.ooc_p2.update(
                    float(self.telemetry.get("lat", 0.0)),
                    float(self.telemetry.get("lon", 0.0)),
                    float(self.telemetry.get("ground_speed", 0.0)),
                    2,
                    now,
                )
            self.score_reports[2] = self.score_calc.report(
                parkur=2,
                g2=p2_stats.crossed_count,
                contact2=self.contact_p2.total_counted(),
                ooc2=self.ooc_p2.times_out,
            )
            if not self.obstacles_ok(now):
                self.enter_failsafe("obstacle_timeout", now)
                command = Command(0.0, 0.0, 0.0, "failsafe obstacle_timeout")
                self.publish(command, debug)
                return
            # P2 rota bitişi bir kez latch'lendikten sonra mission listesine
            # nokta eklemeden 5 m'lik iç geçiş ayağını tamamla. Bu alt faz
            # P2 DWA karar zincirini yeniden çalıştırmaz; yalnız tek rota
            # komutu ve ortak son lidar/fiziksel güvenlik zarfından geçer.
            if self._p2_exit_advance_origin is not None:
                command, debug = self.handle_p2_exit_advance(now)
                self.publish(command, debug)
                return
            if self.dwa_enabled:
                command, debug = self.handle_costmap_navigation()
            else:
                # Yedek zincir (saha testi A/B): corridor bias + obstacle avoidance.
                command, debug = self.handle_waypoint_navigation(use_corridor=True)
                command = apply_obstacle_avoidance(
                    command, self.obstacles, self.obstacle_avoid_distance, self.max_yaw_rate
                )
            # Geçiş: etiketli son P2 waypoint'i authoritative'dir. Kamera
            # kapı sayısı yalnız skor/teşhis bilgisidir; P3'ü bloke etmez.
            at_last_p2_wp = p2_route_exit_ready(
                self.waypoints,
                self.current_wp,
                self.distance_to_current_wp(),
                self.waypoint_threshold_m,
            )
            p2_transition = p2_transition_decision(
                crossed_count=p2_stats.crossed_count,
                min_crossings=self.p2_min_pair_crossings,
                route_complete=at_last_p2_wp,
                elapsed_s=now - self.parkur_start_time,
                max_duration_s=self.p2_max_duration_s,
            )
            if p2_transition == "complete":
                started = self.begin_p2_exit_advance(now, p2_completed=True)
                if started and self.state == "PARKUR_2_AVOIDANCE":
                    command = Command(0.0, 0.0, 0.0, "p2_exit_advance_start")
                    debug = {
                        "p2_exit_advance": {
                            "active": True,
                            "progress_m": 0.0,
                            "remaining_m": self.p2_exit_advance_m,
                            "target_m": self.p2_exit_advance_m,
                            "route_bearing_deg": self._p2_exit_advance_bearing_deg,
                        }
                    }
            elif p2_transition == "timeout":
                self.enter_failsafe("parkur2_incomplete_timeout", now)
                command = Command(0.0, 0.0, 0.0, "failsafe parkur2_incomplete_timeout")
            if self.state != "PARKUR_2_AVOIDANCE":
                self.publish(command, debug)
                return

        elif self.state == "PARKUR_3_TARGET_LOCK":
            if getattr(self, "_p3_initial_hold_started", None) is None:
                # Saf replay/bench harness'i doğrudan P3 durumundan başlatırsa
                # da hold yalnız bu ilk tick'e göre bir kez zamanlanır.
                self._p3_initial_hold_started = now
            initial_hold_active = p3_initial_hold_active(
                now,
                self._p3_initial_hold_started,
                self.p3_search_initial_hold_s,
            )
            if not self.target_color:
                if self.p3_hold_started is None:
                    self.p3_hold_started = now
                if task_timeout_expired(
                    now - self.p3_hold_started, self.p3_hold_timeout_s
                ):
                    self.enter_failsafe("target_color_missing", now)
                    command = Command(0.0, 0.0, 0.0, "failsafe target_color_missing")
                    self.publish(command, debug)
                    return
                command = Command(0.0, 0.0, 0.0, "hold target_color_missing")
            elif not self.buoys_ok(now):
                self.enter_failsafe("target_perception_timeout", now)
                command = Command(0.0, 0.0, 0.0, "failsafe target_perception_timeout")
                self.publish(command, debug)
                return
            else:
                self.p3_hold_started = None
                selected_targets = self._p3_visible_target_candidates()
                target_visible = bool(selected_targets)
                safety_pause_was_active = bool(
                    getattr(self, "_p3_safety_pause_active", False)
                )
                if target_visible:
                    self._p3_target_lost_since = None
                    self._p3_reacquisition_started = False
                    if self._p3_target_seen_since is None:
                        self._p3_target_seen_since = now
                    if self._p3_target_last_buoy_seq != self._buoy_frame_seq:
                        self._p3_target_seen_frames += 1
                        self._p3_target_last_buoy_seq = self._buoy_frame_seq
                else:
                    if safety_pause_was_active:
                        # Near-field pivotu hedefi kameradan çıkardıysa bu model
                        # kaybı değildir. Loss grace'i başlatma; son doğrulanmış
                        # dünya hedefi güvenlik kalkınca yeniden kullanılacak.
                        self._p3_target_lost_since = None
                    else:
                        if self._p3_target_lost_since is None:
                            self._p3_target_lost_since = now
                        if self._p3_target_memory_expired(now):
                            had_tracked_target = bool(
                                self._p3_target_seen_since is not None
                                or self._p3_last_tracking_command is not None
                                or self._p3_last_target_fix is not None
                            )
                            self._p3_target_seen_since = None
                            self._p3_target_seen_frames = 0
                            self._p3_target_last_buoy_seq = -1
                            self._p3_locked_target_id = None
                            self._p3_target_confirmed_latched = False
                            self._p3_last_tracking_command = None
                            self._p3_last_target_distance_m = None
                            self._p3_last_target_fix = None
                            # Hedef grace sonrası kaybolduysa yeni bir rastgele
                            # arama noktasından devam etme: kaydedilmiş P3 ankrajına
                            # dön ve ölçülen heading ile aramayı yeniden başlat.
                            if had_tracked_target:
                                self._set_p3_return_to_anchor()
                            self._p3_reacquisition_started = True
                target_confirmed = bool(
                    self._p3_target_seen_since is not None
                    and now - self._p3_target_seen_since >= self.p3_lock_confirm_s
                    and self._p3_target_seen_frames >= self.p3_lock_confirm_frames
                )
                if target_confirmed:
                    self._p3_target_confirmed_latched = True
                risk = wrong_target_risk(
                    self.buoys,
                    self.target_color,
                    risk_radius_m=2.5,
                    min_confidence=self.target_min_confidence,
                )
                self.ts3_risk = 1 if risk is not None else 0
                # Puan ceza modülü (P3): TS3 anlık riski (0/1) + İHA bonusu.
                # "TS3" şartnameye göre farklı hedeflere TEMAS sayısıdır; kamera
                # yalnız "hedefe angaje olmadan önce farklı hedef teması" riskini
                # görebilir. Bu anlık risk (0/1) puan hesabında kullanılır;
                # kesin TS3 hakem kamerasından gelir (estimated=true).
                self.score_reports[3] = self.score_calc.report(
                    parkur=3,
                    ts3=self.ts3_risk,
                    uav_bonus=bool(self.ts3_risk == 0),
                )
                if (
                    not target_visible
                    and safety_pause_was_active
                ):
                    # Güvenlik pivotu sırasında kamera hedefi kaybetse bile
                    # search/anchor otoritesine geçme. Son doğrulanmış dünya
                    # hedefini koru; başka engel kalktığı tick'te aynı yaklaşma
                    # yayı kaldığı yerden devam eder.
                    if self._p3_last_target_fix is not None:
                        intent = self._p3_virtual_target_command()
                    else:
                        intent = Command(0.0, 0.0, 0.0, "p3_safety_pause_hold")
                    command, debug, _ = self._p3_guard_tracking_command(
                        intent, debug, selected_targets, now
                    )
                elif not target_visible and (
                    self._p3_target_lost_since is not None
                    and not self._p3_target_memory_expired(now)
                    and self._p3_last_target_fix is not None
                ):
                    # Hedef geçici olarak kameradan çıktığında son lidar-fused
                    # dünya koordinatı sanal waypoint olarak korunur. 3.5 s'lik
                    # grace komut sürekliliğini; uzun memory timeout ise ancak
                    # gerçekten bulunamayan hedefte ankraja dönüşü belirler.
                    # Komut her tick yeniden hesaplanır ve lidar veto yetkisi
                    # aynen korunur; ham motor komutu tekrarlanmaz.
                    command = self._p3_virtual_target_command()
                    # Geçici hedef yalnız yön hafızasıdır. Doğrulanmış temas
                    # hedefi filtrelenir; diğer lidar engelleri daha yüksek
                    # otoriteyle yaklaşmayı duraklatabilir.
                    command, debug, _ = self._p3_guard_tracking_command(
                        command, debug, selected_targets, now
                    )
                else:
                    tracking_command = target_engagement_command(
                        self.target_color,
                        self._p3_tracking_buoys(selected_targets),
                        self.max_speed,
                        self.max_yaw_rate,
                        self.target_min_confidence,
                        wrong_avoid_lateral_m=self.wrong_avoid_lateral_m,
                        wrong_avoid_speed_mps=self.wrong_avoid_speed_mps,
                        search_yaw_rate_deg_s=self.p3_search_yaw_rate_deg_s,
                        lock_yaw_gain_deg_s=self.p3_lock_yaw_gain_deg_s,
                        engage_center_max=self.p3_engage_center_max,
                        engage_distance_m=self.p3_engage_distance_m,
                        lock_min_speed_mps=self.p3_lock_min_speed_mps,
                        engage_speed_mps=self.p3_engage_speed_mps,
                        engage_allowed=target_confirmed,
                        align_distance_m=self.p3_align_distance_m,
                        approach_speed_mps=self.p3_approach_speed_mps,
                        approach_yaw_max_deg_s=self.p3_approach_yaw_max_deg_s,
                        search_advance_after_rotations=0,
                    )
                    if target_visible:
                        # Hedef niyetini safety override'dan ÖNCE sakla. Aksi
                        # halde sanal hedef hafızasına near_field pivot komutu
                        # yazılır ve engel kalkınca yaklaşma devam edemez.
                        if selected_targets:
                            self._remember_p3_virtual_target(selected_targets[0])
                            try:
                                self._p3_last_target_distance_m = float(
                                    selected_targets[0].get("distance", 99.0)
                                )
                            except (TypeError, ValueError, OverflowError):
                                self._p3_last_target_distance_m = None
                        self._p3_last_tracking_command = tracking_command
                        command, debug, _ = self._p3_guard_tracking_command(
                            tracking_command, debug, selected_targets, now
                        )
                    elif str(tracking_command.action).startswith("avoid_wrong_target"):
                        # Yanlış renkli sert engel arama hareketinden önceliklidir,
                        # fakat P3'te geri-ileri süpürme başlatamaz.
                        command, debug = self._near_field_guarded(
                            tracking_command, debug, allow_reverse_escape=False
                        )
                    else:
                        command, debug = self._p3_anchor_search_command(debug)
                # Parkur girişine bağlı tek seferlik bekleme, hedef görünürlük
                # dalından bağımsız olarak tam olarak bir kez uygulanır.
                if initial_hold_active:
                    command = Command(0.0, 0.0, 0.0, "p3_initial_hold")
                if "engage" in command.action:
                    self.state = "ENGAGE"
                    self.engage_started = now

        elif self.state == "ENGAGE":
            if not self.buoys_ok(now):
                self.enter_failsafe("target_perception_timeout", now)
                command = Command(0.0, 0.0, 0.0, "failsafe target_perception_timeout")
            else:
                selected_targets = self._p3_visible_target_candidates()
                if not selected_targets:
                    can_commit_virtual_contact = bool(
                        self._p3_target_confirmed_latched
                        and self._p3_last_target_fix is not None
                        and self.engage_started is not None
                    )
                    if can_commit_virtual_contact:
                        # ENGAGE'e girmiş lidar-fused hedef kısa süre kameradan
                        # çıksa da ayrı search/anchor otoritesine geçme. Sabit
                        # dünya hedefiyle temas yayını sürdür; başka engel varsa
                        # aynı P3 safety pause zamanlayıcıyı dondurur.
                        if (
                            not getattr(self, "_p3_safety_pause_active", False)
                            and self._p3_target_lost_since is None
                        ):
                            self._p3_target_lost_since = now
                        intent = self._p3_virtual_target_command()
                        command, debug, safety_paused = self._p3_guard_tracking_command(
                            intent, debug, selected_targets, now
                        )
                        if safety_paused:
                            pass
                        elif now - self.engage_started >= self.engage_window_s:
                            self.state = "COMPLETE"
                            command = Command(0.0, 0.0, 0.0, "mission_complete")
                        elif (
                            self._p3_target_lost_since is not None
                            and now - self._p3_target_lost_since
                            > self.p3_target_loss_grace_s
                        ):
                            # Sanal temas penceresi sonuç üretmediyse normal
                            # P3 reacquisition'a dön; sonsuz kör itki yok.
                            self.state = "PARKUR_3_TARGET_LOCK"
                            self.engage_started = None
                            command = Command(
                                0.0, 0.0, 0.0, "engage_virtual_contact_timeout"
                            )
                    elif getattr(self, "_p3_safety_pause_active", False):
                        # Hedef fix'i yoksa ileri itki üretme; yalnız güvenlik
                        # latch'inin kontrollü biçimde çözülmesini bekle.
                        intent = Command(0.0, 0.0, 0.0, "p3_safety_pause_hold")
                        command, debug, _ = self._p3_guard_tracking_command(
                            intent, debug, selected_targets, now
                        )
                    else:
                        # Kör temas yok: gerçek hedef kaybında ileri itki
                        # sıfırlanır ve aynı track'i yeniden edinmeye dönülür.
                        self.state = "PARKUR_3_TARGET_LOCK"
                        self.engage_started = None
                        self._p3_target_lost_since = now
                        command = Command(0.0, 0.0, 0.0, "engage_target_lost_stop")
                else:
                    self._p3_target_lost_since = None
                    self._p3_target_confirmed_latched = True
                    tracked = target_engagement_command(
                        self.target_color,
                        self._p3_tracking_buoys(selected_targets),
                        self.max_speed,
                        self.max_yaw_rate,
                        self.target_min_confidence,
                        wrong_avoid_lateral_m=self.wrong_avoid_lateral_m,
                        wrong_avoid_speed_mps=self.wrong_avoid_speed_mps,
                        search_yaw_rate_deg_s=self.p3_search_yaw_rate_deg_s,
                        lock_yaw_gain_deg_s=self.p3_lock_yaw_gain_deg_s,
                        engage_center_max=self.p3_engage_center_max,
                        engage_distance_m=self.p3_engage_distance_m,
                        lock_min_speed_mps=self.p3_lock_min_speed_mps,
                        engage_speed_mps=self.p3_engage_speed_mps,
                        engage_allowed=True,
                        align_distance_m=getattr(
                            self, "p3_align_distance_m", 5.0
                        ),
                        approach_speed_mps=getattr(
                            self, "p3_approach_speed_mps", 0.40
                        ),
                        approach_yaw_max_deg_s=getattr(
                            self, "p3_approach_yaw_max_deg_s", 12.0
                        ),
                    )
                    guarded, debug, safety_paused = self._p3_guard_tracking_command(
                        tracked, debug, selected_targets, now
                    )
                    if safety_paused:
                        # Başka bir sert engel hareketi geçici olarak devralır;
                        # ENGAGE state'i ve temas süresi korunur.
                        command = guarded
                    elif str(tracked.action).startswith(
                        ("target_align", "target_lock", "target_approach")
                    ):
                        # Engage geometrisi dar bir eşiktir; tek karelik merkez
                        # sapması state'i düşürüp vx=0 darbesi üretmesin. Homing
                        # yayı sürer, fakat temas penceresi ancak engage
                        # geometrisi kesintisiz geri kazanıldığında sayılır.
                        self.engage_started = now
                        command = Command(
                            guarded.vx,
                            guarded.vy,
                            guarded.yaw_rate,
                            f"engage_homing {tracked.action}",
                        )
                    elif "engage" not in tracked.action:
                        # Yanlış hedef kaçışı gibi ayrı bir P3 niyeti ortaya
                        # çıktıysa lock state'ine yumuşak geç; sıfır-komut darbesi
                        # yerine hesaplanan güvenli komutu koru.
                        self.state = "PARKUR_3_TARGET_LOCK"
                        self.engage_started = None
                        command = guarded
                    elif (
                        self.engage_started is not None
                        and now - self.engage_started < self.engage_window_s
                    ):
                        command = Command(
                            tracked.vx, 0.0, tracked.yaw_rate,
                            "engage_contact_window",
                        )
                    else:
                        self.state = "COMPLETE"
                        command = Command(0.0, 0.0, 0.0, "mission_complete")

        elif self.state in {"COMPLETE", "FAILSAFE"}:
            command = Command(0.0, 0.0, 0.0, self.state.lower())

        self.publish(command, debug)

    def recover_from_failsafe(self, now: float) -> None:
        """FAILSAFE'ten ilgili parkur state'ine otomatik dön."""
        if self.failsafe_reason in MISSION_INCOMPLETE_FAILSAFE_REASONS:
            self.get_logger().warn(
                f"Non-recoverable mission phase failure remains HOLD: {self.failsafe_reason}"
            )
            return
        if self.failsafe_reason in {"target_color_missing", "target_perception_timeout"}:
            # B12: hedef renk hiç gelmediyse 3 retry sonrası COMPLETE — aksi halde
            # 60s hold + 5s recovery sonsuz döngüsü 20dk yarışma süresini tüketir.
            if self.failsafe_reason == "target_color_missing" and not self.target_color:
                self.p3_target_retries += 1
                if self.p3_target_retries >= self.p3_target_max_retries:
                    self.get_logger().warn(
                        f"target_color_missing {self.p3_target_retries} retry aşıldı; "
                        "COMPLETE'e geçiliyor (hedef rengi hiç gelmedi)"
                    )
                    self.state = "COMPLETE"
                    return
            self.state = "PARKUR_3_TARGET_LOCK"
        elif self.failsafe_reason == "obstacle_timeout":
            self.state = "PARKUR_2_AVOIDANCE"
        else:
            self.state = self._failsafe_resume_state or self.parkur_state_for_current_wp()
        self.failsafe_reason = ""
        self._failsafe_resume_state = None
        self.parkur_start_time = now
        self.get_logger().info("Recovered from FAILSAFE")

    def distance_to_current_wp(self) -> float:
        """Aktif waypoint'e olan mesafe; waypoint yoksa sonsuz."""
        if self.telemetry is None or self.current_wp >= len(self.waypoints):
            return float("inf")
        wp = self.waypoints[self.current_wp]
        return haversine_m(self.telemetry["lat"], self.telemetry["lon"], wp["lat"], wp["lon"])

    def _build_corridor(self) -> Dict[str, Any]:
        """Turuncu dubalardan aktif koridor bilgisi üretir (DWA corridor bias).

        Mantık planner.build_corridor_info'da (saf) — bu metod yalnız delegasyon;
        simülasyon testleri gerçek kodu kullanır (kopya yok). Yetersiz görüşte
        ``{"active": False}`` döner — DWA bias terimi etkisiz kalır.
        """
        return build_corridor_info(self.buoys)

    def _build_p2_route_corridor(self) -> Dict[str, Any]:
        """P2 mission polyline'ını kamera-bağımsız DWA zarfına dönüştür."""

        if (
            self.state != "PARKUR_2_AVOIDANCE"
            or self.telemetry is None
            or not getattr(self, "p2_route_corridor_enabled", True)
        ):
            return {"active": False, "source": "mission_route_disabled"}
        _, p2_route = split_parkur_routes(self.raw_waypoints or self.waypoints)
        return build_route_corridor_info(
            self.telemetry,
            p2_route,
            lookahead_m=getattr(self, "p2_route_corridor_lookahead_m", 8.0),
            hard_half_width_m=getattr(
                self, "p2_route_corridor_half_width_m", 4.0
            ),
            confidence=getattr(self, "p2_route_corridor_confidence", 1.0),
        )

    def _update_stuck(
        self, now: float, ground_speed: float, *, motion_expected: bool
    ) -> None:
        """T3: zaman tabanlı stuck sayacı (anlık vx koşulu kaldırıldı).

        ground_speed integrali: her tick'te ölçülen hız x dt biriktirilir.
        - ground_speed >= stuck_vx_threshold ise araç ilerliyor demektir ->
          sayaç sıfırlanır (takılı değil).
        - sürekli düşük hız ve < stuck_min_dist_m yol alınmadıysa, stuck_timeout_s
          dolunca stuck bayrağı döner (autonomy_node recovery'yi tetikler).

        Bu, yaklaşma freni / sarı fren ile doğal yavaşlayan aracı (2 tick vx<0.1)
        takılma saymaz — yalnızca gerçek sürekli hareketsizliği yakalar.
        """
        if self.telemetry is None:
            return
        # Yerinde hizalama, near-field stop/pivot ve final güvenlik stop'u
        # kasıtlı hareketsizliktir. Bunlar stuck süresine yazılmaz; aksi halde
        # komut hakemi ALIGN/STOP'tan çıkarken sahte recovery tetiklenir.
        if not motion_expected:
            self._stuck_since = None
            self._stuck_dist_m = 0.0
            return
        if ground_speed < self.stuck_vx_threshold:
            if self._stuck_since is None:
                self._stuck_since = now
            self._stuck_dist_m = 0.0  # düşük hızda mesafe birikmez (ölü bant)
        else:
            # İlerleme var: ölçülen hız ile mesafe biriktir, sayacı güncelle.
            self._stuck_since = None
            # (integral dt'ye bağlı; burada sadece "ilerliyor" bayrağı yeterli —
            #  gerçek mesafe eşiği düşük hızda zaten birikmez)

    def _is_stuck(self, now: float) -> bool:
        """Sürekli düşük hız >= stuck_timeout_s -> stuck."""
        if self._stuck_since is None:
            return False
        return (now - self._stuck_since) >= self.stuck_timeout_s

    def _current_parkur_index(self) -> Optional[int]:
        """Aktif parkuru state'ten tek noktada çözer."""
        if self.state == "PARKUR_1_NAV":
            return 1
        if self.state == "PARKUR_2_AVOIDANCE":
            return 2
        if self.state in {"PARKUR_3_TARGET_LOCK", "ENGAGE"}:
            return 3
        return None

    def _parkur_idx_in(self, lst: List[int]) -> bool:
        """Parkur bazlı davranışı yalnız konfigürasyon listesi belirler."""
        parkur = self._current_parkur_index()
        return parkur is not None and parkur in lst

    def _use_corridor_goal(self) -> bool:
        """Koridor-hedef override'ı yalnız P1'de (corridor_goal_parkurs) aktif."""
        return self._parkur_idx_in(getattr(self, "corridor_goal_parkurs", [0]))

    def _corridor_bias_configured(self) -> bool:
        """DWA koridor bias'ının parkur bazlı konfigürasyonu."""

        return self._parkur_idx_in(getattr(self, "corridor_bias_parkurs", [1]))

    def _dwa_heading_align_configured(self) -> bool:
        """Yerinde waypoint hizalaması yalnız merkezi profil parkurlarında."""

        return self._parkur_idx_in(getattr(self, "dwa_align_parkurs", [1]))

    def _use_corridor_bias(self, corridor: Optional[Dict[str, Any]] = None) -> bool:
        """Bias yalnız konfigüreyse ve algı koridoru gerçekten aktifse açılır.

        P2'de False -> dwa.plan(corridor=None) -> corridor_score=0 (çifte
        uygulama kaldırılır; sarı engel üstüne hedef düşmez).
        """
        return self._corridor_bias_configured() and bool(
            corridor and corridor.get("active")
        )

    def _corridor_debug(self, corridor: Dict[str, Any]) -> Dict[str, Any]:
        """Normal ve recovery return yolları için tek bias debug kaynağı."""

        return {
            "corridor": corridor,
            "corridor_bias_configured": self._corridor_bias_configured(),
            "corridor_bias_active": self._use_corridor_bias(corridor),
        }

    def handle_waypoint_navigation(self, use_corridor: bool) -> tuple[Command, Dict[str, Any]]:
        assert self.telemetry is not None
        if self.current_wp >= len(self.waypoints):
            if self.state == "PARKUR_1_NAV":
                # P1 waypoint'leri bitti -> P2 geçişi (p1 tamamlama koşulu).
                self.transition_to_p2(self.now_sec())
                return Command(0.0, 0.0, 0.0, "p1_waypoints_complete"), {"reached": True}
            if self.state == "PARKUR_2_AVOIDANCE":
                # P2 parkuru bitti ama geçiş şartı (ikili geçiş sayısı) tick'te
                # kontrol edilir; burada sadece bekle.
                return Command(0.0, 0.0, 0.0, "hold parkur2_wait_crossings"), {"reached": True}
            self.state = "PARKUR_3_TARGET_LOCK"
            return Command(0.0, 0.0, 0.0, "parkur3_start"), {}

        wp = self.waypoints[self.current_wp]
        distance = haversine_m(self.telemetry["lat"], self.telemetry["lon"], wp["lat"], wp["lon"])
        # Çifte geçiş önleme: DWA etkinken (P1/P2) waypoint geçişi yalnız
        # handle_costmap_navigation'da yapılır — burada geçilmez (yoksa aynı tick'te
        # iki kez current_wp += 1 olur, salınımı besler).
        if distance <= self.waypoint_threshold_m and not self.dwa_enabled:
            self.current_wp += 1
            if self.current_wp >= len(self.waypoints):
                # Tüm waypoint'ler bitti. Geçişler tamamlama koşullarına göre:
                # P1'de yol bitti -> P2'ye ilerle; P2'de gate kanıtını tick
                # doğrulamadan P3'e geçilmez.
                if self.state == "PARKUR_1_NAV":
                    self.transition_to_p2(self.now_sec())
                    return Command(0.0, 0.0, 0.0, "p1_waypoints_complete"), {"reached": True}
                if self.state == "PARKUR_2_AVOIDANCE":
                    return Command(0.0, 0.0, 0.0, "hold parkur2_wait_crossings"), {"reached": True}
                self.state = "PARKUR_3_TARGET_LOCK"
                return Command(0.0, 0.0, 0.0, "waypoints_complete"), {"reached": True}
            # NOT: waypoint parkur alanı SADECE başlangıç state seçimi içindir;
            # parkur geçişleri tamamlama şartlarına göre tick() içinde yapılır.
            wp = self.waypoints[self.current_wp]

        command = waypoint_command(self.telemetry, wp, self.max_speed, self.max_yaw_rate, self.yaw_kp)
        if use_corridor:
            command = apply_corridor_bias(command, self.buoys, self.max_yaw_rate)
        return command, {"waypoint_index": self.current_wp, "distance_m": distance, "parkur": wp.get("parkur", 1)}

    def handle_costmap_navigation(self) -> tuple[Command, Dict[str, Any]]:
        """P1/P2 DWA planlaması: costmap.update() -> /planning/costmap -> dwa.plan().

        Lidar engelleri + kamera renkleri (turuncu koridor = maliyet 1, sarı
        engel = maliyet 8, unknown = yavaş/dar) tek maliyet haritasında birleşir;
        DWA bu maliyeti aday skorlamasında kullanır. Turuncu koridor bilgisi
        ``_build_corridor()`` ile ayrıca ``corridor`` bias terimi olarak da
        ``dwa.plan(corridor=...)``'a gider (heading katmanı). Hedef: hedef
        waypoint'in gövde-frame konumu. Tüm adaylar çarpışırsa ``recovery()``
        (hedef yönelimli dönüş). unknown hücreler yakınsa ``slow_vx`` ile yavaşla.
        """
        assert self.telemetry is not None
        # Hız governor'ı (idaws:302-331): ölçülen hız sensör menzilinin izin
        # verdiği v_max_sensor'ü aşıyorsa DWA hız ölçeğini düşür, altındaysa
        # artır. Ölü bant 0.25 m/s, integral gain 0.03, clamp [0.05, 1.0].
        speed = max(0.0, float(self.telemetry.get("ground_speed", 0.0) or 0.0))
        if speed > self._v_max_sensor + 0.25:
            self._speed_scale_target = max(0.05, self._speed_scale_target - 0.03)
        elif speed < self._v_max_sensor - 0.25:
            self._speed_scale_target = min(1.0, self._speed_scale_target + 0.03)
        planner_speed_scale = self._speed_scale_target
        p2_nearest_forward_obstacle_m = None
        if self.state == "PARKUR_2_AVOIDANCE":
            # Sabit costmap şişirmesi koridoru açık tutar; bunun karşılığında
            # gerçek tekne ataleti, engel yaklaşınca DWA'nın hız tavanında
            # temsil edilir. Bu ayrı bir davranış/komut üretmez: yalnız mevcut
            # aday kümesini saha profilindeki slow hızına sınırlar.
            p2_nearest_forward_obstacle_m = nearest_forward_hard_obstacle_m(
                self.obstacles,
                rear_clearance_m=0.0,
                lateral_clearance_m=self.near_field_slow_lateral_m,
            )
            if (
                p2_nearest_forward_obstacle_m is not None
                and p2_nearest_forward_obstacle_m <= self.near_field_slow_m
            ):
                planner_speed_scale = min(
                    planner_speed_scale,
                    self.near_field_slow_speed_mps / max(0.1, self.max_speed),
                )
        self.dwa.set_max_speed_scale(planner_speed_scale)
        # Plan-komut eşleşmesi: P1'de costmap'i ölçülen hızla değil, komutun
        # üreteceği hızla şişir. P2'de ise DWA aday izi aynı riski zaten taradığı
        # için saha profilindeki tavan global şişirmeyi sınırlar; böylece dar
        # koridorda durma mesafesi iki kez sayılmaz.
        plan_speed = max(speed, self.last_dwa.vx if self.last_dwa else speed)
        inflation_speed = costmap_inflation_speed_for_state(
            self.state,
            plan_speed,
            self.p2_costmap_inflation_speed_cap_mps,
        )
        self.costmap.update(self.obstacles, self.buoys, self.target_color,
                            ground_speed=inflation_speed,
                            strict_corridor=self.strict_orange_latched)
        # Costmap yayını her 4. tick'te (DEBUG topic'i; state her tick yayınlanır).
        # to_json 14.400 hücre tarar (~11ms) — her tick'te yayınlanırsa tick bloklanır.
        self._costmap_pub_div = (self._costmap_pub_div + 1) % 4
        if self._costmap_pub_div == 0:
            self.publish_costmap()
        # Koridor bias: turuncu duba bilgisi DWA karar mekanizmasına girer
        # (kök neden: dwa_enabled=true iken apply_corridor_bias çağrılmıyordu).
        visual_corridor = self._build_corridor()
        route_corridor = self._build_p2_route_corridor()
        # P2'de mission doğrusu birincil koridor zarfıdır; kamera yalnız
        # costmap renklerini zenginleştirir. P1'in görsel koridor davranışı
        # byte-for-byte korunur.
        if route_corridor.get("active"):
            corridor = dict(route_corridor)
            corridor["visual_corridor_active"] = bool(
                visual_corridor.get("active")
            )
        else:
            corridor = visual_corridor
        corridor_debug = self._corridor_debug(corridor)

        # Waypoint ilerleme: normal eşik geçişi veya yalnız mevcut WP düzleminin
        # guard yarıçapı içinde kaçırılması. Sonraki WP'nin arkada görünmesi tek
        # başına geçiş değildir; bu eski koşul rosbag'de 50 m uzaktaki WP2'den
        # WP3 ve WP4'e zincirleme atlamaya neden oldu.
        advanced_this_tick = False
        wp_distance = self.distance_to_current_wp()
        if self.current_wp < len(self.waypoints) - 1:
            nxt = self.waypoints[self.current_wp + 1]
            fwd_nxt, _ = self.goal_body_forward_m(nxt)
            fwd_cur, _ = self.goal_body_forward_m(self.waypoints[self.current_wp])
            if waypoint_advance_decision(
                wp_distance,
                self.waypoint_threshold_m,
                fwd_cur,
                fwd_nxt,
                self.waypoint_overshoot_guard_m,
            ):
                previous_wp = self.current_wp
                self.current_wp += 1
                advanced_this_tick = True
                # Yeni waypoint yeni bir hedef bağlamıdır; eski hedefe ait
                # align/recovery dwell komutu bir sonraki tick'e taşınmaz.
                self.nav_arbiter.reset()
                self._stuck_since = None
                self.get_logger().info(
                    "WAYPOINT_ADVANCE: "
                    f"{previous_wp}->{self.current_wp} dist={wp_distance:.2f}m "
                    f"fwd_cur={fwd_cur:.2f}m fwd_next={fwd_nxt:.2f}m"
                )

        # Hedef waypoint -> gövde (forward, lateral) metre.
        if self.current_wp >= len(self.waypoints):
            # P1/P2'de yol bitmişse: geçiş tick() içinde yapılır; burada bekle.
            return Command(0.0, 0.0, 0.0, "hold parkur_transition"), {"costmap": True}
        # KORİDOR HEDEFİ: koridor aktifken araç waypoint'e değil, 8-10m ilerideki
        # koridor merkezine baksın (yanal hedef yumuşatılmış). Bu, salınımı keser —
        # araç mikro waypoint bearing'ine aşırı duyarlı takip yerine koridoru
        # ortalıyor. Tekil hedef kaynağı (T1): hedef kararı parkur bazlıdır —
        # P1'de koridor hedefi (corridor_goal_parkurs), P2'de her zaman waypoint.
        # Seçim mantığı planner.corridor_goal_body'ye delege edilir (tek kaynak).
        #
        # KRİTİK (debug 2026-08-13): P1'de koridor hedefi HER ZAMAN waypoint'e
        # tercih edilir — waypoint'ler koridor merkezinde DEĞİL (araç saptığında
        # waypoint 8m+ yanal görünür; waypoint hedefe gidince araç koridordan
        # daha da sapar, dubaya çarpar). Hayalet merkez (tek taraflı görüşte
        # -5.5m) corridor_goal_body'deki lateral clamp (max_lateral_m) ile
        # sınırlanır — aşırı sapma yok. conf eşiği/histerezis KALDIRILDI:
        # koridor hedefi P1'de aktif kaldıkça araç koridoru asla terk etmez.
        guidance_index = waypoint_guidance_index(
            self.current_wp,
            len(self.waypoints),
            wp_distance,
            self.waypoint_threshold_m,
            advanced_this_tick,
        )
        transition_alignment = guidance_index != self.current_wp
        wp = self.waypoints[guidance_index]
        dx_n, dy_e = self.goal_body_forward_m(wp)
        use_cg = (
            bool(corridor) and corridor.get("active")
            and self._use_corridor_goal()
            and not transition_alignment
        )
        if use_cg:
            # 8-10m ilerideki koridor merkezi hedef.
            # DUNKU CALISAN: cl*0.3 (0.5 gerçek sim'de dubalara yaklaştırdı —
            # basit 2D simülasyon overshoot'u yakalayamadı; ArduPilot fiziği
            # ile araç koridordan çıktı). Geri alindi.
            goal_body = corridor_goal_body(
                dx_n, dy_e,
                float(corridor.get("center_forward_m", 0.0)),
                float(corridor.get("center_lateral_m", 0.0)),
                float(corridor.get("confidence", 0.0)),
                use_corridor_goal=True,
                lookahead_min_m=self.corridor_goal_lookahead_min_m,
                lateral_gain=self.corridor_goal_lateral_gain,
                max_lateral_m=2.5,  # hayalet merkez koruması (tek taraflı görüş)
            )
        else:
            goal_body = (dx_n, dy_e)

        # Waypoint öncesinde mesafeye bağlı fren kaldırıldı: tekne etkisiz itki
        # bölgesine düşüp waypoint değişiminde yeniden hız sıçraması yapıyordu.
        # Köşe kontrolü, waypoint kabulünden sonra ayrı düşük hızlı heading yayı
        # ile yapılır; ileri hız artışı tek davranış hakeminde eğimlidir.
        approach_vx = self.dwa.effective_max_speed()
        slow = self.dwa.near_unknown(self.costmap)
        last = self.last_dwa
        if slow:
            # unknown yakınsa: hız limitini düşürerek planla (yavaş/dar koridor).
            # B6: kalıcı self.dwa_slow kullanılır — her tick'te DwaPlanner yeniden
            # kurulmaz (cache boş -> ~120ms/tick). Hız ölçeği, governor + approach
            # tavanının slow_vx'e oranıyla set edilir; etkin tavan slow_vx'i aşmaz.
            slow_scale = min(
                1.0,
                self.dwa.effective_max_speed() / max(0.1, self.dwa_slow_vx),
                approach_vx / max(0.1, self.dwa_slow_vx),
            )
            self.dwa_slow.set_max_speed_scale(slow_scale)
            # T1: corridor bias P2'de kapalı (çifte uygulama kaldırılır) — P2'de
            # corridor=None -> dwa.py corridor_score=0; sarı engel üstüne hedef düşmez.
            result = self.dwa_slow.plan(goal_body, self.costmap, last,
                                        corridor=corridor if self._use_corridor_bias(corridor) else None,
                                        allow_heading_align=self._dwa_heading_align_configured(),
                                        yaw_rate_limit_deg_s=(
                                            self.p2_dwa_max_yaw_rate_deg_s
                                            if self.state == "PARKUR_2_AVOIDANCE"
                                            else None
                                        ))
        else:
            # Waypoint yaklaşımı: normal DWA'ya geçici ölçek (approach_vx / nominal).
            prev_scale = self.dwa.max_speed_scale
            self.dwa.set_max_speed_scale(min(prev_scale, approach_vx / max(0.1, self.dwa.effective_max_speed())))
            # T1: corridor bias P2'de kapalı — P2'de corridor=None -> corridor_score=0.
            result = self.dwa.plan(goal_body, self.costmap, last,
                                   corridor=corridor if self._use_corridor_bias(corridor) else None,
                                   allow_heading_align=self._dwa_heading_align_configured(),
                                   yaw_rate_limit_deg_s=(
                                       self.p2_dwa_max_yaw_rate_deg_s
                                       if self.state == "PARKUR_2_AVOIDANCE"
                                       else None
                                   ))
            self.dwa.set_max_speed_scale(prev_scale)

        # Planner/align/recovery yalnız bir ADAYDIR. Near-field ve davranış
        # hakemi aşağıda tek komuta indirger; son güvenlik zarfından sonra kalan
        # komut bir sonraki DWA tick'inin gerçek ``last`` girdisi olur.
        candidate = Command(
            result.vx, 0.0, result.yaw_rate, f"{result.action} slow={slow}"
        )
        base_debug = {
            "costmap": True,
            "goal_body": [round(goal_body[0], 2), round(goal_body[1], 2)],
            "slow": slow,
            "recovery": "recovery" in result.action,
            "dwa_enabled": self.dwa_enabled,
            "dwa_heading_align_enabled": self._dwa_heading_align_configured(),
            "transition_alignment": transition_alignment,
            "gov": round(self._speed_scale_target, 2),
            "planner_speed_scale": round(planner_speed_scale, 3),
            "p2_nearest_forward_obstacle_m": (
                round(p2_nearest_forward_obstacle_m, 3)
                if p2_nearest_forward_obstacle_m is not None
                else None
            ),
            "vmax": round(self.dwa.effective_max_speed(), 2),
            "costmap_inflation_speed_mps": round(inflation_speed, 3),
            "costmap_inflation_radius_m": round(
                self.costmap.get_inflation_radius(), 3
            ),
            # corridor dict'i her zaman gösterilir; aktif olup olmadığı ayrı
            # alanlardan anlaşılır (P2'de bias kapalıyken corridor.active=true
            # görünse bile DWA'ya None gider — analizci yanılmasın).
            **corridor_debug,
            "corridor_goal_active": use_cg,
        }
        now = self.now_sec()
        command, debug, decision = self._arbitrate_navigation_candidate(
            candidate,
            base_debug,
            goal_body=goal_body,
            now=now,
        )

        # Stuck yalnız hakemin gerçekten ileri ilerleme istediği sürede sayılır.
        # ALIGN, near-field pivot ve safety stop kasıtlı duruştur.
        ground_speed = float(self.telemetry.get("ground_speed", 0.0) or 0.0)
        motion_expected = motion_expected_for_stuck(
            decision, self.stuck_vx_threshold
        )
        self._update_stuck(now, ground_speed, motion_expected=motion_expected)
        debug["stuck"] = {
            "motion_expected": motion_expected,
            "since": self._stuck_since,
        }
        if self._is_stuck(now) and self.distance_to_current_wp() > 5.0:
            self._stuck_since = None
            recovery_cmd = self.dwa.recovery(
                self.costmap, last, goal_body=goal_body
            )
            recovery_candidate = Command(
                recovery_cmd.vx,
                0.0,
                recovery_cmd.yaw_rate,
                "stuck_recovery",
            )
            recovery_debug = {
                **base_debug,
                "recovery": True,
                "stuck": True,
            }
            command, debug, _ = self._arbitrate_navigation_candidate(
                recovery_candidate,
                recovery_debug,
                goal_body=goal_body,
                now=now,
            )
        return command, debug

    def _arbitrate_navigation_candidate(
        self,
        candidate: Command,
        debug: Dict[str, Any],
        *,
        goal_body: Optional[tuple],
        now: float,
    ) -> tuple[Command, Dict[str, Any], NavigationDecision]:
        """Near-field -> tek davranış hakemi -> final güvenlik zarfı.

        P1/P2 hareket komutlarının tek çıkış noktasıdır. ``last_dwa`` da ham
        planner adayından değil, gerçekten yayınlanacak son komuttan beslenir.
        """

        guarded, debug = self._near_field_guarded(
            candidate, debug, goal_body=goal_body
        )
        # P2'de recovery kararını DWA zaten üretir ve yönünü kendi
        # histerezisiyle korur. Arbiter'in aynı recovery'yi ikinci kez dwell
        # etmesi rosbag'de 25.3 s ek bekleme ve 83 DWA<->recovery geçişi
        # üretti. P1 davranışı aynen kalır; P2'de arbiter yalnız son fiziksel
        # çarpışma zarfı ve hız eğimi olarak çalışır.
        decision = self.nav_arbiter.select(
            guarded,
            self.obstacles,
            now,
            allow_dwell=getattr(self, "state", "") != "PARKUR_2_AVOIDANCE",
        )
        debug["behavior"] = {
            "mode": decision.mode,
            "requested_mode": decision.requested_mode,
            "held": decision.held,
            "safety_reason": decision.safety_reason,
            "proposal_action": candidate.action,
            "guarded_action": guarded.action,
        }
        final = decision.command
        self.last_dwa = DwaCommand(
            final.vx, final.yaw_rate, final.action, score=0.0
        )
        return final, debug, decision

    def _near_field_guarded(
        self, candidate: Command, debug: Dict[str, Any],
        goal_body: Optional[tuple] = None,
        allow_reverse_escape: Optional[bool] = None,
        obstacles_override: Optional[List[Dict[str, Any]]] = None,
    ) -> tuple[Command, Dict[str, Any]]:
        """Navigasyon veya uzak P3 yaklaşmasına yakın-alan kuralını uygular.

        Hard obstacle yakınsa ileri komut sıfırlanır (stop/pivot) veya
        yavaşlatılır (slow). Histerezis: stop aktifken engel release eşiğine
        kadar açılmaz — tick'ten tick'e v=0/pivot ile v=0.7/arc geçişi bastırılır.
        P3'te yalnız 5 m dışındaki yaklaşma ve hedefsiz düz konum değişimi bu
        katmana girer; yakın temas zinciri bilerek girmez. Engel yoksa ve
        goal_body verildiyse waypoint heading hizalaması uygulanabilir.
        """
        # P2 tek yetkili zincir: lidar-costmap DWA -> NavigationBehaviorArbiter
        # final çarpışma zarfı. Rosbag 2026-08-19'da bu DWA'nın önündeki ikinci
        # radial stop katmanı, 111 saniyelik P2'nin 70.6 saniyesini durdurarak
        # medyan hızı 0.04 m/s'ye indirdi. DWA zaten şişirilmiş hard-obstacle
        # costmap kullanır; arbiter da yayın öncesi gövde koridoru/yüksek-yaw
        # stop'unu uygular. Burada üçüncü bir karar üretmek güvenliği artırmak
        # yerine birbirine karşı çalışan otoriteler oluşturuyordu.
        if getattr(self, "state", "") == "PARKUR_2_AVOIDANCE":
            self._near_field_latch = False
            escape = getattr(self, "center_obstacle_escape", None)
            if escape is not None:
                escape.reset()
            debug["near_field"] = {
                "active": False,
                "latch": False,
                "reason": "disabled_for_p2_dwa_authority",
                "escape": "disabled_for_p2",
            }
            return candidate, debug

        heading_error_deg = None
        # KULLANICI GÖZLEMİ (2026-08-18): engel yokken waypoint'e hizalanma
        # (near_field_heading_align) yalnız P1'de çalışır — P2'de (engelli
        # ortam) kaçınma önceliklidir; heading hizalaması kaçınmayı bloke eder.
        if (
            goal_body is not None
            and self.state == "PARKUR_1_NAV"
            and "recovery" not in str(candidate.action).lower()
        ):
            try:
                gx = float(goal_body[0])
                gy = float(goal_body[1])
                heading_error_deg = math.degrees(math.atan2(gy, gx))
            except (TypeError, ValueError, IndexError):
                heading_error_deg = None
        # Rosbag 2026-08-19: P2'de merkez engel kaçışı 200 saniyede 19 ayrı
        # reverse yayı ve 181 davranış geçişi üreterek aracı aynı çizgide
        # ileri/geri süpürdü. P2'nin yetkili kaçışı DWA'nın ileri/pivot recovery
        # katmanıdır; P3 araması da hiçbir zaman reverse olmamalıdır. Reverse
        # deadlock yardımcısı yalnız P1'de, açıkça override edilmedikçe aktiftir.
        if allow_reverse_escape is None:
            allow_reverse_escape = getattr(self, "state", "") == "PARKUR_1_NAV"

        near_field_obstacles = (
            self.obstacles
            if obstacles_override is None
            else obstacles_override
        )

        # P3'te doğrulanmış angajman hedefi bu listeden zaten çıkarılır. Kalan
        # engeller için P1/P2'nin geniş stop balonu hedef yaklaşımını gereksiz
        # yere kesmesin; P3'e özgü, gövde boyutuna yakın bir sert-stop ve ayrı
        # bırakma histerezisi kullanılır. P1/P2 saha eşikleri değişmeden kalır.
        p3_motion_state = getattr(self, "state", "") in {
            "PARKUR_3_TARGET_LOCK",
            "ENGAGE",
        }
        stop_m = (
            getattr(self, "p3_near_field_stop_m", self.near_field_stop_m)
            if p3_motion_state
            else self.near_field_stop_m
        )
        stop_release_m = (
            getattr(
                self,
                "p3_near_field_stop_release_m",
                self.near_field_stop_release_m,
            )
            if p3_motion_state
            else self.near_field_stop_release_m
        )

        decision = near_field_command(
            near_field_obstacles,
            candidate,
            stop_m=stop_m,
            slow_m=self.near_field_slow_m,
            stop_lateral_m=self.near_field_stop_lateral_m,
            slow_lateral_m=self.near_field_slow_lateral_m,
            pivot_yaw_deg_s=self.near_field_pivot_yaw_deg_s,
            slow_speed_mps=self.near_field_slow_speed_mps,
            max_yaw_rate_deg_s=self.max_yaw_rate,
            stop_release_m=stop_release_m,
            previous_latch=self._near_field_latch,
            heading_error_deg=heading_error_deg,
            heading_align_deadband_deg=10.0,
            heading_align_max_yaw_deg_s=(
                getattr(self, "near_field_heading_align_max_yaw_deg_s", 12.0)
            ),
            heading_align_speed_mps=getattr(
                self, "near_field_heading_align_speed_mps", 0.20
            ),
        )
        self._near_field_latch = decision.latch
        guarded = decision.command if decision.command is not None else candidate
        escape = getattr(self, "center_obstacle_escape", None)
        escape_reason = "disabled"
        if escape is not None and allow_reverse_escape:
            guarded, escape_reason = escape.apply(
                guarded, near_field_obstacles, self.now_sec()
            )
        elif escape is not None:
            # P3 search/reposition must not inherit an armed or active reverse
            # phase from navigation. Its safe fallback is stop/rotate/scan.
            escape.reset()
            escape_reason = "disabled_for_search"
        debug["near_field"] = {
            "active": decision.active,
            "latch": decision.latch,
            "reason": decision.reason,
            "escape": escape_reason,
            "stop_m": stop_m,
            "stop_release_m": stop_release_m,
        }
        return guarded, debug

    def goal_body_forward_m(self, wp: Dict[str, Any]) -> tuple[float, float]:
        """Waypoint -> gövde koordinatı (forward_m, lateral_m), metre.

        Telemetri lat/lon'u -> N/E farkı -> ``world_to_body`` ile gövdeye
        çevrilir. Telemetri doğrulanmış olmalı (tick akışı garantiler).
        """
        assert self.telemetry is not None
        dlat_rad = math.radians(float(wp["lat"]) - float(self.telemetry["lat"]))
        dlon_rad = math.radians(float(wp["lon"]) - float(self.telemetry["lon"]))
        # Dünya N/E farkı (geo.latlon_to_local_m ile aynı formül).
        dx_n = dlat_rad * 6371000.0
        dy_e = dlon_rad * 6371000.0 * math.cos(math.radians(float(self.telemetry["lat"])))
        heading = float(self.telemetry["heading_deg"])
        h = math.radians(heading)
        forward = math.cos(h) * dx_n + math.sin(h) * dy_e
        right = -math.sin(h) * dx_n + math.cos(h) * dy_e
        return forward, right

    def _remember_p3_virtual_target(self, target: Dict[str, Any]) -> None:
        """Latch a stationary fused target as a short-lived global waypoint."""

        if self.telemetry is None or not isinstance(target, dict):
            return
        try:
            distance = float(target["distance"])
            if "forward_m" in target and "lateral_m" in target:
                relative_bearing = math.degrees(
                    math.atan2(float(target["lateral_m"]), float(target["forward_m"]))
                )
            else:
                relative_bearing = float(target["bearing_deg"])
            heading = float(self.telemetry["heading_deg"])
            lat = float(self.telemetry["lat"])
            lon = float(self.telemetry["lon"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        if not all(
            math.isfinite(value)
            for value in (distance, relative_bearing, heading, lat, lon)
        ) or distance <= 0.0:
            return
        absolute_bearing = math.radians((heading + relative_bearing) % 360.0)
        north_m = distance * math.cos(absolute_bearing)
        east_m = distance * math.sin(absolute_bearing)
        target_lat, target_lon = local_m_to_latlon(lat, lon, north_m, east_m)
        self._p3_last_target_fix = (target_lat, target_lon)

    def _p3_virtual_target_command(self) -> Command:
        """Recompute guidance to the last fused target during a short dropout.

        The remembered buoy is stationary, so this is safer than replaying a
        stale motor command. The short configured loss grace keeps a bounded
        homing arc until the estimated contact zone.
        """

        target_fix = self._p3_last_target_fix
        if self.telemetry is None or target_fix is None:
            return p3_target_loss_grace_command(
                self._p3_last_tracking_command,
                self._p3_last_target_distance_m,
                self.p3_align_distance_m,
            )
        forward_m, right_m = self.goal_body_forward_m(
            {"lat": target_fix[0], "lon": target_fix[1]}
        )
        distance_m = math.hypot(forward_m, right_m)
        if not all(math.isfinite(value) for value in (forward_m, right_m, distance_m)):
            return Command(0.0, 0.0, 0.0, "target_loss_virtual_invalid")
        bearing_error_deg = math.degrees(math.atan2(right_m, forward_m))
        yaw_rate_deg_s = clamp(
            bearing_error_deg * self.yaw_kp,
            -self.p3_approach_yaw_max_deg_s,
            self.p3_approach_yaw_max_deg_s,
        )
        engage_distance_m = float(getattr(self, "p3_engage_distance_m", 1.2))
        # Son fix lidar-fused, doğrulanmış ve dünya koordinatında sabittir.
        # Eski davranış engage_distance'a (1.2 m) gelince vx=0 yapıyor, tekne
        # duba ile temas etmeden 3.5 s sonra anchor'a dönüyordu. Hedef kısa süre
        # FOV dışına çıksa da waypoint'e kadar kontrollü temas itkisini koru;
        # hedef tekne düzlemini geçtiğinde veya merkezler çok yaklaştığında kes.
        contact_stop_m = max(0.15, min(0.40, 0.25 * engage_distance_m))
        forward_allowed = forward_m > 0.0 and distance_m > contact_stop_m
        if distance_m <= engage_distance_m:
            requested_speed = float(
                getattr(self, "p3_engage_speed_mps", self.p3_lock_min_speed_mps)
            )
            action = "target_loss_virtual_contact_commit"
        elif distance_m > self.p3_align_distance_m:
            requested_speed = self.p3_approach_speed_mps
            action = "target_loss_virtual_waypoint"
        else:
            requested_speed = getattr(
                self, "p3_lock_min_speed_mps", self.p3_approach_speed_mps
            )
            action = "target_loss_virtual_waypoint"
        vx = clamp(requested_speed, 0.0, self.max_speed) if forward_allowed else 0.0
        return Command(
            vx,
            0.0,
            math.radians(yaw_rate_deg_s),
            f"{action} distance={distance_m:.1f}",
        )

    def publish_costmap(self) -> None:
        """Costmap'i /planning/costmap topic'ine String/JSON yayınlar (10 Hz)."""
        msg = String()
        msg.data = self.costmap.to_json()
        self.costmap_pub.publish(msg)

    def parkur_state_for_current_wp(self) -> str:
        if self.current_wp >= len(self.waypoints):
            return "PARKUR_3_TARGET_LOCK"
        parkur = int(self.waypoints[self.current_wp].get("parkur", 1))
        if parkur <= 1:
            return "PARKUR_1_NAV"
        if parkur == 2:
            return "PARKUR_2_AVOIDANCE"
        return "PARKUR_3_TARGET_LOCK"

    def _bench_flag(self) -> bool:
        """Bench oturumu etkinse True (analiz araçları gerçek mission'dan ayırsın)."""
        return bool(self.bench_p3_only_enabled)

    # --- Yayın ----------------------------------------------------------------

    def publish(self, command: Command, debug: Dict[str, Any]) -> None:
        course_debug = self._course_geometry_debug()
        twist = Twist()
        twist.linear.x = float(command.vx)
        twist.linear.y = float(command.vy)
        twist.angular.z = float(command.yaw_rate)
        self.cmd_pub.publish(twist)

        p1 = self.p1_stats_cache
        p2 = self.p2_stats_cache
        # Puan ceza modülü: penalty sürücü sinyali (yakın duba/engel -> yavaşla),
        # estimated_score_* tahmini puanlar (DEBUG; hakem sayısı değildir).
        # Report henüz oluşmadıysa None yerine 0.0 sentinel (L3) — JSON şeması
        # kararlı kalır, tüketici None işlemek zorunda kalmaz.
        p1_report = self.score_reports.get(1)
        p2_report = self.score_reports.get(2)
        p3_report = self.score_reports.get(3)
        penalty = 0.0
        if p1_report is not None:
            penalty = max(penalty, float(p1_report.penalty))
        if p2_report is not None:
            penalty = max(penalty, float(p2_report.penalty))
        state_msg = String()
        state_msg.data = dumps(
            {
                "stamp": self.now_sec(),
                "state": self.state,
                "bench": self._bench_flag(),
                "current_waypoint": self.current_wp,
                "target_color": self.target_color,
                "action": command.action,
                "total_waypoints": len(self.waypoints),
                "p1_crossed": p1.get("crossed_count", 0),
                "p1_kd": p1.get("kd_estimate", 0),
                "p1_ratio": p1.get("ratio", 0.0),
                "p2_crossed": p2.get("crossed_count", 0),
                "p2_kd_est": p2.get("kd_estimate", 0),
                "yellow_counter": self.yellow_count,
                "yellow_threshold": self.yellow_threshold,
                "ts3_risk": self.ts3_risk,
                "failsafe_reason": self.failsafe_reason,
                **course_debug,
                "penalty": round(penalty, 2),
                "estimated_score_1": (
                    round(float(p1_report.total), 2) if p1_report is not None else 0.0
                ),
                "estimated_score_2": (
                    round(float(p2_report.total), 2) if p2_report is not None else 0.0
                ),
                "estimated_score_3": (
                    round(float(p3_report.total), 2) if p3_report is not None else 0.0
                ),
            }
        )
        self.state_pub.publish(state_msg)

        # /autonomy/score: tahmini puan dökümü (her tick yayınlanır; canlı).
        # "events": itiraz adayı zaman damgalı olaylar (events_logger CSV'ye yazar).
        # Contact olaylar kapanınca (sustained/closed) yayınlanır; parkur dışı
        # olaylar her 40 sn'lik çıkışta (times_out artışı) tek satır üretir.
        score_events = self._score_events(now=self.now_sec())
        score_msg = String()
        score_msg.data = dumps(
            {
                "stamp": self.now_sec(),
                "state": self.state,
                "bench": self._bench_flag(),
                "estimated": True,
                "penalty": round(penalty, 2),
                "reports": {
                    str(k): {
                        "total": round(float(v.total), 2),
                        "penalty": round(float(v.penalty), 2),
                        "sections": v.sections,
                    }
                    for k, v in self.score_reports.items()
                },
                "events": score_events,
            }
        )
        self.score_pub.publish(score_msg)

        debug_msg = String()
        debug_payload = {
            "stamp": self.now_sec(),
            "command": command.__dict__,
            "bench": self._bench_flag(),
            "failsafe_reason": self.failsafe_reason,
            **debug,
            # Her erken dönüş/failsafe/yol-sonu publish yolu aynı kararlı
            # geometri şemasını taşır; debug içindeki eski alias bunu ezemez.
            **course_debug,
        }
        debug_payload.setdefault(
            "corridor_bias_configured", self._corridor_bias_configured()
        )
        debug_payload.setdefault("corridor_bias_active", False)
        if self.state == "PARKUR_2_AVOIDANCE" and self.p2_stats is not None:
            # Mevcut debug şemasını KORU; ikili geçiş bilgisi ekle.
            debug_payload["pair_gates_active"] = [g.key for g in self.p2_stats.active_gates]
            debug_payload["pair_crossed_keys"] = self.p2_stats.crossed_keys
            debug_payload["nearest_obstacle"] = (
                min(self.obstacles, key=lambda o: float(o.get("distance", 99.0)))
                if self.obstacles
                else None
            )
        debug_msg.data = dumps(debug_payload)
        self.debug_pub.publish(debug_msg)


def main(args=None) -> None:
    import rclpy  # main() içinde — modül import'u rclpy gerektirmez (test edilebilirlik)
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = AutonomyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
