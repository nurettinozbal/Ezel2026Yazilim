"""Mission-level planning helpers: waypoint navigation, corridor bias, obstacle
avoidance, target engagement and pair-gate (duba ikilisi) crossing detection.

All functions are pure w.r.t. time: the PairCrossingDetector receives `now` from
the caller so tests and the autonomy node stay deterministic.
"""

import math
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from ida_planning.geo import (
    bearing_deg,
    clamp,
    haversine_m,
    latlon_to_local_m,
    normalize_angle_deg,
)


@dataclass
class Command:
    vx: float
    vy: float
    yaw_rate: float
    action: str


@dataclass(frozen=True)
class CourseGeometryStatus:
    """Gerçek rota geometrisinin anlık durumu.

    Geçersiz/eksik rota veya telemetride ``valid=False`` olur; bu durumda
    ``inside`` ve ``distance_m`` bilerek ``None`` döner. Güvenlik latch'i bu
    durumdan bağımsızdır.
    """

    valid: bool
    inside: Optional[bool]
    distance_m: Optional[float]


def sanitize_mission_waypoints(payload: Any) -> Optional[List[Dict[str, Any]]]:
    """Mission mesajını atomik olarak doğrula ve kanonikleştir.

    Tek bir bozuk girdi bile kısmi rota kurulmasına yol açmamalıdır; bu
    nedenle geçersiz payload/entry/koordinat/parkur için ``None`` döner.
    """

    if isinstance(payload, dict):
        raw = payload.get("waypoints")
    elif isinstance(payload, list):
        raw = payload
    else:
        return None
    if not isinstance(raw, list) or not raw:
        return None
    result: List[Dict[str, Any]] = []
    for waypoint in raw:
        if not isinstance(waypoint, dict):
            return None
        parkur = waypoint.get("parkur")
        if isinstance(parkur, bool) or not isinstance(parkur, int) or parkur not in (1, 2, 3):
            return None
        try:
            lat = float(waypoint["lat"])
            lon = float(waypoint["lon"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return None
        if not (
            math.isfinite(lat)
            and math.isfinite(lon)
            and -90.0 <= lat <= 90.0
            and -180.0 <= lon <= 180.0
        ):
            return None
        clean = dict(waypoint)
        clean.update({"lat": lat, "lon": lon, "parkur": parkur})
        result.append(clean)
    return result


def mission_waypoint_fingerprint(waypoints: List[Dict[str, Any]]) -> tuple:
    """Tekrarlanan transient-local mission mesajları için kararlı kimlik."""

    return tuple((float(w["lat"]), float(w["lon"]), int(w["parkur"])) for w in waypoints)


def p1_route_exit_ready(
    waypoints: List[Dict[str, Any]],
    current_wp: int,
    current_distance_m: float,
    waypoint_threshold_m: float,
) -> bool:
    """Return whether the boat has actually reached the P1 route exit.

    Yellow P2 buoys can be visible before the last P1 waypoint.  They are only
    allowed to advance the phase after every P1-labelled waypoint before the
    current target has been consumed, or while the final P1 target is inside
    the normal waypoint acceptance radius.
    """

    p1_route, _ = split_parkur_routes(waypoints)
    if not p1_route:
        return False
    # Canonical missions are ordered P1 -> P2 -> P3.  Derive the boundary
    # from the labelled route split; never bake a competition-specific index.
    last_p1 = len(p1_route) - 1
    if current_wp > last_p1:
        return True
    try:
        distance = float(current_distance_m)
        threshold = float(waypoint_threshold_m)
    except (TypeError, ValueError, OverflowError):
        return False
    return (
        current_wp == last_p1
        and math.isfinite(distance)
        and math.isfinite(threshold)
        and threshold >= 0.0
        and distance <= threshold
    )


def p2_route_exit_ready(
    waypoints: List[Dict[str, Any]],
    current_wp: int,
    current_distance_m: float,
    waypoint_threshold_m: float,
) -> bool:
    """Return whether the final labelled P2 waypoint has been reached."""

    p1_route, p2_route = split_parkur_routes(waypoints)
    # split_parkur_routes includes the final P1 waypoint as the P2 anchor.
    p2_only_count = len(p2_route) - (1 if p1_route and p2_route else 0)
    if p2_only_count <= 0:
        return False
    last_p2 = len(p1_route) + p2_only_count - 1
    if current_wp > last_p2:
        return True
    try:
        distance = float(current_distance_m)
        threshold = float(waypoint_threshold_m)
    except (TypeError, ValueError, OverflowError):
        return False
    return (
        current_wp == last_p2
        and math.isfinite(distance)
        and math.isfinite(threshold)
        and threshold >= 0.0
        and distance <= threshold
    )


def task_timeout_expired(elapsed_s: float, max_duration_s: float) -> bool:
    """Return whether an optional task timeout expired (0 disables it)."""
    elapsed = float(elapsed_s)
    duration = float(max_duration_s)
    return duration > 0.0 and elapsed > duration


def p1_transition_decision(
    *,
    yellow_count: int,
    yellow_threshold: int,
    route_exit_ready: bool,
    all_waypoints_complete: bool,
    elapsed_s: float,
    max_duration_s: float,
) -> Optional[str]:
    """Return ``complete``, ``timeout`` or ``None`` for the P1 phase."""

    if all_waypoints_complete or (
        route_exit_ready and int(yellow_count) >= int(yellow_threshold)
    ):
        return "complete"
    # A non-positive duration explicitly disables task-completion timeout.
    # Sensor/telemetry freshness timeouts are separate and remain active.
    if task_timeout_expired(elapsed_s, max_duration_s):
        return "timeout"
    return None


def p2_transition_decision(
    *,
    crossed_count: int,
    min_crossings: int,
    route_complete: bool,
    elapsed_s: float,
    max_duration_s: float,
) -> Optional[str]:
    """Return P2 completion when the labelled route endpoint is reached.

    ``crossed_count`` and ``min_crossings`` remain in the contract for
    scoring/diagnostics, but camera-derived gate evidence is deliberately not
    a phase-transition interlock.  The authoritative mission waypoint is more
    reliable when real-world colour classification is intermittent.  A
    duration limit is still an explicit incomplete result, never implicit
    permission to start P3.
    """

    # Keep these arguments at the call boundary for backwards-compatible
    # telemetry/configuration, without making perception evidence authoritative.
    _ = crossed_count, min_crossings
    if route_complete:
        return "complete"
    if task_timeout_expired(elapsed_s, max_duration_s):
        return "timeout"
    return None


def split_parkur_routes(
    waypoints: List[Dict[str, Any]],
) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Hakem rotasını P1 ve P2 geometrilerine ayır.

    P1 yalnız ``parkur<=1`` noktalarıdır. P2, kesintisiz bir başlangıç
    segmenti oluşturmak için son P1 noktasını anchor olarak alır ve ardına
    ``parkur==2`` noktalarını ekler. Sonlu olmayan veya bozuk koordinatlar
    geometriye alınmaz.
    """

    p1: List[Dict[str, Any]] = []
    p2_only: List[Dict[str, Any]] = []
    for waypoint in waypoints if isinstance(waypoints, list) else []:
        if not isinstance(waypoint, dict):
            continue
        try:
            lat = float(waypoint["lat"])
            lon = float(waypoint["lon"])
            parkur = int(waypoint.get("parkur", 1))
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
        if not (math.isfinite(lat) and math.isfinite(lon)):
            continue
        clean = dict(waypoint)
        clean["lat"] = lat
        clean["lon"] = lon
        if parkur <= 1:
            p1.append(clean)
        elif parkur == 2:
            p2_only.append(clean)
    p2 = ([dict(p1[-1])] if p1 and p2_only else []) + p2_only
    return p1, p2


class CourseGeometryTracker:
    """Sonlu rota parçalarına minimum uzaklık + Schmitt histerezisi.

    Dışarıdayken yalnız ``distance<=enter_m`` ile içeri girer; içerideyken
    ``distance<=exit_m`` boyunca içeride kalır. Böylece eşik yakınındaki GPS
    gürültüsü debug durumunu tick'ten tick'e zıplatmaz.
    """

    def __init__(self, enter_m: float = 4.5, exit_m: float = 5.5) -> None:
        enter = float(enter_m)
        exit_ = float(exit_m)
        if not (
            math.isfinite(enter)
            and math.isfinite(exit_)
            and 0.0 <= enter <= exit_
        ):
            raise ValueError("course geometry thresholds must be finite and 0 <= enter <= exit")
        self.enter_m = enter
        self.exit_m = exit_
        self._origin: Optional[tuple[float, float]] = None
        self._points: List[tuple[float, float]] = []
        self._inside: Optional[bool] = None

    def set_route(self, waypoints: List[Dict[str, Any]]) -> None:
        """Rotayı yeniler ve histerezis durumunu sıfırlar."""

        valid: List[tuple[float, float]] = []
        for waypoint in waypoints if isinstance(waypoints, list) else []:
            if not isinstance(waypoint, dict):
                continue
            try:
                lat = float(waypoint["lat"])
                lon = float(waypoint["lon"])
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            if math.isfinite(lat) and math.isfinite(lon):
                valid.append((lat, lon))
        self._inside = None
        if len(valid) < 2:
            self._origin = None
            self._points = []
            return
        origin_lat, origin_lon = valid[0]
        locals_ = [
            latlon_to_local_m(origin_lat, origin_lon, lat, lon) for lat, lon in valid
        ]
        local_points = [(point.x, point.y) for point in locals_]
        if not all(math.isfinite(x) and math.isfinite(y) for x, y in local_points):
            self._origin = None
            self._points = []
            return
        self._origin = (origin_lat, origin_lon)
        self._points = local_points

    def reset_state(self) -> None:
        """Rotayı koruyup yalnız histerezis hafızasını temizler."""

        self._inside = None

    def update(self, lat: Any, lon: Any) -> CourseGeometryStatus:
        if self._origin is None or len(self._points) < 2:
            return CourseGeometryStatus(False, None, None)
        try:
            latitude = float(lat)
            longitude = float(lon)
        except (TypeError, ValueError, OverflowError):
            return CourseGeometryStatus(False, None, None)
        if not (math.isfinite(latitude) and math.isfinite(longitude)):
            return CourseGeometryStatus(False, None, None)
        point = latlon_to_local_m(
            self._origin[0], self._origin[1], latitude, longitude
        )
        if not (math.isfinite(point.x) and math.isfinite(point.y)):
            return CourseGeometryStatus(False, None, None)
        distance = min(
            _finite_segment_distance_m(point.x, point.y, ax, ay, bx, by)
            for (ax, ay), (bx, by) in zip(self._points[:-1], self._points[1:])
        )
        if self._inside is True:
            self._inside = distance <= self.exit_m
        else:
            self._inside = distance <= self.enter_m
        return CourseGeometryStatus(True, self._inside, distance)


def _finite_segment_distance_m(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    """Noktadan sonlu doğru parçasına Öklid uzaklığı."""

    abx = bx - ax
    aby = by - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / denom
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * abx), py - (ay + t * aby))


def update_strict_orange_latch(
    previous: bool, state: str, *, new_mission: bool = False
) -> bool:
    """Mission-scope sert turuncu güvenlik latch'inin saf geçişi.

    Yeni mission/reset tek temizleme yoludur. P1 veya P2 navigasyonunun
    başlaması latch'i kurar; sonraki geometrik durumlar ve parkur geçişi onu
    çözemez.
    """

    if new_mission:
        return False
    return bool(previous) or state in {"PARKUR_1_NAV", "PARKUR_2_AVOIDANCE"}


def course_geometry_parkur(
    state: str, *, telemetry_fresh: bool, failsafe_resume_state: Optional[str] = None
) -> Optional[int]:
    """Debug geometrisi için güvenli aktif parkuru seç."""

    if not telemetry_fresh:
        return None
    active = failsafe_resume_state if state == "FAILSAFE" else state
    if active == "PARKUR_1_NAV":
        return 1
    if active == "PARKUR_2_AVOIDANCE":
        return 2
    return None


def waypoint_command(
    telemetry: Dict[str, Any],
    waypoint: Dict[str, Any],
    max_speed: float,
    max_yaw_rate: float,
    yaw_kp: float,
) -> Command:
    target_bearing = bearing_deg(telemetry["lat"], telemetry["lon"], waypoint["lat"], waypoint["lon"])
    yaw_error = normalize_angle_deg(target_bearing - telemetry["heading_deg"])
    distance = haversine_m(telemetry["lat"], telemetry["lon"], waypoint["lat"], waypoint["lon"])

    speed_scale = 1.0
    if distance < 5.0:
        speed_scale = 0.35
    elif distance < 15.0:
        speed_scale = 0.65

    if abs(yaw_error) > 35.0:
        speed_scale *= 0.35

    yaw_rate = math.radians(clamp(yaw_error * yaw_kp, -max_yaw_rate, max_yaw_rate))
    return Command(
        vx=max_speed * speed_scale,
        vy=0.0,
        yaw_rate=yaw_rate,
        action=f"waypoint distance={distance:.1f} yaw_error={yaw_error:.1f}",
    )


def apply_corridor_bias(command: Command, buoys: List[Dict[str, Any]], max_yaw_rate: float) -> Command:
    orange = [b for b in buoys if b.get("color") == "orange" and b.get("confidence", 0.0) >= 0.35]
    front = [b for b in orange if b.get("distance", 99.0) < 18.0]
    if len(front) < 2:
        return command

    # Positive bbox_norm_x means the visual center is to the right. If the average
    # corridor center drifts right, steer slightly right, otherwise left.
    avg_center = sum(float(b.get("bbox_norm_x", 0.0)) for b in front[:4]) / min(len(front), 4)
    correction = math.radians(clamp(avg_center * 18.0, -max_yaw_rate * 0.35, max_yaw_rate * 0.35))
    return Command(command.vx, command.vy, command.yaw_rate + correction, f"{command.action} corridor_bias={avg_center:.2f}")


def apply_obstacle_avoidance(
    command: Command,
    obstacles: List[Dict[str, Any]],
    avoid_distance_m: float,
    max_yaw_rate: float,
    corridor_half_width_m: float = 2.5,
    avoid_speed_mps: float = 0.28,
) -> Command:
    """Slow down and steer around obstacles inside the travel corridor.

    Obstacles count as dangerous when their distance is <= avoid_distance_m, the
    lateral offset fits inside the corridor (|lateral_m| <= corridor_half_width_m)
    and they are still ahead (forward_m > 0). The boat passes the nearest danger
    on the opposite side: obstacle on the left -> turn right, obstacle on the
    right -> turn left. vy stays 0 (two-thruster differential boat).
    """
    danger = [
        o
        for o in obstacles
        if float(o.get("distance", 99.0)) <= avoid_distance_m
        and abs(float(o.get("lateral_m", o.get("y", 99.0)))) <= corridor_half_width_m
        and float(o.get("forward_m", o.get("x", 0.0))) > 0.0
    ]
    if not danger:
        return command

    nearest = min(danger, key=lambda o: float(o.get("distance", 99.0)))
    lateral = float(nearest.get("lateral_m", nearest.get("y", 0.0)))
    # Obstacle on the left (negative lateral) -> turn right; on the right -> turn left.
    if lateral < 0.0:
        yaw_rate = math.radians(-max_yaw_rate * 0.75)
        action = "avoid_left"
    else:
        yaw_rate = math.radians(max_yaw_rate * 0.75)
        action = "avoid_right"
    vx = min(command.vx, avoid_speed_mps)
    return Command(vx=vx, vy=0.0, yaw_rate=yaw_rate, action=f"{action} obstacle={nearest.get('distance', 0):.1f}m")


@dataclass
class NearFieldDecision:
    """Yakın-alan güvenlik kararı: komut ve eşik durumu.

    ``active``: bu tick'te yakın-alan kuralı komutu değiştirdi mi?
    ``latch``: histerezis kilitli mi (engel stop eşiğinin üstüne çıksa bile
    stop sürüyor)?
    ``reason``: hangi engel/kural tetikledi (debug).
    """

    active: bool
    latch: bool
    reason: str
    command: Optional[Command] = None


def near_field_command(
    obstacles: List[Dict[str, Any]],
    command: Command,
    *,
    stop_m: float = 1.5,
    slow_m: float = 3.0,
    stop_lateral_m: float = 1.0,
    slow_lateral_m: Optional[float] = None,
    pivot_yaw_deg_s: float = 30.0,
    slow_speed_mps: float = 0.30,
    max_yaw_rate_deg_s: float = 50.0,
    stop_release_m: float = 2.5,
    previous_latch: bool = False,
    heading_error_deg: Optional[float] = None,
    heading_align_deadband_deg: float = 10.0,
    heading_align_max_yaw_deg_s: float = 20.0,
    heading_align_min_yaw_deg_s: float = 0.0,
    heading_align_speed_mps: Optional[float] = None,
) -> NearFieldDecision:
    """Hard obstacle yakın-alan stop/pivot + histerezisli hız freni.

    Saha bulgusu (2026-08-17): hard obstacle 0.93-1.56 m'de iken bazı
    komutlarda ``v=0.7 m/s`` korundu. DWA ``min_drive_speed`` yüzünden yakın
    engelde bile ileri aday seçiyor; costmap şişirmesi hıza bağlı olduğundan
    hızlı adaylar geçebiliyor. Bu katman DWA skorlamasından BAĞIMSIZ kesin bir
    güvenlik kuralıdır:

    * ``distance <= stop_m`` ve ``|lateral| <= stop_lateral_m`` ve engel
      önde/yarım gövde uzunluğu içinde ise → ``vx=0`` + engelden uzaklaşan pivot.
    * ``distance <= slow_m``, ``|lateral| <= slow_lateral_m`` ve önde ise
      → ``vx`` düşürülür (``slow_speed_mps``). Yalnız yanda kalan engel
      waypoint hizalamasını yerinde pivota kilitlemez.
    * Histerezis: stop aktifken engel ``stop_release_m`` üstüne çıkmadan ileri
      komut açılmaz (tekrar ileri açılış kararlı kanıt ister).

    ``hard_obstacle`` bayrağı korunur: lidar unknown obstacle'ları (renk
    algılanmasa bile) bu kurala girer. P3 hedef angajmanı bu katmandan
    ETKİLENMEZ — P3 kendi mesafe/merkezleme kurallarına tabidir (hedef 1.2 m'de
    angaje olur; ileri komut kesilirse temas imkânsız olur).
    """
    max_yaw = math.radians(float(max_yaw_rate_deg_s))
    pivot_yaw = max(0.0, math.radians(float(pivot_yaw_deg_s)))
    pivot_yaw = min(pivot_yaw, max_yaw)
    stop_m = float(stop_m)
    slow_m = float(slow_m)
    stop_lateral = float(stop_lateral_m)
    slow_lateral = (
        float("inf") if slow_lateral_m is None else float(slow_lateral_m)
    )
    release_m = float(stop_release_m)
    slow_speed = float(slow_speed_mps)

    # En yakın öndeki hard obstacle'ı bul (gövde frame).
    nearest_fwd: Optional[float] = None
    nearest_lat: Optional[float] = None
    nearest_dist = float("inf")
    for obs in obstacles:
        if not isinstance(obs, dict):
            continue
        # Soft/annotative obstacles must never mask a physical hard obstacle
        # that is slightly farther away.  Lidar unknowns default to hard, as
        # required by the field safety contract; only an explicit
        # ``hard_obstacle: false`` is ignored by this guard.
        if not bool(obs.get("hard_obstacle", True)):
            continue
        try:
            forward = float(obs.get("forward_m", obs.get("x", 0.0)))
            lateral = float(obs.get("lateral_m", obs.get("y", 0.0)))
        except (TypeError, ValueError, OverflowError):
            continue
        if not (math.isfinite(forward) and math.isfinite(lateral)):
            continue
        # Arkada kalan engel ileri komutu etkilemez (araç geri gitmez).
        if forward < -0.59:  # yarım gövde uzunluğu
            continue
        distance = math.hypot(forward, lateral)
        if distance < nearest_dist:
            nearest_dist = distance
            nearest_fwd = forward
            nearest_lat = lateral

    if nearest_fwd is None:
        # Engel yok. Heading hatası varsa ufak yaw ile waypoint'e hizalan
        # (kullanıcı gözlemi 2026-08-18); yoksa komut değişmez.
        if heading_error_deg is not None:
            err = float(heading_error_deg)
            if math.isfinite(err) and abs(err) > float(heading_align_deadband_deg):
                max_yaw = max(0.0, float(heading_align_max_yaw_deg_s))
                min_yaw = max(0.0, float(heading_align_min_yaw_deg_s))
                yaw_mag_deg = clamp(abs(err) * 0.5, min_yaw, max_yaw)
                yaw = math.radians(math.copysign(yaw_mag_deg, err))
                yaw = clamp(yaw, -math.radians(float(max_yaw_rate_deg_s)),
                            math.radians(float(max_yaw_rate_deg_s)))
                aligned_vx = float(command.vx)
                if heading_align_speed_mps is not None:
                    aligned_vx = min(
                        max(0.0, aligned_vx),
                        max(0.0, float(heading_align_speed_mps)),
                    )
                    # Differential boats/SITL barely yaw at exactly zero
                    # surge. A bounded crawl turns the hull without the
                    # 0.6 m/s overshoot observed at waypoint corners.
                    if aligned_vx <= 0.0:
                        aligned_vx = max(0.0, float(heading_align_speed_mps))
                return NearFieldDecision(
                    True, False,
                    f"near_field_heading_align err={err:.1f}deg",
                    Command(aligned_vx, command.vy, yaw,
                            f"near_field_heading_align {command.action}"),
                )
        return NearFieldDecision(False, False, "none")

    # STOP bölgesi: araç gövdesiyle çakışıyor / temas riski. YALNIZCA engel
    # ÖNDE ve lateral stop_lateral içindeyse (gövdeyle çakışma riski) tetiklenir.
    # KULLANICI GÖZLEMİ (2026-08-18): araç engelin YANINDA stuck kalıyordu —
    # latch (`stop_hold`) yanal engelde de vx=0 tutuyordu. Düzeltme:
    # - in_stop: engel önde (forward > -0.59) VE lateral stop_lateral içinde.
    # - stop_hold (latch): yalnız engel HÂLÂ stop geometrisindeyse sürer;
    #   engel yanala geçtiyse latch kırılır (araç ilerleyebilir).
    # - near_field_stop_toward_goal KALDIRILDI: engel yakınında hedefe dönüş
    #   kaçınmayı bloke ediyordu. Pivot daima engelden uzaklaşır.
    in_stop = (
        nearest_dist <= stop_m
        and abs(nearest_lat) <= stop_lateral
        and nearest_fwd > -0.59
    )
    # Histerezis: stop başladıysa release eşiğine kadar sürer, AMA yalnız engel
    # hâlâ stop geometrisindeyse (yanal latch kırılır — araç dönüp engeli
    # yanala aldıysa ilerlemeye devam edebilir).
    stop_hold = bool(
        previous_latch
        and nearest_dist <= release_m
        and abs(nearest_lat) <= stop_lateral
        and nearest_fwd > -0.59
    )
    if in_stop or stop_hold:
        # Engelden uzaklaşan pivot: engel soldaysa sağa (+), sağdaysa sola (-).
        pivot_dir = -1.0 if nearest_lat >= 0.0 else 1.0
        yaw = math.copysign(pivot_yaw, pivot_dir)
        action = "near_field_stop"
        if abs(nearest_lat) < 0.15:
            # Merkezdeki engel: deterministik sağ pivot (eski recovery yönü).
            yaw = pivot_yaw
            action = "near_field_stop_center"
        return NearFieldDecision(
            True, True,
            f"{action} dist={nearest_dist:.2f}m lat={nearest_lat:.2f}m",
            Command(0.0, 0.0, yaw, action),
        )

    # YAVAŞ bölge: önde yakın engel varsa ileri hız düşürülür.
    if (
        nearest_dist <= slow_m
        and abs(nearest_lat) <= slow_lateral
        and nearest_fwd > 0.0
    ):
        new_vx = min(float(command.vx), slow_speed)
        return NearFieldDecision(
            True, False,
            f"near_field_slow dist={nearest_dist:.2f}m lat={nearest_lat:.2f}m",
            Command(new_vx, command.vy, command.yaw_rate,
                    f"near_field_slow {command.action}"),
        )

    # Engeller uzak: histerezis dışında normal komut. Engel YOKSA ve waypoint
    # heading hatası deadband üstündeyse, araç kendini waypoint'e küçük yaw ile
    # hizalar (kullanıcı gözlemi 2026-08-18: kaçınma sonrası düz gidiyor, heading
    # düzeltmiyor). Yalpalama olmaması için hata deadband ALTINA inene kadar
    # düzeltme sürer (Schmitt benzeri histerezis).
    if heading_error_deg is not None:
        err = float(heading_error_deg)
        if not math.isfinite(err):
            err = 0.0
        deadband = float(heading_align_deadband_deg)
        max_yaw = max(0.0, float(heading_align_max_yaw_deg_s))
        min_yaw = max(0.0, float(heading_align_min_yaw_deg_s))
        # Hata pozitif = sağa dönülmeli (yaw > 0); negatif = sola.
        if abs(err) > deadband:
            yaw_mag_deg = clamp(abs(err) * 0.5, min_yaw, max_yaw)
            yaw = math.radians(math.copysign(yaw_mag_deg, err))
            yaw = clamp(yaw, -math.radians(float(max_yaw_rate_deg_s)),
                        math.radians(float(max_yaw_rate_deg_s)))
            aligned_vx = float(command.vx)
            if heading_align_speed_mps is not None:
                aligned_vx = min(
                    max(0.0, aligned_vx),
                    max(0.0, float(heading_align_speed_mps)),
                )
                if aligned_vx <= 0.0:
                    aligned_vx = max(0.0, float(heading_align_speed_mps))
            # Köşede düz seyir hızını korumak büyük heading sıçraması ve ters
            # komut salınımı üretir; dönüş yayı ayrı hız tavanına sahiptir.
            return NearFieldDecision(
                True, False,
                f"near_field_heading_align err={err:.1f}deg",
                Command(aligned_vx, command.vy, yaw,
                        f"near_field_heading_align {command.action}"),
            )
    return NearFieldDecision(False, False, "clear")


def wrong_target_risk(
    buoys: List[Dict[str, Any]],
    target_color: str,
    risk_radius_m: float = 2.5,
    min_confidence: float = 0.45,
) -> Optional[Dict[str, Any]]:
    """Nearest wrong-color selectable target inside the risk radius.

    Returns the buoy dict of the closest risk buoy, or None when no wrong-colored
    target is dangerously close. Used during Parkur 3 to avoid ramming the wrong
    target buoy.
    """
    wrong = [
        b
        for b in buoys
        if str(b.get("color", "")).lower() != target_color.lower()
        and str(b.get("color", "")).lower() in {"red", "green", "black", "orange", "yellow"}
        and float(b.get("confidence", 0.0)) >= min_confidence
        and float(b.get("distance", 99.0)) < risk_radius_m
    ]
    if not wrong:
        return None
    return min(wrong, key=lambda b: float(b.get("distance", 99.0)))


def target_engagement_command(
    target_color: str,
    buoys: List[Dict[str, Any]],
    max_speed: float,
    max_yaw_rate: float,
    min_confidence: float,
    wrong_avoid_lateral_m: float = 1.5,
    wrong_avoid_speed_mps: float = 0.35,
    search_yaw_rate_deg_s: float = 15.0,
    lock_yaw_gain_deg_s: float = 35.0,
    engage_center_max: float = 0.18,
    engage_distance_m: float = 1.2,
    lock_min_speed_mps: float = 0.0,
    engage_speed_mps: float = 0.18,
    engage_allowed: bool = True,
    align_distance_m: float = 5.0,
    approach_speed_mps: float = 0.4,
    approach_yaw_max_deg_s: float = 12.0,
    search_advance_after_rotations: int = 0,
    search_advance_speed_mps: float = 0.5,
    search_advance_heading_deg: Optional[float] = None,
) -> Command:
    """Lock onto the target-color buoy and steer into it.

    When a wrong-colored target is dangerously close (wrong_target_risk), the boat
    keeps moving slowly and steers away from it instead of stopping. The risk count
    is appended to the action string so operators can watch it in /autonomy/debug.
    """
    candidates = [
        b
        for b in buoys
        if str(b.get("color", "")).lower() == target_color.lower()
        and float(b.get("confidence", 0.0)) >= min_confidence
    ]

    risk = wrong_target_risk(buoys, target_color, risk_radius_m=2.5, min_confidence=min_confidence)
    # ts3_risk: risk yarıçapı içindeki yanlış renkli hedef sayısı (DEBUG/operatör için).
    # Güven filtresi BİLEREK yok: düşük güvenli ama yakın yanlış hedefler de sayılır,
    # böylece avoid tetiklenmese bile operatör riski görür.
    wrong_count = sum(
        1
        for b in buoys
        if str(b.get("color", "")).lower() != target_color.lower()
        and str(b.get("color", "")).lower() in {"red", "green", "black", "orange", "yellow"}
        and float(b.get("distance", 99.0)) < 2.5
    )
    if risk is not None:
        # Yanlış hedeften kaçın ama tam durma YOK: yavaşça ilerlerken yana kaç.
        k = 8.0
        yaw_rate = math.radians(clamp(risk.get("lateral_m", 0.0) * k, -max_yaw_rate, max_yaw_rate))
        return Command(wrong_avoid_speed_mps, 0.0, yaw_rate, "avoid_wrong_target")
    if not candidates:
        # Kullanıcı gözlemi (2026-08-18): P3'te hedef bulunamazsa sabit dönüş
        # sonsuz sürer, araç hiç ilerlemez. `search_advance_after_rotations`
        # kadar tam dönüş sonrası hâlâ hedef yoksa, araç waypoint yönüne doğru
        # ileri gider (hedef genellikle parkur sonundadır). `search_advance_heading_deg`
        # verilirse o yöne ilerlenir (son waypoint bearing'i); yoksa mevcut dönüş
        # yönünde ileri gidilir.
        rotations_done = int(search_advance_after_rotations)
        if rotations_done > 0 and search_advance_heading_deg is not None:
            advance_yaw = math.radians(
                clamp(
                    float(search_advance_heading_deg),
                    -float(max_yaw_rate),
                    float(max_yaw_rate),
                )
            )
            return Command(
                float(search_advance_speed_mps),
                0.0,
                advance_yaw,
                f"search_advance ts3_risk={wrong_count}",
            )
        return Command(
            0.0,
            0.0,
            math.radians(clamp(search_yaw_rate_deg_s, 1.0, max_yaw_rate)),
            f"search_target_360 ts3_risk={wrong_count}",
        )

    target = min(candidates, key=lambda b: float(b.get("distance", 99.0)))
    center = float(target.get("bbox_norm_x", 0.0))
    distance = float(target.get("distance", 99.0))
    yaw_rate = math.radians(
        clamp(center * lock_yaw_gain_deg_s, -max_yaw_rate, max_yaw_rate)
    )
    approach_yaw_rate = math.radians(
        clamp(
            center * lock_yaw_gain_deg_s,
            -min(float(max_yaw_rate), float(approach_yaw_max_deg_s)),
            min(float(max_yaw_rate), float(approach_yaw_max_deg_s)),
        )
    )

    # Doğrulama ve burun hizalama tamamlanmadan ileri itki verme. bbox_size
    # yalnız teşhistir; fiziksel temas kararı fusion/lidar mesafesine dayanır.
    if not engage_allowed:
        confirm_yaw = approach_yaw_rate if distance > align_distance_m else yaw_rate
        return Command(0.0, 0.0, confirm_yaw, "target_confirming")
    # Far target: do not stop for pixel-perfect alignment. A bounded forward
    # arc brings the target into the camera center and closes range. The bag
    # regression showed stationary 35 deg/s alignment repeatedly throwing a
    # 22-26 m target out of the FOV without reducing distance.
    if distance > float(align_distance_m):
        vx = clamp(float(approach_speed_mps), 0.0, float(max_speed))
        return Command(
            vx,
            0.0,
            approach_yaw_rate,
            f"target_approach color={target_color} distance={distance:.1f} "
            f"center={center:.2f} ts3_risk={wrong_count}",
        )
    if abs(center) >= engage_center_max:
        # 2026-08-21 P3 telemetrisi: 0.7 m/s ile kararlı yaklaşan araç
        # 5 m sınırında vx=0 + tam yaw'a geçince hedefi FOV dışına
        # attı ve anchor aramasına döndü. Seçili/doğrulanmış hedefte
        # durarak piksel-merkezleme yerine sınırlı homing yayını koru.
        vx = clamp(float(lock_min_speed_mps), 0.0, float(max_speed))
        return Command(
            vx,
            0.0,
            approach_yaw_rate,
            f"target_align_moving ts3_risk={wrong_count}",
        )

    engage_geometry = distance < engage_distance_m
    if engage_geometry:
        vx = clamp(float(engage_speed_mps), 0.0, float(max_speed))
        return Command(vx, 0.0, yaw_rate, f"engage ts3_risk={wrong_count}")
    if distance < 3.0:
        vx = max_speed * 0.35
    else:
        vx = max_speed * 0.65
    vx = clamp(max(vx, float(lock_min_speed_mps)), 0.0, float(max_speed))
    return Command(vx, 0.0, yaw_rate, f"target_lock color={target_color} distance={distance:.1f} center={center:.2f} ts3_risk={wrong_count}")


def _buoy_key(buoy: Dict[str, Any]) -> str:
    """Stable key for a buoy: explicit id if present, else rounded lateral_m."""
    buoy_id = buoy.get("id")
    if buoy_id is not None:
        return str(buoy_id)
    return f"L{round(float(buoy.get('lateral_m', 0.0)) * 2.0) / 2.0:.1f}"


@dataclass
class PairGate:
    key: str                 # örn. "l1|r1" — sol/sağ duba id'leri (id yoksa geometrik anahtar)
    left_id: str
    right_id: str
    left_behind: bool = False   # sol duba artık arkada (forward_m < 0)
    right_behind: bool = False
    crossed: bool = False
    last_seen_stamp: float = 0.0
    explicit_ids: bool = False
    center_forward_m: float = 0.0
    center_lateral_m: float = 0.0
    width_m: float = 0.0
    left_behind_stamp: Optional[float] = None
    right_behind_stamp: Optional[float] = None


@dataclass
class PairCrossingStats:
    crossed_count: int      # farklı ikililerden geçiş sayısı (G)
    kd_estimate: int        # algıdan tahmini toplam ikili sayısı
    ratio: float            # crossed_count / max(1, kd_estimate)
    active_gates: List[PairGate] = field(default_factory=list)
    crossed_keys: List[str] = field(default_factory=list)


class PairCrossingDetector:
    """Detects when the boat crosses a pair of opposite-lateral orange buoys.

    Orange buoys seen on opposite laterals are paired into gates. A gate is
    crossed when both buoys fall behind the boat (forward_m < cross_complete_forward_m).
    Each gate is counted once. Determinism is guaranteed by passing `now` from the
    caller (a ROS2 clock in the node, a fixed value in tests).
    """

    def __init__(
        self,
        max_pair_lateral_m: float = 15.0,
        min_lateral_gap_m: float = 1.0,
        cross_trigger_forward_m: float = 3.0,
        cross_complete_forward_m: float = -0.5,
        min_confidence: float = 0.35,
        max_age_s: float = 20.0,
        max_pair_forward_gap_m: float = 3.0,
        max_pair_crossing_gap_s: float = 2.0,
        allow_sim_truth_geometry: bool = False,
    ) -> None:
        values = {
            "max_pair_lateral_m": max_pair_lateral_m,
            "min_lateral_gap_m": min_lateral_gap_m,
            "cross_trigger_forward_m": cross_trigger_forward_m,
            "cross_complete_forward_m": cross_complete_forward_m,
            "min_confidence": min_confidence,
            "max_age_s": max_age_s,
            "max_pair_forward_gap_m": max_pair_forward_gap_m,
            "max_pair_crossing_gap_s": max_pair_crossing_gap_s,
        }
        parsed: Dict[str, float] = {}
        for name, value in values.items():
            number = _finite_float(value)
            if number is None:
                raise ValueError(f"{name} sonlu olmalı")
            parsed[name] = number
        if parsed["min_lateral_gap_m"] <= 0.0:
            raise ValueError("min_lateral_gap_m > 0 olmalı")
        if parsed["max_pair_lateral_m"] < parsed["min_lateral_gap_m"]:
            raise ValueError("max_pair_lateral_m >= min_lateral_gap_m olmalı")
        for name in ("max_age_s", "max_pair_forward_gap_m", "max_pair_crossing_gap_s"):
            if parsed[name] <= 0.0:
                raise ValueError(f"{name} > 0 olmalı")
        if not 0.0 <= parsed["min_confidence"] <= 1.0:
            raise ValueError("min_confidence 0..1 aralığında olmalı")
        self.max_pair_lateral_m = parsed["max_pair_lateral_m"]
        self.min_lateral_gap_m = parsed["min_lateral_gap_m"]
        self.cross_trigger_forward_m = parsed["cross_trigger_forward_m"]
        self.cross_complete_forward_m = parsed["cross_complete_forward_m"]
        self.min_confidence = parsed["min_confidence"]
        self.max_age_s = parsed["max_age_s"]
        self.max_pair_forward_gap_m = parsed["max_pair_forward_gap_m"]
        self.max_pair_crossing_gap_s = parsed["max_pair_crossing_gap_s"]
        self.allow_sim_truth_geometry = bool(allow_sim_truth_geometry)
        self.gates: Dict[str, PairGate] = {}
        self.crossed_history: Dict[str, PairGate] = {}
        # Doğrulanıp tam oluşturulmuş fiziksel gate kimliklerinin monotonic
        # mission-scope geçmişi. Stale track silinmesi KD tahminini azaltamaz.
        self.seen_gate_keys: set[str] = set()
        self._next_fallback_id = 1

    def reset(self) -> None:
        """Clear all tracked gates; used when moving to a new parkur section."""
        self.gates.clear()
        self.crossed_history.clear()
        self.seen_gate_keys.clear()
        self._next_fallback_id = 1

    def update(self, buoys: List[Dict[str, Any]], now: float) -> PairCrossingStats:
        valid_now = _finite_float(now)
        if valid_now is None:
            return self._stats()
        now = valid_now
        # Association'dan önce stale track'leri serbest bırak. Aksi halde
        # t=1'de görülen geo:1, arada boş frame olmadan t=100'de yeniden gelen
        # farklı bir gate'e eşlenip last_seen'i yeniler ve max_age sözleşmesini
        # fiilen bypass eder.
        self._expire_stale(now)
        # Sayısal alanları bir kez güvenli parse et; malformed/NaN/Inf detection
        # tüm update'i düşürmez, yalnız ilgili detection yok sayılır.
        orange: List[Dict[str, Any]] = []
        for raw in buoys if isinstance(buoys, list) else []:
            if not isinstance(raw, dict) or str(raw.get("color", "")).lower() != "orange":
                continue
            confidence = _finite_float(raw.get("confidence", 0.0))
            forward = _finite_float(raw.get("forward_m"))
            lateral = _finite_float(raw.get("lateral_m"))
            if (
                confidence is None
                or forward is None
                or lateral is None
                or confidence < self.min_confidence
            ):
                continue
            buoy = dict(raw)
            buoy.update(confidence=confidence, forward_m=forward, lateral_m=lateral)
            orange.append(buoy)

        explicit_groups: Dict[str, Dict[str, Dict[str, Any]]] = {}
        fallback_left: List[Dict[str, Any]] = []
        fallback_right: List[Dict[str, Any]] = []
        for buoy in orange:
            identity = _explicit_pair_identity(buoy)
            if identity is not None:
                pair_key, side = identity
                # Aynı stable ID bir mesajda tekrarlanırsa yalnız en güvenilir
                # detection kullanılır; bir buoy iki gate'e asla tahsis edilmez.
                group = explicit_groups.setdefault(pair_key, {})
                previous = group.get(side)
                if previous is None or _detection_rank(buoy) < _detection_rank(previous):
                    group[side] = buoy
                continue
            # Stable course namespace'ine benzeyen bozuk ID anonim tracker'a
            # düşerse yanlış bir fiziksel gate üretebilir. Böyle kayıtları yok
            # say; yalnız genel/opaque tracker ID'leri geometrik fallback alır.
            if _malformed_course_pair_id(buoy.get("id")):
                continue
            lateral = float(buoy.get("lateral_m", 0.0))
            if -self.max_pair_lateral_m <= lateral <= -self.min_lateral_gap_m:
                fallback_left.append(buoy)
            elif self.min_lateral_gap_m <= lateral <= self.max_pair_lateral_m:
                fallback_right.append(buoy)

        # Stable explicit ID'ler her zaman geometrik fallback'ten öncelikli ve
        # tek eşleşmedir: p1_o_l7 yalnız p1_o_r7 ile gate oluşturabilir.
        for pair_key in sorted(explicit_groups):
            group = explicit_groups[pair_key]
            gate = self.gates.get(pair_key) or self.crossed_history.get(pair_key)
            left, right = group.get("left"), group.get("right")
            if gate is None and left is not None and right is not None:
                if not self._valid_pair(left, right) or not self._new_pair_is_ahead(left, right):
                    continue
                gate = self._new_gate(pair_key, left, right, now, explicit_ids=True)
                self.gates[pair_key] = gate
            if gate is not None:
                if left is not None and right is not None and not self._valid_pair(left, right):
                    continue
                # Tek taraflı explicit gözlemde `_valid_pair` çağrılamaz;
                # yine de bozuk/yanlış association (ornegin lateral=100 m)
                # mevcut gate'in crossing latch'ini tetiklememelidir. Tekne dönünce
                # ID tarafı ile görüntü tarafı yer değiştirebildiğinden burada
                # lateral işareti değil yalnız fiziksel tekil sınır doğrulanır.
                if left is not None and not self._valid_side_observation(left):
                    left = None
                if right is not None and not self._valid_side_observation(right):
                    right = None
                self._observe_gate(gate, left, right, now)

        # ID'siz detection'lar: önce aynı frame içinde global düşük maliyet
        # sırasıyla bire-bir eşlenir, sonra gate merkez/genişlik durumuna göre
        # geçmiş track'lere temporal association yapılır.
        fallback_pairs = self._minimum_cost_pairs(fallback_left, fallback_right)
        fallback_pairs = [pair for pair in fallback_pairs if self._valid_pair(*pair)]
        available_gates = [
            gate for gate in self.gates.values() if not gate.explicit_ids and not gate.crossed
        ]
        associations: List[tuple[float, int, int]] = []
        for pair_index, (left, right) in enumerate(fallback_pairs):
            center_fwd, center_lat, width = _pair_geometry(left, right)
            for gate_index, gate in enumerate(available_gates):
                cost = (
                    abs(center_fwd - gate.center_forward_m)
                    + abs(center_lat - gate.center_lateral_m)
                    + 0.5 * abs(width - gate.width_m)
                )
                if cost <= 8.0:
                    associations.append((cost, pair_index, gate_index))
        used_pairs: set[int] = set()
        used_gates: set[int] = set()
        for _cost, pair_index, gate_index in sorted(associations):
            if pair_index in used_pairs or gate_index in used_gates:
                continue
            left, right = fallback_pairs[pair_index]
            self._observe_gate(available_gates[gate_index], left, right, now)
            used_pairs.add(pair_index)
            used_gates.add(gate_index)
        for pair_index, (left, right) in enumerate(fallback_pairs):
            if pair_index in used_pairs or not self._new_pair_is_ahead(left, right):
                continue
            key = f"geo:{self._next_fallback_id}"
            self._next_fallback_id += 1
            self.gates[key] = self._new_gate(key, left, right, now, explicit_ids=False)

        # Geçişleri say (aynı ikiliden yalnızca bir kez).
        for gate in list(self.gates.values()):
            if (
                not gate.crossed
                and gate.left_behind_stamp is not None
                and gate.right_behind_stamp is not None
                and abs(gate.left_behind_stamp - gate.right_behind_stamp)
                <= self.max_pair_crossing_gap_s
            ):
                gate.crossed = True

        # Çok eski aktif track kimlikleri serbest kalır. Crossed gate'ler ayrı
        # geçmişe taşınır; drop/reappear aynı explicit ikiliyi ikinci kez saymaz.
        self._expire_stale(now)

        active = sorted(
            (g for g in self.gates.values() if not g.crossed), key=lambda gate: gate.key
        )
        crossed = {key: gate for key, gate in self.crossed_history.items()}
        crossed.update({key: gate for key, gate in self.gates.items() if gate.crossed})
        crossed_keys = sorted(crossed)
        kd_estimate = len(self.seen_gate_keys)
        return PairCrossingStats(
            crossed_count=len(crossed_keys),
            kd_estimate=kd_estimate,
            ratio=len(crossed_keys) / max(1, kd_estimate),
            active_gates=active,
            crossed_keys=crossed_keys,
        )

    def _stats(self) -> PairCrossingStats:
        active = sorted(
            (gate for gate in self.gates.values() if not gate.crossed),
            key=lambda gate: gate.key,
        )
        crossed_keys = sorted(
            set(self.crossed_history)
            | {key for key, gate in self.gates.items() if gate.crossed}
        )
        kd = len(self.seen_gate_keys)
        return PairCrossingStats(
            crossed_count=len(crossed_keys),
            kd_estimate=kd,
            ratio=len(crossed_keys) / max(1, kd),
            active_gates=active,
            crossed_keys=crossed_keys,
        )

    def _expire_stale(self, now: float) -> None:
        for key, gate in list(self.gates.items()):
            if now - gate.last_seen_stamp <= self.max_age_s:
                continue
            self.gates.pop(key, None)
            if gate.crossed:
                self.crossed_history[key] = gate

    def _valid_pair(self, left: Dict[str, Any], right: Dict[str, Any]) -> bool:
        left_fwd = _finite_float(left.get("forward_m"))
        right_fwd = _finite_float(right.get("forward_m"))
        left_lat = _finite_float(left.get("lateral_m"))
        right_lat = _finite_float(right.get("lateral_m"))
        if None in (left_fwd, right_fwd, left_lat, right_lat):
            return False
        assert left_fwd is not None and right_fwd is not None
        assert left_lat is not None and right_lat is not None
        width = math.hypot(left_fwd - right_fwd, left_lat - right_lat)
        forward_aligned = abs(left_fwd - right_fwd) <= self.max_pair_forward_gap_m
        if self.allow_sim_truth_geometry and _matching_sim_truth_ids(left, right):
            # Ayrı sim-only topicte stable pair ID fiziksel eşleşmenin
            # otoritesidir. Heading offset, aynı gate'in iki dubasını body
            # forward ekseninde ayırabilir; production default bu yolu açmaz.
            forward_aligned = True
        return (
            forward_aligned
            and self.min_lateral_gap_m <= width <= 2.0 * self.max_pair_lateral_m
            and abs(left_lat) <= self.max_pair_lateral_m
            and abs(right_lat) <= self.max_pair_lateral_m
        )

    def _new_pair_is_ahead(self, left: Dict[str, Any], right: Dict[str, Any]) -> bool:
        return min(
            float(left.get("forward_m", 0.0)), float(right.get("forward_m", 0.0))
        ) > self.cross_trigger_forward_m

    def _valid_side_observation(self, buoy: Dict[str, Any]) -> bool:
        forward = _finite_float(buoy.get("forward_m"))
        lateral = _finite_float(buoy.get("lateral_m"))
        return forward is not None and lateral is not None and abs(lateral) <= self.max_pair_lateral_m

    def _new_gate(
        self,
        key: str,
        left: Dict[str, Any],
        right: Dict[str, Any],
        now: float,
        explicit_ids: bool,
    ) -> PairGate:
        center_fwd, center_lat, width = _pair_geometry(left, right)
        self.seen_gate_keys.add(key)
        return PairGate(
            key=key,
            left_id=_buoy_key(left),
            right_id=_buoy_key(right),
            last_seen_stamp=now,
            explicit_ids=explicit_ids,
            center_forward_m=center_fwd,
            center_lateral_m=center_lat,
            width_m=width,
        )

    def _observe_gate(
        self,
        gate: PairGate,
        left: Optional[Dict[str, Any]],
        right: Optional[Dict[str, Any]],
        now: float,
    ) -> None:
        if (
            self.allow_sim_truth_geometry
            and left is not None
            and right is not None
            and _matching_sim_truth_ids(left, right)
            and 0.5 * (float(left["forward_m"]) + float(right["forward_m"]))
            < self.cross_complete_forward_m
        ):
            # Oblique heading'de stable gate'in bir ucu önde, diğeri arkada
            # kalabilir. Sim-only truth'te fiziksel geçiş gate merkezinin tekne
            # düzlemini geçmesidir; iki stamp aynı frame'e latch edilir.
            gate.left_behind = True
            gate.right_behind = True
            gate.left_behind_stamp = now
            gate.right_behind_stamp = now
        if left is not None:
            if float(left["forward_m"]) < self.cross_complete_forward_m:
                gate.left_behind = True
                gate.left_behind_stamp = now
        if right is not None:
            if float(right["forward_m"]) < self.cross_complete_forward_m:
                gate.right_behind = True
                gate.right_behind_stamp = now
        if left is not None and right is not None:
            gate.center_forward_m, gate.center_lateral_m, gate.width_m = _pair_geometry(
                left, right
            )
        if left is not None or right is not None:
            gate.last_seen_stamp = now

    def _minimum_cost_pairs(
        self,
        left: List[Dict[str, Any]], right: List[Dict[str, Any]]
    ) -> List[tuple[Dict[str, Any], Dict[str, Any]]]:
        # Girdi sırasından bağımsız canonical sıralama equal-cost tie'larını da
        # deterministik yapar.
        left = sorted(left, key=_detection_canonical_key)
        right = sorted(right, key=_detection_canonical_key)
        # Geçersiz kenarları matching'den önce ele. Ardından successive
        # shortest augmenting path ile maksimum kardinaliteli *ve* bu kardinalite
        # içinde minimum maliyetli bire-bir eşlemeyi bul. Basit greedy seçim,
        # L=[10,14], R=[13,15] gibi bir tabloda iki geçerli gate varken birini
        # tüketebiliyordu.
        candidate_costs: Dict[tuple[int, int], int] = {}
        for left_index, left_buoy in enumerate(left):
            for right_index, right_buoy in enumerate(right):
                if not self._valid_pair(left_buoy, right_buoy):
                    continue
                # Aynı kapının iki dubası yaklaşık aynı forward düzlemindedir;
                # ikincil terim merkez lateral simetrisini tercih eder.
                cost = abs(
                    float(left_buoy.get("forward_m", 0.0))
                    - float(right_buoy.get("forward_m", 0.0))
                ) + 0.25 * abs(
                    float(left_buoy.get("lateral_m", 0.0))
                    + float(right_buoy.get("lateral_m", 0.0))
                )
                candidate_costs[(left_index, right_index)] = int(round(cost * 1_000_000.0))

        source = 0
        left_base = 1
        right_base = left_base + len(left)
        sink = right_base + len(right)
        graph: List[List[List[int]]] = [[] for _ in range(sink + 1)]

        def add_edge(start: int, end: int, capacity: int, cost: int) -> List[int]:
            forward_edge = [end, len(graph[end]), capacity, cost]
            reverse_edge = [start, len(graph[start]), 0, -cost]
            graph[start].append(forward_edge)
            graph[end].append(reverse_edge)
            return forward_edge

        for left_index in range(len(left)):
            add_edge(source, left_base + left_index, 1, 0)
        matched_edges: Dict[tuple[int, int], List[int]] = {}
        for left_index, right_index in sorted(candidate_costs):
            matched_edges[(left_index, right_index)] = add_edge(
                left_base + left_index,
                right_base + right_index,
                1,
                candidate_costs[(left_index, right_index)],
            )
        for right_index in range(len(right)):
            add_edge(right_base + right_index, sink, 1, 0)

        # Residual graph negatif ters kenarlar içerdiğinden Bellman-Ford kullan.
        # Düğüm/kenar sırası canonical olduğu için eşit maliyetli sonuç da
        # input permütasyonundan bağımsızdır.
        node_count = sink + 1
        while True:
            distance: List[Optional[int]] = [None] * node_count
            previous: List[Optional[tuple[int, int]]] = [None] * node_count
            distance[source] = 0
            for _ in range(node_count - 1):
                changed = False
                for start in range(node_count):
                    if distance[start] is None:
                        continue
                    for edge_index, edge in enumerate(graph[start]):
                        end, _reverse, capacity, edge_cost = edge
                        if capacity <= 0:
                            continue
                        proposal = distance[start] + edge_cost
                        if distance[end] is None or proposal < distance[end]:
                            distance[end] = proposal
                            previous[end] = (start, edge_index)
                            changed = True
                if not changed:
                    break
            if distance[sink] is None:
                break
            node = sink
            while node != source:
                step = previous[node]
                if step is None:  # defensive: reachable sink must have a full path
                    raise RuntimeError("minimum-cost pair path is incomplete")
                start, edge_index = step
                edge = graph[start][edge_index]
                edge[2] -= 1
                graph[node][edge[1]][2] += 1
                node = start

        selected = [
            (left_index, right_index)
            for (left_index, right_index), edge in matched_edges.items()
            if edge[2] == 0
        ]
        return [(left[i], right[j]) for i, j in sorted(selected)]


def _find_buoy(buoys: List[Dict[str, Any]], key: str) -> Optional[Dict[str, Any]]:
    """Re-match a buoy by its stored key (id if present, else lateral slot)."""
    for b in buoys:
        if _buoy_key(b) == key:
            return b
    return None


_PAIR_ID_RE = re.compile(r"^(?P<prefix>.*?)(?:_)?(?P<side>[lr])(?P<number>\d+)$", re.IGNORECASE)


def _explicit_pair_identity(buoy: Dict[str, Any]) -> Optional[tuple[str, str]]:
    """Stable ID'den (kanonik gate key, left/right) çıkarır.

    ``p1_o_l7``/``p1_o_r7`` ve kısa ``l7``/``r7`` biçimleri desteklenir.
    Geometrik lateral işareti değil ID side'ı otoritedir; araç döndüğünde
    görüntüde taraf değiştirse dahi aynı fiziksel ikili korunur.
    """
    raw_id = buoy.get("id")
    if raw_id is None:
        return None
    match = _PAIR_ID_RE.match(str(raw_id).strip())
    if match is None:
        return None
    prefix = match.group("prefix").rstrip("_")
    number = int(match.group("number"))
    side = "left" if match.group("side").lower() == "l" else "right"
    return f"id:{prefix}:{number}", side


_COURSE_PAIR_NAMESPACE_RE = re.compile(r"^p\d+_o_[lr]", re.IGNORECASE)


def _malformed_course_pair_id(raw_id: Any) -> bool:
    if raw_id is None:
        return False
    text = str(raw_id).strip()
    return _COURSE_PAIR_NAMESPACE_RE.match(text) is not None and _PAIR_ID_RE.match(text) is None


def _matching_sim_truth_ids(left: Dict[str, Any], right: Dict[str, Any]) -> bool:
    if (
        left.get("source") != "sim_gate_truth"
        or right.get("source") != "sim_gate_truth"
    ):
        return False
    left_identity = _explicit_pair_identity(left)
    right_identity = _explicit_pair_identity(right)
    return bool(
        left_identity
        and right_identity
        and left_identity[0] == right_identity[0]
        and left_identity[1] == "left"
        and right_identity[1] == "right"
    )


def _pair_geometry(
    left: Dict[str, Any], right: Dict[str, Any]
) -> tuple[float, float, float]:
    left_fwd = float(left.get("forward_m", 0.0))
    right_fwd = float(right.get("forward_m", 0.0))
    left_lat = float(left.get("lateral_m", 0.0))
    right_lat = float(right.get("lateral_m", 0.0))
    return (
        0.5 * (left_fwd + right_fwd),
        0.5 * (left_lat + right_lat),
        math.hypot(left_fwd - right_fwd, left_lat - right_lat),
    )


def _finite_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _detection_canonical_key(buoy: Dict[str, Any]) -> tuple:
    return (
        float(buoy["forward_m"]),
        float(buoy["lateral_m"]),
        str(buoy.get("id", "")),
        -float(buoy["confidence"]),
    )


def _detection_rank(buoy: Dict[str, Any]) -> tuple:
    """Küçük rank tercih edilir: yüksek confidence, sonra canonical geometri."""
    return (-float(buoy["confidence"]),) + _detection_canonical_key(buoy)[:3]


def nearest_neighbor_route(
    start_lat: float, start_lon: float, waypoints: List[Dict[str, Any]]
) -> List[Dict[str, Any]]:
    """Greedy nearest-neighbor ordering of waypoints starting from the boat.

    Her adımda henüz ziyaret edilmemiş en yakın waypoint'e gidilir (haversine_m
    ile). Giriş listesi MUTASYONA UĞRATILMAZ: yalnızca sıralama değişir, her
    waypoint sözlüğü aynı kalır ve "parkur" alanı korunur (sıralama anahtarı
    değildir). Karmaşıklık O(N^2).

    Edge durumlar: boş liste -> []; tek waypoint -> kopya; tüm waypoint'ler aynı
    noktadaysa giriş sırası korunur; start bir waypoint ile aynı noktadaysa rota
    o waypoint'ten başlar.
    """
    if not waypoints:
        return []
    remaining = list(waypoints)
    route: List[Dict[str, Any]] = []
    cur_lat, cur_lon = start_lat, start_lon
    while remaining:
        nearest_idx = 0
        nearest_dist = haversine_m(cur_lat, cur_lon, remaining[0]["lat"], remaining[0]["lon"])
        for idx in range(1, len(remaining)):
            dist = haversine_m(cur_lat, cur_lon, remaining[idx]["lat"], remaining[idx]["lon"])
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = idx
        chosen = remaining.pop(nearest_idx)
        route.append(chosen)
        cur_lat, cur_lon = chosen["lat"], chosen["lon"]
    return route


def corridor_goal_body(
    waypoint_fwd: float,
    waypoint_lat: float,
    center_forward_m: float,
    center_lateral_m: float,
    confidence: float,
    use_corridor_goal: bool = True,
    lookahead_min_m: float = 8.0,
    lateral_gain: float = 0.3,
    max_lateral_m: float = 2.5,
) -> tuple:
    """Koridor aktif + conf >= 0.3 + use_corridor_goal -> (max(cf, lookahead_min), cl*lateral_gain);
    değilse waypoint gövde koordinatı (waypoint_fwd, waypoint_lat) döner.

    autonomy_node.handle_costmap_navigation koridor-hedef seçiminin ve eski
    test_saf.pl_corridor_goal_body helper'ının TEK KAYNAĞI (duplication
    kaldırıldı). ``use_corridor_goal=False`` P2 davranışıdır: koridor aktif
    olsa bile hedef her zaman waypoint olur (sarı engel üstüne hedef düşmez).
    Parametre adları eski helper ile aynıdır; varsayılanlar P1 davranışını
    (lookahead 8.0, yanal kazanç 0.3) korur.

    ``max_lateral_m``: hedef yanalını sınırlar — tek taraflı görüşte
    build_corridor_info fallback'i sadece görülen tarafı merkez sanar
    (hayalet merkez -5.5m); clamp aşırı sapmayı önler (araç koridoru asla
    terk etmez). Varsayılan 2.5m koridor genişliği için makul.
    """
    if not use_corridor_goal:
        return waypoint_fwd, waypoint_lat
    if float(confidence) < 0.3:
        return waypoint_fwd, waypoint_lat
    lookahead = max(float(center_forward_m), float(lookahead_min_m))
    lateral = float(center_lateral_m) * float(lateral_gain)
    # Hayalet merkez koruması: hedef yanalı sınırlı (aşırı sapma yok).
    lateral = max(-float(max_lateral_m), min(float(max_lateral_m), lateral))
    return lookahead, lateral


def waypoint_advance_decision(
    dist_to_current_m: float,
    threshold_m: float,
    fwd_current_m: float,
    fwd_next_m: float,
    overshoot_guard_m: float = 5.0,
) -> bool:
    """Aktif waypoint'e gerçekten ulaşıldı/geçildiğinde ``True`` döndürür.

    İki güvenli yol vardır:

    * Normal: mevcut WP eşik içinde ve sonraki WP teknenin önünde.
    * Sınırlı overshoot: mevcut WP artık arkada, fakat tekne hâlâ
      ``overshoot_guard_m`` yarıçapı içinde. Bu, keskin köşede eşiği bir tick
      kaçırınca kilitlenmeyi önler.

    ``fwd_next_m < 0`` tek başına hiçbir zaman geçiş sebebi değildir. Böylece
    rosbag'deki gibi 50 m uzaktaki WP2'nin, yalnız WP3 gövde arkasında göründü
    diye WP3 ve WP4'e zincirleme atlanması engellenir.
    """
    threshold = float(threshold_m)
    guard = float(overshoot_guard_m)
    if not math.isfinite(threshold) or threshold <= 0.0:
        raise ValueError("threshold_m sonlu ve > 0 olmalı")
    if not math.isfinite(guard) or guard < threshold:
        raise ValueError("overshoot_guard_m sonlu ve threshold_m'den küçük olmamalı")

    distance = float(dist_to_current_m)
    if not math.isfinite(distance) or distance < 0.0:
        return False
    normal_close = distance <= threshold and float(fwd_next_m) > 0.5
    bounded_current_overshoot = (
        threshold < distance <= guard and float(fwd_current_m) < 0.0
    )
    return normal_close or bounded_current_overshoot


def waypoint_guidance_index(
    current_index: int,
    total_waypoints: int,
    dist_to_current_m: float,
    threshold_m: float,
    advanced_this_tick: bool = False,
) -> int:
    """Select a stable guidance target without weakening advance safety.

    A waypoint that is already inside the arrival radius has an unstable body
    bearing: centimetres of motion can flip it from left to right.  When the
    strict advance guard intentionally waits for the *next* waypoint to become
    forward, guide the heading toward that next waypoint instead of pivoting
    around the nearly coincident current point.  The index is not advanced by
    this helper; ``waypoint_advance_decision`` remains the sole authority.
    """
    current = int(current_index)
    total = int(total_waypoints)
    if total <= 0 or current < 0 or current >= total:
        return current
    if advanced_this_tick or current >= total - 1:
        return current
    try:
        distance = float(dist_to_current_m)
        threshold = float(threshold_m)
    except (TypeError, ValueError, OverflowError):
        return current
    if not math.isfinite(distance) or distance < 0.0:
        return current
    if not math.isfinite(threshold) or threshold <= 0.0:
        return current
    return current + 1 if distance <= threshold else current


def is_last_waypoint_index(current_wp: int, total_waypoints: int) -> bool:
    """Son waypoint'e (index-1 tabanlı) ulaşılıp ulaşılmadığı.

    current_wp, bir sonraki hedef waypoint'in index'idir; dolayısıyla
    current_wp >= total - 1 ise geriye yalnızca son waypoint kalmıştır.
    Parkur alanı kontrolü burada YAPILMAZ: P2 tamamlama koşulu yalnızca
    rota pozisyonuna bağlıdır (tasarım kararı, test edilebilirlik için saf).
    """
    return current_wp >= max(0, total_waypoints - 1)


class YellowGateCounter:
    """Sarý dubalardan (sarı kapı) geçiş sayacı.

    PairCrossingDetector desenini izler: iç durum `counter` (float, 0.0'dan
    başlar). Sarı duba filtrelerden geçerse counter tavanlanarak artar
    (min(counter + 1, threshold)); hiç sarı görülmezse decay ile azalır
    (max(counter - decay, 0.0)). Dönüş değeri int(round(counter))'dır.

    Filtreler: color == "yellow", confidence >= min_confidence,
    distance <= range_m, forward_m > forward_m, abs(lateral_m) <= lateral_m.
    Alan yedekleri: distance -> 99.0, forward_m -> 0.0, lateral_m -> "y".
    `now` parametresi determinizm/imza uyumu içindir; bu sayaçta zaman
    kullanılmaz ama testler sabit `now` ile çağırır.
    """

    def __init__(
        self,
        threshold: float,
        decay: float,
        range_m: float,
        forward_m: float,
        lateral_m: float,
        min_confidence: float,
    ) -> None:
        self.threshold = threshold
        self.decay = decay
        self.range_m = range_m
        self.forward_m = forward_m
        self.lateral_m = lateral_m
        self.min_confidence = min_confidence
        self.counter: float = 0.0

    def reset(self) -> None:
        """Sayacı sıfırla; parkur geçişlerinde kullanılır."""
        self.counter = 0.0

    def update(self, buoys: List[Dict[str, Any]], now: float) -> int:
        seen = False
        for b in buoys:
            # lateral_m yedeği "y" (sentinel): eksikse float'a çevrilemez ve filtre
            # sessizce başarısız olur (exception fırlatılmaz).
            lateral_raw = b.get("lateral_m", "y")
            try:
                lateral_ok = abs(float(lateral_raw)) <= self.lateral_m
            except (TypeError, ValueError):
                lateral_ok = False
            if (
                str(b.get("color", "")).lower() == "yellow"
                and float(b.get("confidence", 0.0)) >= self.min_confidence
                and float(b.get("distance", 99.0)) <= self.range_m
                and float(b.get("forward_m", 0.0)) > self.forward_m
                and lateral_ok
            ):
                seen = True
                break
        if seen:
            self.counter = min(self.counter + 1.0, self.threshold)
        else:
            self.counter = max(self.counter - self.decay, 0.0)
        return int(round(self.counter))


def build_corridor_info(buoys: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Turuncu dubalardan aktif koridor bilgisi üretir (DWA corridor bias).

    autonomy_node._build_corridor'ın SAF versiyonu (aynı mantık) — simülasyon
    testlerinin gerçek kodu kullanabilmesi için. apply_corridor_bias filtresiyle
    aynı eşikler: turuncu, confidence >= 0.35, distance < 18.0. En az 2 duba
    görülüyorsa koridor aktif; ``center_lateral_m`` sol/sağ çiftlerin orta
    noktalarının ortalaması, ``confidence`` görülen sayıya göre. Yetersiz görüşte
    ``{"active": False}`` döner.
    """
    orange = [
        b
        for b in buoys
        if str(b.get("color", "")).lower() == "orange"
        and float(b.get("confidence", 0.0)) >= 0.35
        and float(b.get("distance", 99.0)) < 18.0
    ]
    if len(orange) < 2:
        return {"active": False}
    left_buoys: List[Dict[str, Any]] = []
    right_buoys: List[Dict[str, Any]] = []
    for b in orange:
        if "lateral_m" in b:
            lat = float(b["lateral_m"])
        else:
            distance = float(b.get("distance", 0.0))
            bearing = float(b.get("bearing_deg", 0.0))
            lat = distance * math.sin(math.radians(bearing))
        if "forward_m" in b:
            fwd = float(b["forward_m"])
        else:
            distance = float(b.get("distance", 0.0))
            bearing = float(b.get("bearing_deg", 0.0))
            fwd = distance * math.cos(math.radians(bearing))
        if lat >= 0.0:
            right_buoys.append({"fwd": fwd, "lat": lat})
        else:
            # SOL duba lateral'i NEGATİF kalmalı (abs yok!) — merkez hesabı
            # (right + left)/2 için. abs() asimetrik koridorda yanlış merkez
            # üretiyordu (implementation_plan Sorun 1 — sola sapma kök nedeni).
            left_buoys.append({"fwd": fwd, "lat": lat})
    centers_fwd: List[float] = []
    centers_lat: List[float] = []
    for l_b in sorted(left_buoys, key=lambda x: x["fwd"])[:2]:
        for r_b in sorted(right_buoys, key=lambda x: x["fwd"])[:2]:
            centers_fwd.append((l_b["fwd"] + r_b["fwd"]) / 2.0)
            # right(+) + left(-) = koridor merkezi (implementation_plan Sorun 1).
            centers_lat.append((r_b["lat"] + l_b["lat"]) / 2.0)
    if centers_fwd:
        center_forward = sum(centers_fwd) / max(1, len(centers_fwd))
        center_lateral = sum(centers_lat) / max(1, len(centers_lat))
        # Tek taraflı duba -> conf 0.5 (güvenilmez tahmin tam güvenle kullanılmasın).
        # İki tarafta >=2 duba -> 1.0 (implementation_plan Sorun 2).
        conf = clamp(min(len(left_buoys), len(right_buoys)) / 2.0, 0.0, 1.0)
    else:
        # TEK TARAFLI GÖRÜŞ (fallback): sadece bir tarafın dubaları görülüyor.
        # Görülen tarafın ortalamasını merkez sanmak HAYALET MERKEZ üretir
        # (debug: center_lateral 6m — fiziksel imkansız, araç koridoru terk ediyor).
        # Güvenli varsayım: merkez 0 (koridor ortası) — araç koridora yönelir,
        # corridor bias doğru çeker. conf düşük tutulur (güvenilmez görüş).
        center_forward = sum(b["fwd"] for b in left_buoys[:4] + right_buoys[:4]) / max(
            1, len(left_buoys[:4] + right_buoys[:4])
        ) if (left_buoys or right_buoys) else 6.0
        center_lateral = 0.0  # tek taraflı görüşte merkez koridor ortası varsayılır
        conf = 0.3
    return {
        "active": True,
        "center_lateral_m": center_lateral,
        "center_forward_m": center_forward,
        "confidence": conf,
    }


def build_route_corridor_info(
    telemetry: Dict[str, Any],
    route: List[Dict[str, Any]],
    *,
    lookahead_m: float = 8.0,
    hard_half_width_m: float = 4.0,
    confidence: float = 1.0,
) -> Dict[str, Any]:
    """Mission polyline'dan gövde-frame koridor kısıtı üretir.

    Kamera sınıflandırması P2 sınır dubalarını kaçırsa bile hakem GN4->GN5
    rotası değişmez. Bu yardımcı en yakın rota parçasını bulur, aracın imzalı
    çapraz sapmasını ve aday yörüngelerin rota çapraz eksenine izdüşüm
    katsayılarını döndürür. DWA hedefi yine mission waypoint'tir; bu veri yalnız
    koridor dışına doğru giden kaçınma adaylarını veto eder ve içeri dönen adayı
    ödüllendirir.
    """

    if not isinstance(telemetry, dict) or not isinstance(route, list) or len(route) < 2:
        return {"active": False, "source": "mission_route_invalid"}
    try:
        lat = float(telemetry["lat"])
        lon = float(telemetry["lon"])
        heading_deg = float(telemetry["heading_deg"])
        lookahead = float(lookahead_m)
        hard_half = float(hard_half_width_m)
        conf = float(confidence)
    except (KeyError, TypeError, ValueError, OverflowError):
        return {"active": False, "source": "mission_route_invalid"}
    if not all(math.isfinite(v) for v in (lat, lon, heading_deg, lookahead, hard_half, conf)):
        return {"active": False, "source": "mission_route_invalid"}
    if lookahead <= 0.0 or hard_half <= 0.0 or not 0.0 <= conf <= 1.0:
        return {"active": False, "source": "mission_route_invalid"}

    try:
        origin_lat = float(route[0]["lat"])
        origin_lon = float(route[0]["lon"])
        points = [
            latlon_to_local_m(origin_lat, origin_lon, float(wp["lat"]), float(wp["lon"]))
            for wp in route
        ]
    except (KeyError, TypeError, ValueError, OverflowError):
        return {"active": False, "source": "mission_route_invalid"}
    current = latlon_to_local_m(origin_lat, origin_lon, lat, lon)

    best = None
    for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
        dn = end.x - start.x
        de = end.y - start.y
        length = math.hypot(dn, de)
        if length <= 1e-6:
            continue
        un, ue = dn / length, de / length
        rn, re = current.x - start.x, current.y - start.y
        along_unclamped = rn * un + re * ue
        along = clamp(along_unclamped, 0.0, length)
        proj_n = start.x + along * un
        proj_e = start.y + along * ue
        distance = math.hypot(current.x - proj_n, current.y - proj_e)
        candidate = (distance, index, start, length, un, ue, along)
        if best is None or candidate[:2] < best[:2]:
            best = candidate
    if best is None:
        return {"active": False, "source": "mission_route_invalid"}

    _, index, start, length, un, ue, along = best
    target_along = min(length, along + lookahead)
    target_n = start.x + target_along * un
    target_e = start.y + target_along * ue
    delta_n = target_n - current.x
    delta_e = target_e - current.y
    heading = math.radians(heading_deg)
    center_forward = math.cos(heading) * delta_n + math.sin(heading) * delta_e
    center_right = -math.sin(heading) * delta_n + math.cos(heading) * delta_e

    # İmzalı rota sapması. Pozitif/negatif yalnız tarafı belirtir; DWA mutlak
    # güvenlik zarfını uygular ve daima |sapma| azaltan adaya izin verir.
    current_cross = (current.x - start.x) * ue - (current.y - start.y) * un
    # Body (forward,right) hareketini aynı cross-track eksenine dönüştürür.
    cross_forward_coeff = math.cos(heading) * ue - math.sin(heading) * un
    cross_right_coeff = -math.sin(heading) * ue - math.cos(heading) * un
    return {
        "active": True,
        "source": "mission_route",
        "confidence": conf,
        "center_forward_m": center_forward,
        "center_lateral_m": center_right,
        "current_cross_track_m": current_cross,
        "cross_forward_coeff": cross_forward_coeff,
        "cross_right_coeff": cross_right_coeff,
        "hard_half_width_m": hard_half,
        "route_segment_index": index,
    }
