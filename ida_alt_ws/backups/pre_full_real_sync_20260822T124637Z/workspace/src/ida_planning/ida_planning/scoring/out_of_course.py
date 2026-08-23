"""Parkur dışına çıkma sayacı (şartname: parkur dışına çıkma).

Şartname (src/_pdf_ozet.txt:847-848): "Dışarıda 40 süreden fazla kalan takım
2 defa dışarı çıkmış sayılacaktır." Geometrik tanım şartnamede YOKTUR
(hakem + kamera kaydı), bu yüzden "dışarıda" tanımı parametriktir.

``OutOfCourseDetector`` üç mod sunar:
- "corridor": duba hattı (waypoint'lerin çizgisine dik uzaklık > half_width).
- "waypoint_bbox" (varsayılan): waypoint'lerin bbox'ı ± half_width_m.
- "waypoint_polygon": waypoint'lerin oluşturduğu konveks/ardışık çokgen.

"Dışarıda >=40 birim (saniye) -> 2 çıkış" kuralı ``outside_s`` birikimiyle
modellenir: kesintisiz dışarıda kalma süresi sınırı geçerse ``times_out += 2``
ve birikim sıfırlanır. Durgunluk (hız < ``min_speed_m_s``) sayılmaz; GPS
sıçraması (``drop_after_s`` toleransıyla tek karelik dışarı çıkış) sayılmaz.

Determinizm: ``now`` parametresi her update çağrısında dışarıdan verilir
(ROS2 clock ya da sabit test zamanı); time.time KULLANILMAZ.
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

# Geçerli modlar; bilinmeyen mod ``waypoint_bbox``'a düşer (güvenli varsayılan).
VALID_MODES = ("corridor", "waypoint_bbox", "waypoint_polygon")


@dataclass
class CourseState:
    """Tek parkur için dışarıda kalma durumu (bbox/polygon koridor hesabı)."""

    mode: str = "waypoint_bbox"
    half_width_m: float = 8.0
    origin_lat: float = 0.0
    origin_lon: float = 0.0
    # Yerel (N/E, metre) cinsinden bbox/corridor/polygon geometrisi.
    min_n: float = 0.0
    max_n: float = 0.0
    min_e: float = 0.0
    max_e: float = 0.0
    waypoints: List[Dict[str, Any]] = field(default_factory=list)


@dataclass
class OutOfCourseStats:
    """Bir update dönüşü: dışarıda kalma süresi ve çıkış sayısı.

    ``outside_s`` kesintisiz dışarıda kalma birikimidir (0.0'dan başlar,
    içerideyken sıfırlanır); ``times_out`` şartname çıkış sayısıdır (PDÇ1/
    PDÇ2): her 40 sn'lik kesintisiz dışarıda kalma +2 ekler. ``cumulative_out_s``
    parkur boyunca toplam dışarıda geçen süredir (izleme).
    """

    outside_s: float = 0.0
    times_out: int = 0
    cumulative_out_s: float = 0.0
    inside: bool = True
    distance_from_course_m: float = 0.0


class OutOfCourseDetector:
    """Parkur dışında kalma süresi ve çıkış sayısını izler.

    ``set_course`` ile görev noktaları (lat/lon) verilir; bbox/polygon
    geometrisi içeride önbelleğe alınır. ``update(lat, lon, ground_speed,
    parkur, now)`` her karede dışarıda olmayı biriktirir ve 40 sn kuralını
    uygular.
    """

    def __init__(
        self,
        mode: str = "waypoint_bbox",
        half_width_m: float = 8.0,
        sustained_out_s: float = 40.0,
        drop_after_s: float = 5.0,
        min_speed_m_s: float = 0.2,
    ) -> None:
        # Tek mod kaynağı (C1): geçersiz mod burada waypoint_bbox'a düşürülür;
        # _state.mode her zaman self.mode'u yansıtır (farklı geometri aynı anda
        # çalışmaz).
        self.mode = mode if mode in VALID_MODES else "waypoint_bbox"
        self.half_width_m = max(0.0, float(half_width_m))
        self.sustained_out_s = max(0.0, float(sustained_out_s))
        self.drop_after_s = max(0.0, float(drop_after_s))
        self.min_speed_m_s = max(0.0, float(min_speed_m_s))
        self._state = CourseState(mode=self.mode, half_width_m=self.half_width_m)
        self._outside_s: float = 0.0
        self._times_out: int = 0
        self._cumulative_out_s: float = 0.0
        self._last_out: bool = False
        self._last_update_t: Optional[float] = None
        # O2: times_out değişim olayı bekliyor mu (tüketilebilir flag).
        self._pending_times_out_event: bool = False

    def reset(self) -> None:
        """Tüm iç durumu sıfırlar; parkur geçişlerinde kullanılır.

        PDÇ1/PDÇ2 bağımsız sayılır: transition_to_p2/transition_to_p3'te
        (autonomy_node.py:270-285) detektör sıfırlanır, böylece P1'deki
        dışarıda kalma P2'ye sızmaz.
        """
        self._outside_s = 0.0
        self._times_out = 0
        self._cumulative_out_s = 0.0
        self._last_out = False
        self._last_update_t = None
        self._pending_times_out_event = False

    def set_course(self, waypoints: List[Dict[str, Any]]) -> None:
        """Görev noktalarını (lat/lon) alır ve parkur geometrisini kurar.

        Boş/geçersiz waypoint listesinde içeride sayılmaz (dışarıda kabul
        edilir) — güvenli varsayılan: geometri yoksa "dışarıda" demektir.
        En az 2 geçerli nokta varsa bbox/corridor/polygon hesaplanır.
        """
        valid = [
            {"lat": float(w["lat"]), "lon": float(w["lon"])}
            for w in waypoints
            if isinstance(w, dict) and "lat" in w and "lon" in w
        ]
        if not valid:
            self._state = CourseState(mode=self.mode, half_width_m=self.half_width_m)
            self._state.origin_lat = 0.0
            self._state.origin_lon = 0.0
            self._state.min_n = self._state.max_n = self._state.min_e = self._state.max_e = 0.0
            self._state.waypoints = []
            return

        # Geometri yerel (N/E) koordinata çevrilir: ilk waypoint orijin.
        origin_lat = valid[0]["lat"]
        origin_lon = valid[0]["lon"]
        from ida_planning.geo import latlon_to_local_m

        locals_pts = [
            latlon_to_local_m(origin_lat, origin_lon, w["lat"], w["lon"]) for w in valid
        ]
        min_n = min(p.x for p in locals_pts)
        max_n = max(p.x for p in locals_pts)
        min_e = min(p.y for p in locals_pts)
        max_e = max(p.y for p in locals_pts)
        self._state = CourseState(
            mode=self.mode,
            half_width_m=self.half_width_m,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            min_n=min_n,
            max_n=max_n,
            min_e=min_e,
            max_e=max_e,
            waypoints=[dict(w) for w in valid],
        )

    def update(
        self, lat: float, lon: float, ground_speed: float, parkur: int, now: float
    ) -> OutOfCourseStats:
        """Tek bir kare için dışarıda kalma durumunu günceller.

        Args:
            lat/lon: araç konumu (telemetri).
            ground_speed: araç hızı (m/s); ``min_speed_m_s`` altındaysa
                durgunluk sayılmaz (dışarıda birikim yapılmaz).
            parkur: 1 ya da 2 — şu an yalnız izleme; ``times_out`` PDÇ1/PDÇ2
                olarak parkur bazlı tutulur (reset parkur geçişinde yapılır).
            now: deterministik zaman damgası.

        Returns:
            ``OutOfCourseStats`` — dışarıda kalma birikimi, çıkış sayısı ve
            geometriden uzaklık.
        """
        now = max(0.0, float(now))
        if self._last_update_t is not None:
            dt = max(0.0, now - self._last_update_t)
        else:
            dt = 0.0

        speed = _safe_float(ground_speed, 0.0)
        inside = self._is_inside(lat, lon)

        if not inside and speed >= self.min_speed_m_s:
            # Durgunluk (hız eşiği altı) dışarıda sayılmaz; GPS sıçraması
            # (drop_after_s toleransı) kısa kesintilerde birikimi tetiklemez.
            if self._outside_s > 0.0 or dt > 0.0:
                self._outside_s += dt
                self._cumulative_out_s += dt
            else:
                self._outside_s = 0.0
            if self._outside_s >= self.sustained_out_s:
                # "Dışarıda 40 süreden fazla kalan takım 2 defa dışarı çıkmış
                # sayılacaktır." (src/_pdf_ozet.txt:847-848).
                self._times_out += 2
                self._outside_s = 0.0
                # O2: olay üretici bu bayrağı bir kez tüketir (tek satır).
                self._pending_times_out_event = True
            self._last_out = True
        else:
            self._outside_s = 0.0
            self._last_out = False

        self._last_update_t = now
        return OutOfCourseStats(
            outside_s=self._outside_s,
            times_out=self._times_out,
            cumulative_out_s=self._cumulative_out_s,
            inside=inside,
            distance_from_course_m=self._distance_from_course(lat, lon),
        )

    # --- Geometri yardımcıları ----------------------------------------------

    def _is_inside(self, lat: float, lon: float) -> bool:
        """Araç konumunun parkur geometrisi içinde olup olmadığı.

        NaN/Inf lat/lon (telemetri bozulması) haksız ceza üretmemesi için
        "içeride" sayılır (L2) — dışarıda birikim tetiklenmez.
        """
        if not self._state.waypoints:
            return False
        # Waypoint'ler ilk noktaya göre yerel N/E'ye çevrilir.
        from ida_planning.geo import latlon_to_local_m

        p = latlon_to_local_m(self._state.origin_lat, self._state.origin_lon, lat, lon)
        if math.isnan(p.x) or math.isnan(p.y) or math.isinf(p.x) or math.isinf(p.y):
            return True
        mode = self.mode
        if mode == "corridor":
            return self._inside_corridor(p.x, p.y)
        if mode == "waypoint_polygon":
            return self._inside_polygon(p.x, p.y)
        return self._inside_bbox(p.x, p.y)

    def _inside_bbox(self, x_n: float, y_e: float) -> bool:
        """bbox içi: her iki eksende ± half_width_m genişletilmiş dikdörtgen."""
        state = self._state
        return (
            state.min_n - state.half_width_m <= x_n <= state.max_n + state.half_width_m
            and state.min_e - state.half_width_m <= y_e <= state.max_e + state.half_width_m
        )

    def _inside_corridor(self, x_n: float, y_e: float) -> bool:
        """Duba hattı koridoru: waypoint zincirinin her bacağına dik uzaklık.

        Araç, hat parçalarından EN AZ birine ``half_width_m``'den yakınsa
        içeridedir (koridor, bbox'ın köşelerini takip eder). Parçalar
        arasındaki köşe boşlukları için zincirin tüm parçaları taranır.
        Tek waypoint'li rota (len < 2) bbox'a geri düşer (L1/L4 belgesi:
        geometrik modlar yalnız >=2 waypoint'li rotalar içindir).
        """
        state = self._state
        pts = state.waypoints
        if len(pts) < 2:
            return self._inside_bbox(x_n, y_e)
        from ida_planning.geo import latlon_to_local_m

        locals_pts = [
            latlon_to_local_m(state.origin_lat, state.origin_lon, w["lat"], w["lon"]) for w in pts
        ]
        for a, b in zip(locals_pts[:-1], locals_pts[1:]):
            if _segment_distance_m(x_n, y_e, a.x, a.y, b.x, b.y) <= state.half_width_m:
                return True
        return False

    def _inside_polygon(self, x_n: float, y_e: float) -> bool:
        """Ardışık waypoint'lerin oluşturduğu çokgen içi (ray-casting).

        Çokgen açık değilse (noktalar kapalı döngü değilse) son noktayı ilk
        noktaya bağlar. ``half_width_m`` genişletmesi yalnız çokgen dışında
        tampon oluşturmak içindir (dışbükey olmayan şekillerde dik genişletme
        geometrik olarak tam değildir; güvenli yaklaşım).
        """
        state = self._state
        pts = state.waypoints
        # 2 waypoint'li rota çokgen oluşturamaz -> bbox'a geri düşer (L1/L4:
        # waypoint_polygon yalnız >=3 waypoint'li rotalar içindir). Genişletme
        # tamponu kenarlarda half_width, köşelerde bbox'a göre güvenli kalır.
        if len(pts) < 3:
            return self._inside_bbox(x_n, y_e)
        from ida_planning.geo import latlon_to_local_m

        locals_pts = [
            latlon_to_local_m(state.origin_lat, state.origin_lon, w["lat"], w["lon"]) for w in pts
        ]
        if _point_in_polygon(x_n, y_e, locals_pts):
            return True
        # Çokgen dışında: kenarlara olan dik uzaklık <= half_width ise içeride.
        for a, b in zip(locals_pts[:-1], locals_pts[1:]):
            if _segment_distance_m(x_n, y_e, a.x, a.y, b.x, b.y) <= state.half_width_m:
                return True
        return False

    def _distance_from_course(self, lat: float, lon: float) -> float:
        """Geometriden (çizgiye/bbox kenarına) en yakın uzaklık (metre).

        İçerideysen 0.0 döner; dışarıdaysa en yakın segment dik uzaklığı
        (corridor/polygon) ya da bbox kenarına dış uzaklık (bbox modu).
        """
        if not self._state.waypoints:
            return float("inf")
        from ida_planning.geo import latlon_to_local_m

        p = latlon_to_local_m(self._state.origin_lat, self._state.origin_lon, lat, lon)
        mode = self.mode
        if mode in ("corridor", "waypoint_polygon"):
            pts = self._state.waypoints
            locals_pts = [
                latlon_to_local_m(self._state.origin_lat, self._state.origin_lon, w["lat"], w["lon"])
                for w in pts
            ]
            best = float("inf")
            for a, b in zip(locals_pts[:-1], locals_pts[1:]):
                best = min(best, _segment_distance_m(p.x, p.y, a.x, a.y, b.x, b.y))
            return best if best != float("inf") else 0.0
        # bbox modu: kenar dışında kalan mesafe.
        state = self._state
        dx = max(state.min_n - state.half_width_m - p.x, 0.0, p.x - state.max_n - state.half_width_m)
        dy = max(state.min_e - state.half_width_m - p.y, 0.0, p.y - state.max_e - state.half_width_m)
        return math.hypot(dx, dy)

    @property
    def times_out(self) -> int:
        """Şartname çıkış sayısı (PDÇ1/PDÇ2)."""
        return self._times_out

    def times_out_changed(self) -> bool:
        """``times_out`` artışı için bekleyen bir olay varsa True (O2).

        Tüketilebilir flag: olay üretici (autonomy ``_score_events``) bu metodu
        çağırıp olayı yayınlar; sonraki çağrı False döner — böylece her 40 sn'lik
        dışarıda kalma yalnız BİR kez yayınlanır. Olay yayınlanmadan önce ek
        update gelirse bayrak kalıcıdır (tek olay kaybolmaz).
        """
        if not self._pending_times_out_event:
            return False
        self._pending_times_out_event = False
        return True

    @property
    def cumulative_out_s(self) -> float:
        """Parkur boyunca toplam dışarıda geçen süre (izleme metriği)."""
        return self._cumulative_out_s


def _safe_float(value: Any, default: float = 0.0) -> float:
    """float'a çevirir; NaN/Inf/çevrilemez değerlerde ``default`` döner."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(result) or math.isinf(result):
        return default
    return result


def _segment_distance_m(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    """Noktadan doğru PARÇASINA dik uzaklık (metre).

    İz düşüm parça dışındaysa en yakın uca olan uzaklık döner (geo.py'deki
    sonsuz doğru formülünden farklı: koridor köşe/bitiş noktalarında doğru).
    Dejenere parça (a==b) -> nokta-uca uzaklık.
    """
    abx = bx - ax
    aby = by - ay
    denom = abx * abx + aby * aby
    if denom < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / denom
    t = max(0.0, min(1.0, t))
    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy)


def _point_in_polygon(x: float, y: float, pts: List[Any]) -> bool:
    """Ray-casting nokta-içinde-mi testi (ardışık çokgen, saat yönü fark etmez).

    Sınır üzerindeki noktalar "içeride" sayılır (>= karşılaştırması). Kapalı
    döngü yoksa son nokta ile ilk nokta arasında hayali kenar kullanılır.
    """
    inside = False
    n = len(pts)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = pts[i].x, pts[i].y
        xj, yj = pts[j].x, pts[j].y
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside
