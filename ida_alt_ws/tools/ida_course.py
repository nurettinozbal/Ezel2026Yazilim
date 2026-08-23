"""ida_course — TEKNOFEST İDA parkur şeması tanımı ve üretici.

Varsayılan yerleşim (hepsi parametrik, ``--config`` ile değiştirilebilir):

- P1 "geniş N" (zigzag sağa açılan): 4 köşe gn1=(10,0) -> gn2=(35,30) ->
  gn3=(15,60) -> gn4=(45,90). Koridor genişliği 10 m sabit (duba çiftleri
  arası), duba çifti aralığı 5-8 m değişken (deterministik rastgele).
  Her çift iki turuncu duba: koridor doğrultusuna dik, koridorun sağı/solu.
  İlk çift gn1'de başlar; köşelerde (gn2/gn3/gn4) gelen ve giden kenarın
  çiftleri arasında geçiş çifti bulunur (çiftler kesintisiz, sürekli
  koridor — tek çift dizisi üzerinde yürüyerek üretilir).
- P2: gn4'ten gn5=(45,140)'a düz doğu koridoru (turuncu çiftler devam,
  genişlik 10 m, aralık 5-8 m) + içine 10 sarı engel dubası (koridor
  içinde, deterministik; birbirine ve turuncu dubalara 2 m'den yakın değil).
- P3: gn5 sonrası turuncu/sarı YOK; 3 hedef büyük duba (kırmızı/siyah/yeşil)
  gn5'ten ~25 m ötede, üçgen dağınık (birbirine ~6-8 m).

Koordinat sistemi stack kontratıyla birebir: x = kuzey, y = doğu.
Tüm rastgelelik ``random.Random(seed)`` üzerinden (yeniden üretilebilir,
zaman kullanılmaz). Bu modül SAF'tır: dosya yazmaz, yalnız şema üretir.
"""

import math
import random
from dataclasses import dataclass, field
from typing import Any, Dict, List

# Varsayılan şema parametreleri (--config JSON'uyla ezilebilir).
DEFAULT_CONFIG: Dict[str, Any] = {
    "origin": {"lat": 40.8630501, "lon": 29.2599517},
    "corridor_width_m": 10.0,          # duba çiftleri arası (kenar kenara)
    "pair_min_gap_m": 5.0,             # çift aralığı alt sınırı (çift merkezleri)
    "pair_max_gap_m": 8.0,             # çift aralığı üst sınırı
    "p1_corners": [                    # N-şekil köşeleri (x_north, y_east)
        [10.0, 0.0],
        [35.0, 30.0],
        [15.0, 60.0],
        [45.0, 90.0],
    ],
    "p2_end": [45.0, 140.0],           # gn5 — P2 bitiş / P3 başlangıç
    "p2_yellow_count": 10,             # sarı engel duba sayısı
    "yellow_min_gap_m": 2.0,           # sarı-sarı ve sarı-turuncu min mesafe
    "yellow_wp_gap_m": 8.0,            # sarı-waypoint min mesafe (gn5'e yakın sarı konmaz)
    "p3_center_offset_m": 25.0,        # hedef merkezi gn5'ten ileri
    "p3_spread_m": 4.5,                # hedef merkezi yarıçapı (üçgen köşeleri)
    "yellow_lateral_limit": 3.6,       # sarı ofset üst sınırı (5 m koridorun içi)
}


def _require_finite(name: str, value: Any) -> float:
    """Config sayısını doğrular: NaN/Inf/çevrilemez -> ValueError.

    Şema geometrisi float sınırlarına güvenemez (deterministik çıktı bozulur);
    hatalı config üretimden ÖNCE yakalanır.
    """
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Config '{name}' sayı olmalı (aldık: {value!r})") from exc
    if math.isnan(result) or math.isinf(result):
        raise ValueError(f"Config '{name}' sonlu olmalı (aldık: {value!r})")
    return result


@dataclass
class Segment:
    """Poligonun tek düz bacağı (ör. gn1->gn2). Yalnız geometri taşır."""

    start: List[float]           # [x_north, y_east]
    end: List[float]             # [x_north, y_east]
    cumulative_start: float      # gn1'den bu bacağın başına kadar yol (m)

    def unit(self) -> tuple[float, float]:
        """Segment birim vektörü (kuzey, doğu) — dejenere segmentte (0,0)."""
        return _unit_v(self.start, self.end)

    def normal(self) -> tuple[float, float]:
        """Koridor normali (sol): birim vektörü 90 derece sola döndürür."""
        ux, uy = self.unit()
        return -uy, ux

    def length_m(self) -> float:
        return math.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1])

    def point_at(self, t: float) -> List[float]:
        """Bacak üzerinde yerel t (0..length) konumu."""
        ux, uy = self.unit()
        return [self.start[0] + t * ux, self.start[1] + t * uy]


@dataclass
class Pair:
    """Tek duba çifti: ortak merkez + sol/sağ duba + yol konumu."""

    center: List[float]          # koridor orta hattı üzerinde
    left: List[float]            # normal yönünde (koridor solu)
    right: List[float]           # -normal yönünde (koridor sağı)
    t: float                     # gn1'den yol mesafesi (m)


@dataclass
class ScenarioData:
    """Generator çıktısı: full_mission.yaml ile birebir aynı şema."""

    origin: Dict[str, float]
    initial_pose: Dict[str, float]
    waypoints: List[Dict[str, Any]]
    buoys: List[Dict[str, Any]]
    obstacles: List[Dict[str, Any]]
    targets: List[Dict[str, Any]]


class CourseSchema:
    """Parametrik parkur şeması: köşeler + çift aralıkları -> duba noktaları.

    ``generate(seed)`` tüm parkuru deterministik olarak üretir: P1 N-şekil
    + P2 düz hat TEK sürekli çift dizisi (gn1'den gn5'e yürüyüş), P2 sarı
    engelleri, P3 hedef üçgeni. Sonuç ``ScenarioData`` dataclass'ıdır.
    """

    def __init__(self, config: Dict[str, Any] | None = None) -> None:
        self.config = dict(DEFAULT_CONFIG)
        if config:
            self.config.update(config)
        self.origin = self.config["origin"]
        self.corridor_width_m = _require_finite("corridor_width_m", self.config["corridor_width_m"])
        self.pair_min_gap_m = _require_finite("pair_min_gap_m", self.config["pair_min_gap_m"])
        self.pair_max_gap_m = _require_finite("pair_max_gap_m", self.config["pair_max_gap_m"])
        self.p2_yellow_count = int(self.config["p2_yellow_count"])
        self.yellow_min_gap_m = _require_finite("yellow_min_gap_m", self.config["yellow_min_gap_m"])
        self.yellow_wp_gap_m = _require_finite("yellow_wp_gap_m", self.config["yellow_wp_gap_m"])
        self.p3_center_offset_m = _require_finite("p3_center_offset_m", self.config["p3_center_offset_m"])
        self.p3_spread_m = _require_finite("p3_spread_m", self.config["p3_spread_m"])
        self.yellow_lateral_limit = _require_finite(
            "yellow_lateral_limit", self.config["yellow_lateral_limit"]
        )
        self.p1_corners = [list(c) for c in self.config["p1_corners"]]
        self.p2_end = list(self.config["p2_end"])
        # Geometrik tutarlılık: koridor genişliği pozitif, aralık sıralı.
        if self.corridor_width_m <= 0.0:
            raise ValueError("corridor_width_m > 0 olmalı")
        if self.pair_min_gap_m <= 0.0:
            raise ValueError("pair_min_gap_m > 0 olmalı")
        if self.pair_max_gap_m < self.pair_min_gap_m:
            raise ValueError("pair_max_gap_m >= pair_min_gap_m olmalı")
        if self.yellow_lateral_limit > self.corridor_width_m / 2.0:
            raise ValueError("yellow_lateral_limit koridor yarı genişliğini aşamaz")

    # --- Ana üretim -----------------------------------------------------------

    def generate(self, seed: int) -> ScenarioData:
        rng = random.Random(seed)
        return self._generate(rng)

    def _generate(self, rng: random.Random) -> ScenarioData:
        # 1) Yol poligonu: gn1..gn4 (P1) + gn5 (P2). Tek sürekli çift dizisi.
        path = self.p1_corners + [self.p2_end]
        segments = self._build_segments(path)
        total_p1_m = sum(s.length_m() for s in segments[:-1])
        total_m = sum(s.length_m() for s in segments)

        pairs = self._walk_pairs(segments, total_m, rng)

        # 2) Turuncu dubalar: çift dizisinden; P1/P2 ayrımı yol konumundan.
        orange: List[Dict[str, Any]] = []
        p1_index = 0
        p2_index = 0
        for pair in pairs:
            if pair.t <= total_p1_m:
                p1_index += 1
                parkur_label = f"p1_o_l{p1_index}"
                parkur_label_r = f"p1_o_r{p1_index}"
            else:
                p2_index += 1
                parkur_label = f"p2_o_l{p2_index}"
                parkur_label_r = f"p2_o_r{p2_index}"
            orange.append(
                {
                    "id": parkur_label,
                    "x": pair.left[0],
                    "y": pair.left[1],
                    "color": "orange",
                }
            )
            orange.append(
                {
                    "id": parkur_label_r,
                    "x": pair.right[0],
                    "y": pair.right[1],
                    "color": "orange",
                }
            )

        # 3) P2 sarı engelleri (P2 bacağında, turunculara/1birbirine 2m+).
        yellow = self._place_yellow(segments[-1], orange, rng)

        # 4) P3 hedefleri: gn5'ten ~25 m ötede üçgen dağınık.
        targets = self._place_targets(rng)

        waypoints = []
        for corner in self.p1_corners:
            waypoints.append({"x": corner[0], "y": corner[1], "parkur": 1})
        waypoints.append({"x": self.p2_end[0], "y": self.p2_end[1], "parkur": 2})

        return ScenarioData(
            origin=self.origin,
            initial_pose={
                "x": self.p1_corners[0][0],
                "y": self.p1_corners[0][1],
                "heading_deg": 0.0,
            },
            waypoints=waypoints,
            buoys=orange + yellow,
            obstacles=[],
            targets=targets,
        )

    # --- Poligon + çift yürüyüşü ------------------------------------------------

    def _build_segments(self, path: List[List[float]]) -> List[Segment]:
        """Yol poligonunu segmentlere böler (kümülatif yol konumuyla)."""
        segments: List[Segment] = []
        cum = 0.0
        for a, b in zip(path[:-1], path[1:]):
            segments.append(Segment(start=list(a), end=list(b), cumulative_start=cum))
            cum += math.hypot(b[0] - a[0], b[1] - a[1])
        return segments

    def _point_and_dir(self, segments: List[Segment], t: float) -> tuple[List[float], Segment]:
        """Yol konumu t -> (nokta, aktif segment). t yol sonunu aşarsa son segment."""
        for seg in segments:
            if t < seg.cumulative_start + seg.length_m() + 1e-9:
                return seg.point_at(t - seg.cumulative_start), seg
        last = segments[-1]
        return list(last.end), last

    def _walk_pairs(
        self, segments: List[Segment], total_m: float, rng: random.Random
    ) -> List[Pair]:
        """gn1'den başlayarak tüm yol boyunca sürekli çift dizisi yürür.

        Her düz bacağın iki ucu zorunlu istasyondur. Aradaki mesafe,
        deterministik rastgele aralıklarla ``pair_min_gap_m`` ..
        ``pair_max_gap_m`` arasında paylaştırılır. Böylece bir köşeyi çevreleyen
        iki çift arasındaki açıklık da normal bir koridor açıklığıdır; köşede
        eski uygulamadaki gibi geniş, işaretsiz bir boşluk oluşmaz.

        Düz istasyonlarda çift aktif bacağa diktir. İç köşelerde ise gelen ve
        giden birim teğetlerin açıortayına dik yerleştirilir. Aynı köşe için tek
        ortak çift kullanıldığı için iki bacağın sınırları dip dibe doğmaz ve
        sol/sağ sınırların sırası dönüş boyunca korunur.
        """
        del total_m  # toplam, segmentlerden yeniden ve kesin olarak türetilir
        if not segments:
            return []

        # Bazı segment uzunluğu/seed birleşimleri (örn. 20 m'nin 4x5 m
        # bölünmesi) iç dirsekte aynı fiziksel noktayı iki kez üretebilir.
        # Geometriyi sonradan sessizce silmek aralık kontratını bozar. Bunun
        # yerine aynı RNG akışıyla yeni bir geçerli dağılım aranır; bulunamazsa
        # config açıkça reddedilir.
        last_reason = ""
        for _attempt in range(256):
            stations: List[tuple[float, List[float], int, bool]] = []
            for seg_idx, seg in enumerate(segments):
                length = seg.length_m()
                gaps = self._segment_pair_gaps(length, rng)
                local_t = 0.0
                if seg_idx == 0:
                    stations.append((seg.cumulative_start, list(seg.start), seg_idx, False))
                for gap_idx, gap in enumerate(gaps):
                    local_t += gap
                    is_end = gap_idx == len(gaps) - 1
                    center = list(seg.end) if is_end else seg.point_at(local_t)
                    stations.append(
                        (seg.cumulative_start + local_t, center, seg_idx, is_end)
                    )

            pairs: List[Pair] = []
            for t, center, seg_idx, is_segment_end in stations:
                if is_segment_end and seg_idx + 1 < len(segments):
                    nx, ny = self._corner_normal(
                        segments[seg_idx], segments[seg_idx + 1]
                    )
                else:
                    nx, ny = segments[seg_idx].normal()
                pairs.append(
                    Pair(
                        center=center,
                        left=[
                            center[0] + nx * self.half_width,
                            center[1] + ny * self.half_width,
                        ],
                        right=[
                            center[0] - nx * self.half_width,
                            center[1] - ny * self.half_width,
                        ],
                        t=t,
                    )
                )
            last_reason = self._pair_geometry_error(pairs)
            if not last_reason:
                return pairs
        raise ValueError(
            "Parkur genişliği ve dönüş açılarıyla kesişmeyen duba geometrisi "
            f"üretilemedi: {last_reason}"
        )

    def _segment_pair_gaps(self, length_m: float, rng: random.Random) -> List[float]:
        """Bir bacak uzunluğunu geçerli ve deterministik çift aralıklarına böler."""
        if length_m < self.pair_min_gap_m - 1e-9:
            raise ValueError(
                "Her parkur bacağı pair_min_gap_m kadar uzun olmalı "
                f"(aldık: {length_m:.3f} m)"
            )

        min_count = max(1, math.ceil((length_m - 1e-9) / self.pair_max_gap_m))
        max_count = math.floor((length_m + 1e-9) / self.pair_min_gap_m)
        if min_count > max_count:
            raise ValueError(
                f"{length_m:.3f} m bacak {self.pair_min_gap_m:.3f}.."
                f"{self.pair_max_gap_m:.3f} m aralıklara ayrılamıyor"
            )

        desired_gap = _advance(rng, self.pair_min_gap_m, self.pair_max_gap_m)
        count = min(max(round(length_m / desired_gap), min_count), max_count)
        capacity = self.pair_max_gap_m - self.pair_min_gap_m
        remaining = length_m - count * self.pair_min_gap_m
        gaps: List[float] = []
        for index in range(count):
            slots_after = count - index - 1
            low = max(0.0, remaining - slots_after * capacity)
            high = min(capacity, remaining)
            extra = remaining if slots_after == 0 else rng.uniform(low, high)
            gaps.append(self.pair_min_gap_m + extra)
            remaining -= extra

        # Tam toplamı garanti et; tolerans dışına çıkmak mantık hatasıdır.
        gaps[-1] += length_m - sum(gaps)
        if any(
            gap < self.pair_min_gap_m - 1e-7 or gap > self.pair_max_gap_m + 1e-7
            for gap in gaps
        ):
            raise RuntimeError("Üretilen duba çift aralığı config sınırları dışında")
        return gaps

    @staticmethod
    def _corner_normal(incoming: Segment, outgoing: Segment) -> tuple[float, float]:
        """Köşe çiftinin sol normalini gelen/giden teğet açıortayından üretir."""
        in_x, in_y = incoming.unit()
        out_x, out_y = outgoing.unit()
        tangent_x = in_x + out_x
        tangent_y = in_y + out_y
        norm = math.hypot(tangent_x, tangent_y)
        if norm < 1e-9:
            raise ValueError("Parkur 180 derece geri dönen bir köşe içeremez")
        tangent_x /= norm
        tangent_y /= norm
        return -tangent_y, tangent_x

    @staticmethod
    def _pair_geometry_error(pairs: List[Pair], min_buoy_gap_m: float = 2.0) -> str:
        """Boş string geçerli geometri; aksi halde deterministik hata açıklaması."""
        points = [point for pair in pairs for point in (pair.left, pair.right)]
        for i, point in enumerate(points):
            for other in points[i + 1 :]:
                if math.dist(point, other) < min_buoy_gap_m - 1e-7:
                    return "iki duba dip dibe veya aynı konumda"

        left_edges = [(a.left, b.left) for a, b in zip(pairs, pairs[1:])]
        right_edges = [(a.right, b.right) for a, b in zip(pairs, pairs[1:])]
        boundary_edges = [("left", i, *edge) for i, edge in enumerate(left_edges)]
        boundary_edges += [("right", i, *edge) for i, edge in enumerate(right_edges)]

        for edge_i, (side_a, index_a, a, b) in enumerate(boundary_edges):
            for side_b, index_b, c, d in boundary_edges[edge_i + 1 :]:
                if side_a == side_b and abs(index_a - index_b) <= 1:
                    continue
                if _segments_intersect(a, b, c, d):
                    return "koridor sınırları kesişiyor"

        gates = [(pair.left, pair.right) for pair in pairs]
        for gate_i, (a, b) in enumerate(gates):
            for c, d in gates[gate_i + 1 :]:
                if _segments_intersect(a, b, c, d):
                    return "duba çift çizgileri kesişiyor"
            for _side, edge_i, c, d in boundary_edges:
                if edge_i in (gate_i - 1, gate_i):
                    continue
                if _segments_intersect(a, b, c, d):
                    return "duba çifti koridor sınırını kesiyor"
        return ""

    @property
    def half_width(self) -> float:
        return self.corridor_width_m / 2.0

    # --- Sarı engel yerleşimi ---------------------------------------------------

    def _place_yellow(
        self,
        p2_seg: Segment,
        orange: List[Dict[str, Any]],
        rng: random.Random,
    ) -> List[Dict[str, Any]]:
        """P2 koridoruna istenen sayıda, açıklık kontratına uyan sarı yerleştirir.

        Çakışan aday artık sessizce atlanıp eksik senaryo üretmez. Sınırlı,
        deterministik rejection sampling sonunda tam ``p2_yellow_count`` elde
        edilir; fiziksel olarak mümkün değilse üretim açık hata verir.
        """
        count = self.p2_yellow_count
        ux, uy = p2_seg.unit()
        nx, ny = p2_seg.normal()
        placed: List[Dict[str, Any]] = []
        max_attempts = max(1000, count * 500)
        for _attempt in range(max_attempts):
            if len(placed) == count:
                break
            along = rng.uniform(0.0, p2_seg.length_m())
            lateral = (rng.random() * 2.0 - 1.0) * self.yellow_lateral_limit
            x = p2_seg.start[0] + ux * along + nx * lateral
            y = p2_seg.start[1] + uy * along + ny * lateral
            if self._yellow_conflict(x, y, orange, placed):
                continue
            placed.append(
                {
                    "id": f"p2_yellow_{len(placed) + 1}",
                    "x": x,
                    "y": y,
                    "color": "yellow",
                }
            )
        if len(placed) != count:
            raise ValueError(
                f"P2 için {count} sarı duba yerleştirilemedi "
                f"({len(placed)} adet, {max_attempts} aday)"
            )
        return placed

    def _yellow_conflict(
        self,
        x: float,
        y: float,
        orange: List[Dict[str, Any]],
        placed: List[Dict[str, Any]],
    ) -> bool:
        """Sarı aday (x, y) turuncu/sarı dubalara veya WAYPOINT'lere çok yakın mı?

        Waypoint kontrolü: sarı duba waypoint'e (görev noktası) ``yellow_wp_gap_m``
        (8m) mesafeden yakınsa reddedilir — yoksa araç waypoint'e (örn. gn5)
        ulaşamaz (sarı engel üzerinde), P2 hiç tamamlanmaz.
        """
        for o in orange:
            if math.hypot(x - o["x"], y - o["y"]) < self.yellow_min_gap_m:
                return True
        for p in placed:
            if math.hypot(x - p["x"], y - p["y"]) < self.yellow_min_gap_m:
                return True
        for wp in self.waypoint_list():
            if math.hypot(x - wp["x"], y - wp["y"]) < self.yellow_wp_gap_m:
                return True
        return False

    def waypoint_list(self) -> List[Dict[str, Any]]:
        """Waypoint listesi (waypoint'lere yakın sarı engel konmaz)."""
        wps: List[Dict[str, Any]] = []
        for corner in self.p1_corners:
            wps.append({"x": corner[0], "y": corner[1]})
        wps.append({"x": self.p2_end[0], "y": self.p2_end[1]})
        return wps

    # --- P3 hedefleri -------------------------------------------------------------

    def _p3_center(self) -> List[float]:
        """P3 hedef merkezi: gn5'ten ``p3_center_offset_m`` ileri (kuzey)."""
        return [self.p2_end[0] + self.p3_center_offset_m, self.p2_end[1]]

    def _place_targets(self, rng: random.Random) -> List[Dict[str, Any]]:
        """Üç hedef duba: merkezden ~``p3_spread_m`` yayılır, üçgen.

        Açılar 0/120/240 derece; merkezden uzaklık spread * (0.8..1.2)
        aralığında deterministik -> ikili uzaklıklar ~6-9 m (üçgen kenarı
        = yarıçap * sqrt(3) ≈ 6.2-9.4 m). Hedef renkleri şartname sabitidir
        (kırmızı/siyah/yeşil; contracts.VALID_TARGET_COLORS).
        """
        center = self._p3_center()
        colors = ["red", "black", "green"]
        targets = []
        for i, color in enumerate(colors):
            angle_deg = 120.0 * i
            angle = math.radians(angle_deg)
            dist = self.p3_spread_m * (0.8 + 0.4 * rng.random())
            x = center[0] + math.cos(angle) * dist
            y = center[1] + math.sin(angle) * dist
            targets.append(
                {
                    "id": f"target_{color}",
                    "x": x,
                    "y": y,
                    "color": color,
                }
            )
        return targets


# --- Saf yardımcılar --------------------------------------------------------------

def _unit_v(a: List[float], b: List[float]) -> tuple[float, float]:
    """a->b birim vektörü (dejenere segmentte (0,0))."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 0.0, 0.0
    return dx / length, dy / length


def _advance(rng: random.Random, lo: float, hi: float) -> float:
    """[lo, hi] aralığında deterministik rastgele ilerleme (float)."""
    return lo + (hi - lo) * rng.random()


def _segments_intersect(
    a: List[float], b: List[float], c: List[float], d: List[float], eps: float = 1e-9
) -> bool:
    """Kapalı iki doğru parçası kesişiyor veya değiyorsa True."""
    def orient(p: List[float], q: List[float], r: List[float]) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def on_segment(p: List[float], q: List[float], r: List[float]) -> bool:
        return (
            min(p[0], r[0]) - eps <= q[0] <= max(p[0], r[0]) + eps
            and min(p[1], r[1]) - eps <= q[1] <= max(p[1], r[1]) + eps
        )

    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and (
        (o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)
    ):
        return True
    return (
        (abs(o1) <= eps and on_segment(a, c, b))
        or (abs(o2) <= eps and on_segment(a, d, b))
        or (abs(o3) <= eps and on_segment(c, a, d))
        or (abs(o4) <= eps and on_segment(c, b, d))
    )


def count_orange_pairs(data: ScenarioData) -> int:
    """Senaryodaki turuncu duba sayısı / 2 (çift sayısı)."""
    orange = [b for b in data.buoys if b.get("color") == "orange"]
    return len(orange) // 2


def count_yellow(data: ScenarioData) -> int:
    """Senaryodaki sarı (engel) duba sayısı."""
    return sum(1 for b in data.buoys if b.get("color") == "yellow")


def min_clearance(data: ScenarioData) -> float:
    """Sarı dubaların birbirine ve turuncuya en yakın mesafesi (m).

    Doğrulama yardımcısı: sarı engellerin hiçbiri birbirine ya da turuncu
    dubalara ``yellow_min_gap_m``'den yakın DEĞİLSE bu değer >= 2.0'dır.
    """
    yellow = [b for b in data.buoys if b.get("color") == "yellow"]
    orange = [b for b in data.buoys if b.get("color") == "orange"]
    best = float("inf")
    for y in yellow:
        for o in orange:
            best = min(best, math.hypot(y["x"] - o["x"], y["y"] - o["y"]))
    for i, a in enumerate(yellow):
        for b in yellow[i + 1 :]:
            best = min(best, math.hypot(a["x"] - b["x"], a["y"] - b["y"]))
    return best if best != float("inf") else 0.0
