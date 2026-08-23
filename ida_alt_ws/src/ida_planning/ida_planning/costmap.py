"""Gövde-frame lokal costmap (heading-up) — ida_planning.costmap.

CostMap sınıfı lidar engellerini ve kamera renk bilgisini tek bir maliyet
haritasında birleştirir. Araç ileri yönü haritanın +X eksenidir; her tick
``reset()`` ile temizlenir (anlık harita, bayat veri birikmez).

Maliyet kodu sözleşmesi (DWA ve DEBUG/tarayıcı ortak):
  COST_FREE=0      boş (serbest geçiş)
  COST_ORANGE=1    turuncu kenar dubası koridoru (düşük maliyet — koridor)
  COST_UNKNOWN=4   unknown tag'lı engel (düşük güvenli algı: yavaş/dar geçiş)
  COST_GOAL=3      hedef hücresi (P3 hedef dubası yakını)
  COST_YELLOW=8    sarı engel dubası / lidar engeli (yüksek maliyet)
  COST_OBSTACLE=8  şişirilmiş engel alanı (DWA'da çarpışma = engel)

Şişirme yarıçapı bot yarıçapı + güvenlik mesafesidir; ``inflated`` alanı bu
alanları içerir. Tüm fonksiyonlar saf Python'dur (rclpy yok) — test edilebilir.
"""

import bisect
import json
import math
from typing import Any, Dict, List, Optional, Tuple

from ida_planning.geo import clamp
from ida_planning.speed_safety import (
    DEFAULT_BASE_SAFETY_M,
    DEFAULT_MAX_DECEL_MPS2,
    DEFAULT_MAX_SAFETY_M,
    DEFAULT_REACTION_TIME_S,
    safety_radius,
)

# Maliyet kodları (DWA ve görselleştirme ile paylaşılan sabitler).
COST_FREE = 0
COST_ORANGE = 1
COST_GOAL = 3
COST_UNKNOWN = 4
COST_YELLOW = 8
COST_OBSTACLE = 8

# Engelin işgal ettiği hücrelere yazılan "tag" değerleri (DEBUG/to_json).
_TAG_OBSTACLE = "obstacle"
_TAG_ORANGE = "orange"
_TAG_YELLOW = "yellow"
_TAG_GOAL = "goal"
_TAG_UNKNOWN = "unknown"

# Kamera -> lidar engel eşleme yarı açısı (derece). Bearing farkı bu eşiğin
# altındaysa lidar engeline kamera rengi yazılır.
DEFAULT_COLOR_MATCH_DEG = 6.0

# Bilinen renk seti; bu renkler dışındaki (örn. "black") hedef dubaları
# costmap'e bilinmeyen engel olarak girer.
KNOWN_COLORS = {"orange", "yellow"}


class CostMap:
    """Heading-up gövde-frame lokal maliyet haritası (30x30 m, 0.25 m hücre).

    Sınıf argümanları boyut parametreleridir (varsayılan 30 m / 0.25 m hücre =
    120x120 ızgara). Koordinat dönüşümleri metre cinsinden gövde (forward,
    lateral) girdisi alır; hücre indeksleri ızgara köşesinden başlar.

    Şişirme HIZA BAĞLIDIR: ``update(..., ground_speed=v)`` veya ``set_speed(v)``
    yarıçapı ``bot_radius + safety_radius(v)`` yapar (durma mesafesi formülü,
    speed_safety modülü); v=0'da bot + safety_base tabanıdır (eski sabit
    davranış). ``set_safety_radius()`` manuel/DEBUG geçersiz kılmadır, taban
    altına düşmez. ``reset()`` bu durumu korur (tick'ler arası hız bilgisi).
    """

    def __init__(
        self,
        size_m: float = 30.0,
        cell_m: float = 0.25,
        bot_radius_m: float = 0.6,
        safety_m: float = 0.5,
        color_match_deg: float = DEFAULT_COLOR_MATCH_DEG,
        safety_base_m: Optional[float] = None,
        safety_max_m: float = DEFAULT_MAX_SAFETY_M,
        reaction_time_s: float = DEFAULT_REACTION_TIME_S,
        max_decel_mps2: float = DEFAULT_MAX_DECEL_MPS2,
    ) -> None:
        self.size_m = float(size_m)
        self.cell_m = float(cell_m)
        self.bot_radius_m = float(bot_radius_m)
        self.safety_m = float(safety_m)
        self.color_match_deg = float(color_match_deg)
        # Hıza bağlı şişirme parametreleri (speed_safety.safety_radius formülü).
        # safety_base_m verilmezse eski safety_m'ye alias olur — iki taban
        # kaynağı çakışmaz (B3). Duruş tabanı bot + taban güvenlik mesafesidir;
        # araç hızlandıkça set_speed() yarıçapı durma mesafesine göre büyütür.
        if safety_base_m is None:
            safety_base_m = self.safety_m
        self.safety_base_m = float(safety_base_m)
        self.safety_max_m = float(safety_max_m)
        self.reaction_time_s = float(reaction_time_s)
        self.max_decel_mps2 = float(max_decel_mps2)
        # Anlık gövde hızı (m/s) — set_speed()/update(ground_speed) ile güncellenir.
        self._vel_mps = 0.0
        # Izgara 120x120'i aşmamak için hücre sayısını tamsayıya yuvarla.
        self.n = max(1, int(round(self.size_m / self.cell_m)))
        # Geriye uyumluluk: duruş tabanı (sabit). _add_obstacle artık hıza bağlı
        # _inflate_radius'u kullanır; bu alan yalnızca taban değeri taşır.
        self.inflation_radius_m = self.bot_radius_m + self.safety_m
        self._inflate_radius = self.bot_radius_m + safety_radius(
            0.0,
            self.safety_base_m,
            self.safety_max_m,
            self.reaction_time_s,
            self.max_decel_mps2,
        )
        # Şişirme halkası offset tablosu: _add_obstacle her dubada hypot ödemesin.
        # set_speed() yarıçap değişince _build_inflate_offsets ile yeniden üretir.
        self._inflate_offsets: List[Tuple[int, int]] = []
        self._build_inflate_offsets()

        # Maliyet ve tag dizileri her tick'te reset() ile sıfırlanır.
        self.cells: List[List[int]] = [[COST_FREE] * self.n for _ in range(self.n)]
        self.tags: List[List[str]] = [[""] * self.n for _ in range(self.n)]
        self.inflated: List[List[float]] = [[0.0] * self.n for _ in range(self.n)]
        # B10: dirty-set — sıfır olmayan (cells/inflated yazılan) hücreler burada
        # birikir; to_json 14.400 hücre yerine yalnız bunları tarar (~%55 kazanç).
        self._dirty: set = set()
        # Engel seti: inflated >= COST_YELLOW (sarı/gerçek engel) hücreleri.
        # DWA avoid (clearance) bu sete O(1) lookup yapar — grid tarama yok,
        # maliyet ihmal edilebilir (pad=2 tarama 1473ms'ti, bu ~5ms).
        self._obstacle_set: set = set()
        # Sarı duba gövde lateral'leri (m) — yön-farkında avoid için.
        # update()'te sarı buoys'lardan doldurulur; DWA adayın bu dubalardan
        # UZAKLAŞMASINI ödüllendirir (yaklaşanı cezalandırır). Maliyet: yalnız
        # birkaç float (sarı sayısı kadar), hücre taraması YOK.
        self._yellow_laterals: List[float] = []
        # Sarı duba gövde forward'ları (m) — yön terimi yalnız sarı ÖNDEYKEN
        # aktif olsun (sarı arkaya geçince araç heading ile hedefe döner).
        self._yellow_forwards: List[float] = []
        # DWA offset tablosu (çarpışma sorguları) için hızlı erişim.
        self.inflated_lookup: List[List[float]] = self.inflated
        # Renk eşleme için saklanan ham engel listesi (update() içinde doldurulur).
        self.last_obstacles: List[Dict[str, Any]] = []
        # Sıkı koridor modu: True olduğunda turuncu duba hücresi sert engel
        # (COST_OBSTACLE, DWA collides=True), çevresi yumuşak (inflate_cost=3,
        # geçilebilir). Koridor ~3.5m genişliğinde passable kalır.
        self._strict_corridor = False

    # --- Koordinat dönüşümleri ------------------------------------------------

    def world_to_cell(self, forward_m: float, lateral_m: float) -> tuple[int, int]:
        """Metre (gövde) -> (col, row). Menzil dışı değerler clamp edilir."""
        col = int(math.floor(forward_m / self.cell_m + self.n / 2.0))
        row = int(math.floor(-lateral_m / self.cell_m + self.n / 2.0))
        return clamp(col, 0, self.n - 1), clamp(row, 0, self.n - 1)

    def cell_to_world(self, col: int, row: int) -> tuple[float, float]:
        """Hücre merkezi (col, row) -> (forward_m, lateral_m)."""
        return (col + 0.5 - self.n / 2.0) * self.cell_m, (self.n / 2.0 - row - 0.5) * self.cell_m

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.n and 0 <= row < self.n

    # --- Güncelleme ------------------------------------------------------------

    def set_speed(self, ground_speed_mps: float) -> None:
        """Anlık gövde hızını (m/s) ayarlar ve şişirme yarıçapını büyütür.

        Formül (speed_safety): safety = max(taban, min(taban + durma, tavan)),
        durma = v*t_tepki + v^2/(2*a). v < 0 ise 0 sayılır (geri gitmiyoruz);
        bu yüzden yarıçap asla duruş tabanının altına inmez. update() her tick
        başında bu metodu ground_speed ile çağırır; manuel çağrı da aynıdır.
        """
        v = float(ground_speed_mps)
        # NaN/Inf koruması: sonlu olmayan hızı 0.0 say (sessiz NaN yayılımı
        # olmaz; duruş tabanına düşmek güvenli taraftır).
        if not math.isfinite(v):
            v = 0.0
        v = max(0.0, v)
        self._vel_mps = v
        new_radius = self.bot_radius_m + safety_radius(
            v,
            self.safety_base_m,
            self.safety_max_m,
            self.reaction_time_s,
            self.max_decel_mps2,
        )
        # Şişirme yarıçapı değiştiyse halka offset tablosunu yeniden üret
        # (aynı hızda aynı halka — _add_obstacle her dubada hypot ödemez).
        if abs(new_radius - self._inflate_radius) > 1e-9:
            self._inflate_radius = new_radius
            self._build_inflate_offsets()

    def set_safety_radius(self, radius_m: float) -> None:
        """Şişirme yarıçapını manuel/DEBUG amaçlı sabitler.

        Hız tabanının (bot + safety_base) altına düşmez. Dikkat: sonraki
        update()'in set_speed() çağrısı bu değeri hıza bağlı formülle ezer.
        """
        floor = self.bot_radius_m + self.safety_base_m
        self._inflate_radius = max(floor, float(radius_m))

    def get_inflation_radius(self) -> float:
        """Anlık şişirme yarıçapı (bot + hıza bağlı güvenlik mesafesi)."""
        return self._inflate_radius

    def reset(self) -> None:
        """Haritayı tamamen temizler (anlık harita: bayat veri birikmez).

        Hız/şişirme durumuna (set_speed) DOKUNMAZ — tick'ler arası korunur;
        sürüş modu P1/P2/P3 geçişlerinde hıza bağlı yarıçap aynı kalır.
        B10: dirty-set de temizlenir (yeni tick — to_json boş döner).
        """
        self._dirty.clear()
        self._obstacle_set.clear()
        self._yellow_laterals = []
        self._yellow_forwards = []
        for i in range(self.n):
            for j in range(self.n):
                self.cells[i][j] = COST_FREE
                self.tags[i][j] = ""
                self.inflated[i][j] = 0.0

    def update(
        self,
        obstacles: List[Dict[str, Any]],
        buoys: List[Dict[str, Any]],
        target_color: str,
        ground_speed: float = 0.0,
        strict_corridor: bool = False,
    ) -> None:
        """Engelleri işle, şişir, kamera renklerini lidar engellerine eşle.

        Sıralama: (1) anlık hız (ground_speed) şişirme yarıçapını büyütür
        (hıza bağlı güvenlik; default 0.0 = duruş tabanı, geriye uyumlu),
        (2) lidar engelleri hücrelere işlenir (dairesel şişirme dahil),
        (3) kamera renk bilgisi bearing-only 6° eşiğiyle yakındaki lidar
        engeline yazılır, (4) P3 hedef rengi "goal" hücresi olarak işlenir,
        (5) kameradan görülen ama lidar ile eşleşmeyen turuncu/sarı dubalar
        düşük güvenli (unknown) engel olarak eklenir. Bilinmeyen renkli hedef
        dubaları (örn. "black") da unknown engeldir. Her durumda reset() ilk
        çağrıdır.
        """
        self.set_speed(ground_speed)
        self._strict_corridor = strict_corridor
        self.reset()
        # Ham engel listesi sakla (renk eşleme + DEBUG erişimi).
        self.last_obstacles = [o for o in obstacles if isinstance(o, dict)]
        for obs in self.last_obstacles:
            forward = float(obs.get("forward_m", obs.get("x", 0.0)))
            lateral = float(obs.get("lateral_m", obs.get("y", 0.0)))
            if not (math.isfinite(forward) and math.isfinite(lateral)):
                continue
            self._add_obstacle(forward, lateral, COST_OBSTACLE, _TAG_OBSTACLE)

        # Renk eşleme: kameradan gelen turuncu/sarı detection -> lidar engeli.
        # Detection'larda lateral_m eksikse bearing_deg'den tahmin edilir.
        colored = [
            b
            for b in buoys
            if str(b.get("color", "")).lower() in KNOWN_COLORS
            and float(b.get("confidence", 0.0)) >= 0.35
            and float(b.get("distance", 99.0)) <= self.size_m
        ]
        for b in colored:
            self._paint_color_on_lidar(b)

        # P3 hedefi: hedef renk duba yeri goal hücresi yapar (DWA hedef çekimi).
        # Goal hücresi ŞİŞİRİLMEZ (goal maliyeti DWA'da çekim üretir, engel değil).
        if target_color:
            for b in buoys:
                if str(b.get("color", "")).lower() != target_color.lower():
                    continue
                forward, lateral = self._det_forward_lateral(b)
                if float(b.get("distance", 99.0)) <= self.size_m:
                    c, r = self.world_to_cell(forward, lateral)
                    if self.in_bounds(c, r):
                        self.cells[r][c] = COST_GOAL
                        self.tags[r][c] = _TAG_GOAL

        # Lidar ile eşleşmeyen görsel dubalar: düşük güvenli unknown engel.
        # Turuncu koridor dubaları koridor maliyetiyle (1), sarılar engel (8)
        # olarak işlenir. "black/red/green" hedef dubaları P3'te unknown'tur.
        # Sarı dubaların gövde lateral'leri + forward'ları yön-farkında avoid için.
        yellow_laterals = []
        yellow_forwards = []
        for b in buoys:
            color = str(b.get("color", "")).lower()
            if color not in KNOWN_COLORS and color not in {"black", "red", "green"}:
                continue
            confidence = float(b.get("confidence", 0.0))
            distance = float(b.get("distance", 99.0))
            if confidence < 0.35 or distance > self.size_m:
                continue
            if color == target_color.lower():
                continue  # hedef zaten goal olarak işlendi (hedef çekimi)
            forward, lateral = self._det_forward_lateral(b)
            if color == "yellow":
                yellow_laterals.append(lateral)
                yellow_forwards.append(forward)
            if self._covered_by_obstacle(forward, lateral):
                continue  # bu duba zaten lidar engeliyle eşleşti (renk yazıldı)
            if color == "orange":
                # SIKI KORİDOR: turuncu duba hücresi sert engel (geçilemez);
                # normal modda düşük maliyet (koridor geçilebilir).
                cost = COST_OBSTACLE if self._strict_corridor else COST_ORANGE
                tag = _TAG_ORANGE
            elif color == "yellow":
                # SARI = ENGEL (8) — kamera-only görüşte lidar eşleşmesi olmasa bile
                # DWA çarpışmayı algılamalı. Eski COST_UNKNOWN(4) çarpışma eşiğini
                # geçmiyordu, araç sarı dubaya çarpabilirdi (parkur2_analysis S1).
                cost, tag = COST_YELLOW, _TAG_YELLOW
            else:  # hedef renk dubaları (black/red/green), lidar eşleşmesi yok
                cost, tag = COST_UNKNOWN, _TAG_UNKNOWN
            self._add_obstacle(forward, lateral, cost, tag)
        # Sarı lateral'leri + forward'ları kaydet (yön-farkında avoid).
        self._yellow_laterals = yellow_laterals
        self._yellow_forwards = yellow_forwards

    def _add_obstacle(self, forward: float, lateral: float, cost: int, tag: str) -> None:
        """Tek engeli hücre + şişirme çemberine işler (saf, test edilebilir).

        Şişirme yarıçapı hıza bağlıdır: ``_inflate_radius`` = bot yarıçapı +
        safety_radius(anlık hız) — set_speed()/update(ground_speed) ile büyür.
        """
        if not (math.isfinite(forward) and math.isfinite(lateral)):
            return
        c, r = self.world_to_cell(forward, lateral)
        if not self.in_bounds(c, r):
            return
        self.cells[r][c] = cost
        self.tags[r][c] = tag
        # B10: bu hücre + şişirme halkası dirty-set'e eklenir (to_json bunları tarar).
        self._dirty.add((c, r))
        # RENK AYRIMLI HALKA: turuncu koridor duba -> orta maliyet (3), sarı/engel
        # -> 8. Sıkı koridor modunda turuncu merkez hücre COST_OBSTACLE (sert engel)
        # AMA şişirme halkası 3 (yumuşak) kalır — koridor geçilebilir width korunur
        # (inflate=8 tüm koridoru kapatırdı: 1.6m×2 > 3.5m genişlik).
        inflate_cost = 3 if tag == _TAG_ORANGE else COST_OBSTACLE
        inflate_tag = _TAG_ORANGE if tag == _TAG_ORANGE else _TAG_OBSTACLE
        # SIKI KORİDOR merkez koruması: turuncu hücrenin TAM KENDİSİ sert engel
        # (DWA collides=True); şişirme halkası yumuşak kalır (passable).
        if self._strict_corridor and tag == _TAG_ORANGE:
            self.inflated[r][c] = max(self.inflated[r][c], COST_OBSTACLE)
            self._obstacle_set.add((c, r))
        # Önceden hesaplanmış halka offsetleri (set_speed yarıçap değişince
        # yeniden üretir) — her dubada hypot ödenmez; ~10x hız artışı.
        inflated = self.inflated
        tags = self.tags
        dirty = self._dirty
        obstacle_set = self._obstacle_set
        for dc, dr in self._inflate_offsets:
            rr = r + dr
            cc = c + dc
            if not self.in_bounds(cc, rr):
                continue
            if inflated[rr][cc] < inflate_cost:
                inflated[rr][cc] = inflate_cost
                dirty.add((cc, rr))
            # Sarı/gerçek engel halkası (8) -> obstacle_set (avoid clearance için).
            # Turuncu (3) koridor maliyeti DEĞİL — engelden kaçınma sarıyı hedefler.
            if inflate_cost >= COST_YELLOW:
                obstacle_set.add((cc, rr))
            # Şişirme merkez hücresini de kapsar; kostüm hücresi korunur.
            if (dc, dr) != (0, 0):
                tags[rr][cc] = inflate_tag

    def _build_inflate_offsets(self) -> None:
        """Şişirme halkası offset listesini üretir (dc, dr) — dairesel.

        Yarıçap ``_inflate_radius``'a göre dairesel hücreleri önceden hesaplar;
        ``_add_obstacle`` her duba için hypot ödemez. set_speed() yarıçap
        değişince yeniden çağrılır (hız sabitse halka aynı kalır — cache).
        """
        radius_cells = int(math.ceil(self._inflate_radius / self.cell_m))
        r2 = self._inflate_radius * self._inflate_radius
        cell2 = self.cell_m * self.cell_m
        offsets: List[Tuple[int, int]] = []
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if (dr * dr + dc * dc) * cell2 <= r2:
                    offsets.append((dc, dr))
        self._inflate_offsets = offsets

    def _det_forward_lateral(self, det: Dict[str, Any]) -> tuple[float, float]:
        """Detection'dan gövde (forward, lateral) koordinatı.

        Detection'larda forward_m/lateral_m varsa doğrudan kullanılır; yoksa
        distance + bearing_deg'den hesaplanır (bearing-only kontrat yedeği).
        """
        if "forward_m" in det and "lateral_m" in det:
            try:
                return float(det["forward_m"]), float(det["lateral_m"])
            except (TypeError, ValueError):
                pass
        distance = float(det.get("distance", 0.0))
        bearing = float(det.get("bearing_deg", 0.0))
        rad = math.radians(bearing)
        return distance * math.cos(rad), distance * math.sin(rad)

    def _paint_color_on_lidar(self, det: Dict[str, Any]) -> None:
        """Turuncu/sarı detection'ı 6° eşiğindeki lidar engeline renk yazar.

        Bearing-only kontrat: detection'larda yalnızca ``bearing_deg`` garanti
        edilir. Lidar engelleri de forward/lateral taşır; her engelin bearing'i
        hesaplanır ve açısal fark ``color_match_deg``'in altındaysa eşleşir.

        B9: O(n²) -> O(n log n). Engeller bearing'e göre önceden sıralanır ve
        her detection için ``bisect`` ile en yakın bearing bulunur (30 duba ×
        30 engel = 900 atan2 yerine ~30 log(30) eşleşme).
        """
        if not self.obstacles_any():
            return
        if "bearing_deg" in det:
            det_bearing = float(det["bearing_deg"])
        else:
            forward, lateral = self._det_forward_lateral(det)
            det_bearing = math.degrees(math.atan2(lateral, max(forward, 0.001)))
        color = str(det.get("color", "")).lower()
        cost = COST_YELLOW if color == "yellow" else COST_ORANGE
        tag = _TAG_YELLOW if color == "yellow" else _TAG_ORANGE

        # Engelleri bearing'e göre sırala (bounded cache: aynı last_obstacles'ta
        # yeniden hesaplanmaz — her tick update()'te last_obstacles değişebilir).
        obs = self.obstacles_any()
        if not hasattr(self, "_obs_bearings") or self._obs_source is not obs:
            self._obs_source = obs
            self._obs_bearings = []  # [(bearing, index)]
            for i, o in enumerate(obs):
                fwd = float(o.get("forward_m", o.get("x", 0.0)))
                lat = float(o.get("lateral_m", o.get("y", 0.0)))
                if not (math.isfinite(fwd) and math.isfinite(lat)):
                    continue
                b = math.degrees(math.atan2(lat, max(fwd, 0.001)))
                self._obs_bearings.append((b, i))
            self._obs_bearings.sort(key=lambda t: t[0])
        if not self._obs_bearings:
            return

        # bisect ile en yakın bearing'i bul (sıralı listede komşu iki aday).
        bears = [b for b, _ in self._obs_bearings]
        idx = bisect.bisect_left(bears, det_bearing)
        best_obs = None
        best_sep = float("inf")
        for cand_idx in (idx - 1, idx % len(self._obs_bearings)):
            b, oi = self._obs_bearings[cand_idx]
            sep = abs(_wrap_deg(det_bearing - b))
            if sep < best_sep:
                best_sep = sep
                best_obs = obs[oi]
        if best_obs is not None and best_sep <= self.color_match_deg:
            forward = float(best_obs.get("forward_m", best_obs.get("x", 0.0)))
            lateral = float(best_obs.get("lateral_m", best_obs.get("y", 0.0)))
            # LİDAR ÖNCELİĞİ (güven eşikli min-cost): bu nokta bir LİDAR engeliyle
            # eşleşti. Lidar engeli zaten hard obstacle (COST_OBSTACLE=8) olarak
            # işlendi; kamera rengi YALNIZCA etikettir. Modeller iyi olmadığı için
            # yanlış renk ataması (sarı engel -> turuncu sanılması) engelin
            # maliyetini DÜŞÜRMEMELİ — yoksa DWA sarı engeli "geçilebilir koridor"
            # sanıp içinden geçmeyi dener (koridordan çıkma/çarpışma riski).
            # Kural: turuncu paint (cost=1) yalnız YÜKSEK güvenli detection'da
            # hard engeli koridor yapabilir (P1 koridor dubası geçilebilirliği).
            # Düşük güvenli turuncu (yanlış atama riski) hard engeli asla
            # yumuşatamaz; sarı paint (8) zaten sert kalır.
            try:
                det_confidence = float(det.get("confidence", 0.0))
            except (TypeError, ValueError):
                det_confidence = 0.0
            if cost < COST_OBSTACLE and det_confidence < 0.5:
                # Düşük güvenli turuncu: hard engeli yumuşatma — engel 8 kalır.
                c, r = self.world_to_cell(forward, lateral)
                if self.in_bounds(c, r) and self.cells[r][c] >= COST_OBSTACLE:
                    self.cells[r][c] = COST_OBSTACLE
                    self.tags[r][c] = _TAG_OBSTACLE
                    return
            self._add_obstacle(forward, lateral, cost, tag)

    def _covered_by_obstacle(self, forward: float, lateral: float) -> bool:
        """Koordinat, mevcut bir engel/şişirme hücresiyle örtüşüyor mu?"""
        c, r = self.world_to_cell(forward, lateral)
        return self.in_bounds(c, r) and self.inflated[r][c] >= COST_OBSTACLE

    def obstacles_any(self) -> List[Dict[str, Any]]:
        """Önceki tick'ten saklanan engel listesi (renk eşleme için).

        update() içinde kullanılan anlık tampon; ``last_obstacles`` alanından
        beslenir. Saha/DEBUG'da ham engel listesine erişim için de kullanılır.
        """
        return getattr(self, "last_obstacles", [])

    # --- Sorgu ------------------------------------------------------------------

    def cost(self, x: float, y: float) -> int:
        """Gövde (x=forward, y=lateral) metre koordinatının hücre maliyeti."""
        c, r = self.world_to_cell(x, y)
        if not self.in_bounds(c, r):
            return COST_OBSTACLE
        return self.cells[r][c]

    def cost_at_cell(self, col: int, row: int) -> int:
        if not self.in_bounds(col, row):
            return COST_OBSTACLE
        return self.cells[row][col]

    def tag_at_cell(self, col: int, row: int) -> str:
        if not self.in_bounds(col, row):
            return ""
        return self.tags[row][col]

    # --- Çarpışma ----------------------------------------------------------------

    def collides(self, x: float, y: float) -> bool:
        """Nokta şişirilmiş engel alanında mı? (DWA aday süpürme sorgusu)."""
        c, r = self.world_to_cell(x, y)
        if not self.in_bounds(c, r):
            return True
        return self.inflated[r][c] >= COST_OBSTACLE

    def collides_cells(self, cols: List[int], rows: List[int]) -> bool:
        """Offset tablosu (önceden hesaplı ~180 hücre) ile toplu çarpışma sorgusu."""
        for c, r in zip(cols, rows):
            if not self.in_bounds(c, r):
                return True
            if self.inflated[r][c] >= COST_OBSTACLE:
                return True
        return False

    def collides_only_real(self, cols: List[int], rows: List[int]) -> bool:
        """RENK AYRIMLI çarpışma: yalnız sarı/gerçek engel (cost>=8) çarpışık.

        Turuncu koridor dubaları (cost 1) sert engel sayılmaz — P1'de araç turuncu
        dubalar arasından geçebilir (koridor geçilebilir). Sarı engel/lidar engeli
        hâlâ eler (P2 kaçınma korunur).
        """
        for c, r in zip(cols, rows):
            if not self.in_bounds(c, r):
                return True
            if self.inflated[r][c] >= COST_YELLOW:
                return True
        return False

    # --- Serileştirme ---------------------------------------------------------------

    def to_json(self) -> str:
        """Teslim/DEBUG için kompakt JSON (yalnız sıfır olmayan hücreler).

        Her hücre: [forward_m, lateral_m, cost, tag]. Şişirme halkası (cost 8,
        tag "inflated") haritaya işlenmemiş hücreleri de kapsadığından onlar da
        ayrıca yazılır; görselleştirme tam şişirilmiş alanı çizebilir.
        """
        n = self.n
        half = n / 2.0
        cell_m = self.cell_m
        cells = self.cells
        inflated = self.inflated
        tags = self.tags
        out = []
        append = out.append
        # B10: 14.400 hücre yerine yalnız dirty-set (bu tick'te yazılan) hücreler
        # taranır. reset() _dirty'yi temizlediğinden boş haritada çıktı boştur.
        for c, r in sorted(self._dirty):
            cost = cells[r][c]
            infl = inflated[r][c]
            if cost == COST_FREE and infl < COST_OBSTACLE:
                continue
            fwd = round((c + 0.5 - half) * cell_m, 2)
            lat = round((half - r - 0.5) * cell_m, 2)
            if cost == COST_FREE:
                # Şişirme halkası: tag "inflated", maliyet engel kodu.
                append([fwd, lat, COST_OBSTACLE, "inflated"])
            else:
                append([fwd, lat, cost, tags[r][c]])
        return json.dumps(
            {
                "size_m": self.size_m,
                "cell_m": cell_m,
                "n": n,
                "cells": out,
            },
            ensure_ascii=True,
            separators=(",", ":"),
        )


def _wrap_deg(angle: float) -> float:
    """Açıyı [-180, 180] aralığına sarar (bearing farkı hesabı)."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle
