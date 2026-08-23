"""Dynamic Window Approach (DWA) — gövde-frame hız planlayıcı (ida_planning.dwa).

``plan(goal_body, costmap, last_cmd)`` fonksiyonu aday (vx, yaw_rate) çiftlerini
üretir, costmap üzerinde simüle eder ve skorlar. En yüksek skorlu aday döner;
tüm adaylar çarpışıyorsa ``recovery()`` ile deterministik sağa dönüş yapılır.

Skor terimleri (hepsi yüksek iyidir, engel hariç):
  obstacle  : aday süpürme yolunda engel -> -inf (elenir)
  cost_swept: süpürme hücrelerinin ortalama maliyeti (turuncu koridor = düşük
              maliyet -> yüksek skor; unknown = yavaş/dar koridor maliyeti)
  corridor  : aktif turuncu koridor merkezinin lateral'ine yönelim (heading
              katmanı; plan()'a ``corridor`` dict'i verilirse eklenir)
  heading   : aday son yönelimi hedef bearing'ine ne kadar yakın
  progress  : aday son konumunun hedefe yaklaşımı (metre)
  speed     : vx ödülü (hızlı adaylar tercih edilir, dönüşte ceza)
  smoothness: önceki komuta yakınlık (sarsıntısız sürüş)
  unknown   : süpürme yolu unknown hücreden geçiyorsa ceza (yavaşla)

Parametrelerin tamamı fonksiyon/sınıf argümanıdır; testler ve autonomy.yaml
ile eşleşir. Tüm fonksiyonlar saf Python'dur (rclpy yok).
"""

import math
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ida_planning.costmap import COST_UNKNOWN, COST_YELLOW, CostMap
from ida_planning.geo import clamp

# -- Varsayılan ağırlıklar (autonomy.yaml varsayılanlarıyla aynı) -----------------

# DUNKU CALISAN degerler (P1 koridor takibi): heading 1.8, speed 0.6, sim 3.5.
# Stall fix denemesi (1.4/1.0/2.0) P1'i bozdu — geri alindi.
DEFAULT_W = {
    "obstacle": 0.7,
    "heading": 1.8,
    "progress": 1.2,
    "speed": 0.6,
    "smooth": 0.3,
    "unknown": 0.6,
    "corridor": 1.6,
    "avoid": 0.25,  # Kamera yön ipucu ikincil; lidar çarpışma veto'su birincil.
}

DEFAULT_SIM = {
    "sim_time_s": 3.5,
    "sim_step_s": 0.25,
}

DEFAULT_ACCEL = {
    "max_accel_mps2": 0.8,
    "max_decel_mps2": 1.2,
    "max_yaw_accel_deg_s2": 90.0,
}


@dataclass
class DwaCommand:
    """DWA çıktısı: vx (m/s) ve yaw_rate (rad/s)."""

    vx: float
    yaw_rate: float
    action: str
    score: float = 0.0


@dataclass
class Candidate:
    """Tek aday: vx, yaw_rate (rad/s) ve skor terimleri (DEBUG/teslim)."""

    vx: float
    yaw_rate: float
    score: float = 0.0
    heading: float = 0.0
    progress: float = 0.0
    speed: float = 0.0
    smooth: float = 0.0
    unknown: float = 0.0
    swept_cost: float = 0.0
    corridor: float = 0.0
    avoid: float = 0.0
    collides: bool = False


# ---------------------------------------------------------------------------
# Offset tablosu: robot gövde yarıçapına oturan ~180 hücre önceden hesaplanır.
# DWA adaylarının süpürme çizgisi bu tabloyla offset'lendiği için çarpışma
# sorgusu çevrimiçi geometri gerektirmez (tek seferlik inşa).
# ---------------------------------------------------------------------------

def build_offset_cells(bot_radius_m: float, cell_m: float) -> List[Tuple[int, int]]:
    """Bot yarıçapına oturan dairesel hücre offset listesi üretir (~180 hücre).

    Her hücrenin merkezi bot merkezine ``bot_radius_m + 0.5*hücre`` mesafeye
    kadarsa offset'e dahil edilir (dairesel, köşe boşlukları hariç). Bot
    yarıçapı 1.1 m ve hücre 0.25 m ile ~180 hücre elde edilir; şişirme zaten
    güvenlik marjını kapsadığından çarpışma testi muhafazakârdır.
    """
    radius_cells = max(1, int(math.ceil((bot_radius_m + cell_m * 0.5) / cell_m)))
    offsets: List[Tuple[int, int]] = []
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            # Dairesel (kare değil): köşe hücreleri dahil edilmez.
            if math.hypot(dr * cell_m, dc * cell_m) <= bot_radius_m + cell_m * 0.5:
                offsets.append((dc, dr))
    return offsets


def build_swept_cells(
    sim_time_s: float,
    sim_step_s: float,
    vx: float,
    yaw_rate: float,
    offset_cells: Sequence[Tuple[int, int]],
    n_cells: int,
    cell_m: float,
) -> Tuple[List[int], List[int]]:
    """Adayı (vx, yaw_rate) simülasyon süresi boyunca süpürür; hücre listesi döner.

    Kinematik: yaw değişimi = yaw_rate * t; konum ileri yönde toplanır (vx sabit).
    Dönüşlü adaylar için arklı yörünge: her adımın ileri yönü farklıdır.
    """
    cols: List[int] = []
    rows: List[int] = []
    x_fwd = 0.0
    y_lat = 0.0
    heading = 0.0
    # Harita merkezi gövbe (0,0) demektir; süpürme hücreleri bu merkeze göre
    # ofsetlenir (yoksa aday yolu haritanın başında, engellerden uzak kalır).
    center_offset = n_cells // 2
    for step in range(1, int(max(1.0, sim_time_s / sim_step_s)) + 1):
        t = step * sim_step_s
        heading = yaw_rate * t
        if abs(yaw_rate) > 1e-9:
            # Dairesel yay: R = vx / yaw_rate (rad/s).
            r = vx / yaw_rate
            x_fwd = r * math.sin(heading)
            y_lat = r * (1.0 - math.cos(heading))
        else:
            x_fwd = vx * t
            y_lat = 0.0
        center_col = center_offset + int(round(x_fwd / cell_m))
        center_row = center_offset + int(round(-y_lat / cell_m))
        for dc, dr in offset_cells:
            c = center_col + dc
            r = center_row + dr
            if 0 <= c < n_cells and 0 <= r < n_cells:
                cols.append(c)
                rows.append(r)
    return cols, rows


# ---------------------------------------------------------------------------
# DWA planlayıcı
# ---------------------------------------------------------------------------

class DwaPlanner:
    """Gövde-frame DWA planlayıcı (durum: son komut + offset tablosu).

    ``plan(goal_body, costmap, last_cmd)`` her tick çağrılır; aday üretimi,
    çarpışma elemesi ve skorlama saf fonksiyonlarla yapılır.

    max_speed DİNAMİKTİR: ``set_max_speed()`` nominal tavanı, ``set_max_speed_scale()``
    ise [0,1] ölçeğini değiştirir; etkin tavan ``effective_max_speed()`` =
    nominal * ölçek'tir (governor entegrasyonu). Aday üretimi, skor normalizasyonu
    ve recovery bu etkin tavana saygı duyar; varsayılan ölçek 1.0 = eski davranış.
    """

    def __init__(
        self,
        max_speed_mps: float = 0.8,
        max_yaw_rate_deg_s: float = 45.0,
        vx_steps: int = 7,
        yaw_steps: int = 15,
        sim_time_s: float = DEFAULT_SIM["sim_time_s"],
        sim_step_s: float = DEFAULT_SIM["sim_step_s"],
        accel_limits: Optional[Dict[str, float]] = None,
        w: Optional[Dict[str, float]] = None,
        recovery_vx: float = 0.25,
        recovery_yaw_deg_s: float = 35.0,
        unknown_slow_vx: float = 0.3,
        min_drive_speed_mps: float = 0.08,
        min_turn_rate_deg_s: float = 0.0,
        recovery_latch_ticks: int = 0,
        align_before_drive_deg: float = 0.0,
        align_yaw_rate_deg_s: float = 0.0,
        bot_radius_m: float = 1.0,
        cell_m: float = 0.25,
        n_cells: int = 120,
        max_speed_scale: float = 1.0,
        avoid_direction_latch_ticks: int = 8,
    ) -> None:
        self.max_speed = float(max_speed_mps)
        self._max_speed_nominal = float(max_speed_mps)
        self.max_speed_scale = clamp(float(max_speed_scale), 0.0, 1.0)
        self.max_yaw_rate = math.radians(float(max_yaw_rate_deg_s))
        self.vx_steps = max(3, int(vx_steps))
        self.yaw_steps = max(3, int(yaw_steps))
        self.sim_time_s = float(sim_time_s)
        self.sim_step_s = float(sim_step_s)
        self.accel = {**DEFAULT_ACCEL, **(accel_limits or {})}
        self.w = {**DEFAULT_W, **(w or {})}
        self.recovery_vx = float(recovery_vx)
        self.recovery_yaw = math.radians(float(recovery_yaw_deg_s))
        self.unknown_slow_vx = float(unknown_slow_vx)
        self.min_drive_speed = max(0.0, float(min_drive_speed_mps))
        self.min_turn_rate = min(
            self.max_yaw_rate,
            math.radians(max(0.0, float(min_turn_rate_deg_s))),
        )
        self.recovery_latch_ticks = max(0, int(recovery_latch_ticks))
        self._recovery_direction = 0.0
        self._recovery_latch_remaining = 0
        # DWA kaçınma yön histerezisi (chatter önleme): sabit engelde
        # v=0/pivot ile v=0.7/arc arasındaki hızlı geçiş, en iyi adayın yaw
        # işareti değiştiğinde dahi eski yönü bir süre koruyarak bastırılır.
        # Yön değişimi "kararlı kanıt" ister: yeni yön en az
        # ``avoid_direction_latch_ticks`` boyunca tekrar seçilirse latch kırılır.
        self.avoid_direction_latch_ticks = max(0, int(avoid_direction_latch_ticks))
        self._avoid_direction = 0.0
        self._avoid_ticks_remaining = 0
        align_before = float(align_before_drive_deg)
        align_rate = float(align_yaw_rate_deg_s)
        if not math.isfinite(align_before) or not 0.0 <= align_before <= 180.0:
            raise ValueError("align_before_drive_deg must be finite within [0, 180]")
        if not math.isfinite(align_rate) or not 0.0 <= align_rate <= 180.0:
            raise ValueError("align_yaw_rate_deg_s must be finite within [0, 180]")
        self.align_before_drive = math.radians(align_before)
        self.align_yaw_rate = min(
            self.max_yaw_rate,
            math.radians(align_rate),
        )
        self.bot_radius_m = float(bot_radius_m)
        self.cell_m = float(cell_m)
        self.n_cells = int(n_cells)
        self.offset_cells = build_offset_cells(bot_radius_m, cell_m)
        self.last_cmd: Optional[DwaCommand] = None
        # Aktif koridor bilgisi (plan() ile her tick güncellenir; yoksa bias etkisiz).
        self._corridor: Optional[Dict[str, Any]] = None
        # Süpürme geometrisi cache'i: aynı (vx, yaw, n_cells) aynı hücre listesini
        # üretir. Aday seti (dinamik pencereden) her tick tekrar ettiğinden 1.
        # tick'ten sonra isabet oranı ~1.0 olur — build_swept_cells'teki ~2520 hücre
        # üretimi çoğu tick'te hiç çalışmaz (tick darboğazı ana kaynağı).
        # OrderedDict: ekleme sırası = FIFO sırası — popitem(last=False) O(1) ile
        # en eski giriş atılır; ayrı senkron liste gerekmez (cache/order tutarsızlığı
        # imkânsız — code-review B1/B2).
        self._swept_cache: "OrderedDict[Tuple[float, float, int], Tuple[List[int], List[int]]]" = OrderedDict()
        self._swept_cache_max = 512

    # --- Süpürme cache'i --------------------------------------------------------

    def _swept_cells(self, vx: float, yaw_rate: float, n_cells: int) -> Tuple[List[int], List[int]]:
        """Süpürme hücrelerini cache'ler — aynı (vx, yaw, n_cells) aynı geometriyi üretir.

        Key, float kayan nokta kararlılığı için 5 ondalığa yuvarlanır (aday
        üretimi deterministiktir; aynı vx/yaw her tick aynı float'ı verir).
        Cache doluysa FIFO (en eski) giriş atılır — sınırlı bellek.
        B PLANI: Python list döndürür (numpy array değil — Python döngüleriyle
        uyumlu; numpy kısmi entegrasyonu maliyeti artırmıştı).
        """
        key = (round(vx, 5), round(yaw_rate, 5), n_cells)
        hit = self._swept_cache.get(key)
        if hit is not None:
            return hit
        cols, rows = build_swept_cells(
            self.sim_time_s, self.sim_step_s, vx, yaw_rate,
            self.offset_cells, n_cells, self.cell_m,
        )
        if len(self._swept_cache) >= self._swept_cache_max:
            # FIFO: en eski girişi at (popitem(last=False) O(1)).
            self._swept_cache.popitem(last=False)
        self._swept_cache[key] = (cols, rows)
        return cols, rows

    # --- max_speed yönetimi (governor entegrasyonu) -----------------------------
    # Nominal tavan (_max_speed_nominal) yapıcıda sabitlenir ve runtime'da
    # DEĞİŞMEZ; yalnız ölçek (set_max_speed_scale) etkin tavanı değiştirir.

    def set_max_speed_scale(self, scale: float) -> None:
        """Hız ölçeğini [0, 1] aralığına clamp'lar (governor çıkışı)."""
        self.max_speed_scale = clamp(float(scale), 0.0, 1.0)

    def effective_max_speed(self) -> float:
        """Etkin azami hız: nominal * ölçek (governor'ın sınırladığı tavan)."""
        return self._max_speed_nominal * self.max_speed_scale

    # --- Dinamik pencere (hızlanma limitleri) ---------------------------------

    def dynamic_window(self, last: Optional[DwaCommand]) -> Tuple[float, float, float, float]:
        """Mevcut hıza göre izinli (vx_min, vx_max, yaw_min, yaw_max).

        Hızlanma limitleri: vx üst sınır = last.vx + max_accel * dt (dt=1 tick
        varsayılır); alt sınır last.vx - max_decel; yaw benzeri yaw_accel ile.
        vx tavanı etkin max_speed'tir (governor ölçeği aday üretimini sınırlar).
        Son komut yoksa tam aralık döner.
        """
        vx_eff = self.effective_max_speed()
        if last is None:
            return 0.0, vx_eff, -self.max_yaw_rate, self.max_yaw_rate
        vx_max = clamp(last.vx + self.accel["max_accel_mps2"], 0.0, vx_eff)
        vx_min = clamp(last.vx - self.accel["max_decel_mps2"], 0.0, vx_max)
        yaw_max = clamp(last.yaw_rate + math.radians(self.accel["max_yaw_accel_deg_s2"]),
                        -self.max_yaw_rate, self.max_yaw_rate)
        yaw_min = clamp(last.yaw_rate - math.radians(self.accel["max_yaw_accel_deg_s2"]),
                        -self.max_yaw_rate, self.max_yaw_rate)
        return vx_min, vx_max, yaw_min, yaw_max

    # --- Aday üretimi -----------------------------------------------------------

    def candidates(self, goal: Tuple[float, float], last: Optional[DwaCommand]) -> List[Candidate]:
        """(vx, yaw_rate) aday seti: ~105 aday (7 vx x 15 yaw)."""
        vx_min, vx_max, yaw_min, yaw_max = self.dynamic_window(last)
        # vx=0 stop'u engelle: engel önünde vx=0 serbest aday kazanıp araç duruyor.
        # Minimum ileri hız zorla (>0) — dönüş adayları seçilsin.
        # Su üstünde ölçülen kontrol edilebilir en düşük hızın altındaki adaylar
        # planlama ufkunu yapay biçimde kısaltıp (örn. 0.08*3.5=0.28 m)
        # engeli geç fark ettiriyordu. Alt sınır aday üretilmeden uygulanır;
        # böylece çarpışma hesabı gönderilecek gerçek hızla aynıdır.
        vx_min = min(vx_max, max(self.min_drive_speed, vx_min))
        vxs = [
            round(vx_min + (vx_max - vx_min) * (i / max(1, self.vx_steps - 1)), 2)
            for i in range(self.vx_steps)
        ]
        # yaw taraması [min, max] aralığında; vx=0 adayında 0 dahil.
        # Nicemleme: last her plan'da değişince vx/yaw adayları kayıyordu →
        # _swept_cells cache'i sürekli miss (maliyet 202ms). round ile ardışık
        # plan'larda adaylar aynı kalır, cache hit eder (B1 varsayımı gerçekleşir).
        raw_yaws = [
            round(yaw_min + (yaw_max - yaw_min) * (i / max(1, self.yaw_steps - 1)), 4)
            for i in range(self.yaw_steps)
        ]
        # Saha regresyonu: yaklaşık 5 deg/s yaw komutları diferansiyel itki
        # eşiğinin altında kaldı ve heading değişmedi. Düz aday (0) korunur;
        # fakat seçilen her gerçek dönüş, ölçülmüş etkin alt sınırın altında
        # kalmayacak şekilde nicemlenir.
        yaws = []
        for yaw in raw_yaws:
            if abs(yaw) < 1e-6:
                adjusted = 0.0
            elif abs(yaw) < self.min_turn_rate:
                adjusted = math.copysign(self.min_turn_rate, yaw)
            else:
                adjusted = yaw
            yaws.append(round(clamp(adjusted, -self.max_yaw_rate, self.max_yaw_rate), 4))
        yaws = sorted(set(yaws))
        out: List[Candidate] = []
        for vx in vxs:
            for yaw in yaws:
                out.append(Candidate(vx=vx, yaw_rate=yaw))
        return out

    # --- Skorlama -----------------------------------------------------------------

    def score_candidate(
        self,
        cand: Candidate,
        goal: Tuple[float, float],
        last: Optional[DwaCommand],
        costmap: CostMap,
    ) -> Candidate:
        """Tek adayı skorlar. Çarpışan aday score=-inf (elenir)."""
        swept_cols, swept_rows = self._swept_cells(cand.vx, cand.yaw_rate, costmap.n)
        # TEK GEÇİŞ: çarpışma + cost_swept avg + unknown sayımı aynı döngüde.
        # Eski akışta collides_only_real ayrı gezilirdi (2520 hücre 2x); birleşik
        # döngü erken çıkışı korur ve her hücreye tek erişim yapar.
        # RENK AYRIMLI çarpışma: turuncu koridor geçilebilir (sarı/engel eler);
        # sınır dışı hücre de çarpışık sayılır (collides_only_real davranışı).
        # cost_swept: süpürme hücrelerinin ortalama maliyeti (engel yok, yaşanabilir).
        # Turuncu halka cost 3 (düşük) avg'ye hafif dahil — araç turuncudan hafifçe
        # uzaklaşır ama dar koridorda merkezden sapmaz (geçilebilirlik korunur).
        # B PLANI: numpy GERİ ALINDI (kısmi entegrasyon maliyeti artırdı — swept
        # numpy olunca Python döngüleri yavaşladı). Python döngüsüne dönüldü.
        # TEK GEÇİŞ: çarpışma + cost_swept avg + unknown sayımı aynı döngüde.
        # Early-exit korunur; her hücreye tek erişim. Turuncu (3) geçilebilir,
        # sarı/engel (>=8) eler, sınır dışı collides.
        collides = False
        total = 0.0
        n_real = 0
        unknown_count = 0
        inflated = costmap.inflated
        cells = costmap.cells
        n = costmap.n
        for c, r in zip(swept_cols, swept_rows):
            if not (0 <= c < n and 0 <= r < n):
                collides = True
                break
            v = inflated[r][c]
            if v >= COST_YELLOW:
                collides = True
                break
            if v >= 3:
                total += v
                n_real += 1
            if cells[r][c] == COST_UNKNOWN:
                unknown_count += 1
        if collides:
            cand.collides = True
            cand.score = float("-inf")
            return cand
        avg = total / max(1, n_real) if n_real > 0 else 0.0

        # YÖN-FARKINDA AVOID: sarı dubalardan UZAKLAŞAN adayı ödüllendir,
        # yaklaşanı cezalandır (envanter #13 fix + P2 akıcı kaçınma).
        # Eski kod (a) süpürme içi >= COST_YELLOW arıyordu — collides'ta elendiği
        # için avoid hep 0 (ölü kod); (b) 5x5 çevre taraması 1473ms'ti (ölü).
        # YENİ: (1) _obstacle_set'e 4-komşu O(1) lookup (çarpışmasız yakın engel),
        # (2) sarı duba lateral'lerine göre YÖN: adayın yörünge sonu lateral'i
        # sarıdan uzaklaşıyorsa ceza küçülür (kaçınma ödüllenir), yaklaşıyorsa
        # büyür. Maliyet: set lookup + birkaç float — ihmal edilebilir (~5ms).
        avoid = 0.0
        min_dist = float("inf")
        obstacle_set = costmap._obstacle_set
        half_n = costmap.n / 2.0
        cell_m = costmap.cell_m
        for c, r in zip(swept_cols, swept_rows):
            for dc, dr in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                if (c + dc, r + dr) in obstacle_set:
                    dist = math.hypot((c + dc - half_n) * cell_m, (half_n - (r + dr)) * cell_m)
                    min_dist = dist if dist < min_dist else min_dist
        if math.isfinite(min_dist):
            avoid = 1.0 / (min_dist + 0.5)  # 0.5 stabilizasyon
            # YÖN TERİMİ (yaw_rate işaretine göre): sarı dubadan UZAKLAŞAN adayı
            # ödüllendir. Sarı SAĞDA (lateral>0) ise pozitif yaw (sağa) YAKLAŞIR
            # (ceza), negatif yaw (sola) UZAKLAŞIR (ödül). Sarı solda tersi.
            # Bu, heading'in yanal kaçınmayı ezmesini kırar (P2 akıcı geçiş).
            yellows = costmap._yellow_laterals
            yellow_fwds = costmap._yellow_forwards
            # Yön terimi yalnız sarı ÖNDEYKEN (forward > 0) aktif — sarı arkaya
            # geçince etkisiz, araç heading ile hedefe döner (merkeze dönüş).
            if yellows and any(f > 0.0 for f in yellow_fwds):
                # S4 (parkur2_analysis): en YAKIN öndeki sarıyı seç (mesafe bazlı) —
                # eski `min(y for y in yellows)` en soldakini seçiyordu, çoklu sarıda
                # sağdakini görmezdi. En yakın sarının işareti (sağda +, solda -).
                front_pairs = [(f, l) for f, l in zip(yellow_fwds, yellows) if f > 0.0]
                nearest = min(front_pairs, key=lambda fl: math.hypot(fl[0], fl[1]))
                y_sign = 1.0 if nearest[1] >= 0.0 else -1.0
                # Aday yaw_rate yönü: yaw * sarı işareti. Pozitif = sarıya yaklaşır
                # (ceza büyür), negatif = uzaklaşır (ceza küçülür).
                approach = cand.yaw_rate * y_sign * self.sim_time_s
                # approach>0 (sarıya doğru dönüş) -> direction 1.0 (tam ceza);
                # approach<0 (uzaklaşma) -> direction 0.4 (ceza %60 azalır).
                direction = 1.0 if approach >= 0.0 else 0.4
                avoid *= direction
        cand.avoid = avoid

        # Hedefe yönelim: aday son yönelimi vs hedef bearing.
        # TAM 4-KADRAN atan2 (P2 180° dönüş kök nedeni): hedef arkadaysa
        # (goal[0] < 0) max(goal[0], 0.001) bearing'i 0'a çekiyor ve yaw=0
        # adayına yanlış heading_score=1.0 veriyordu; araç hedef kıçta
        # kalınca geri dönüş hiçbir adayda kazanamıyordu. Hefef arkadaysa
        # bearing ±180° olur; yaw=0 adayı düşük, hedefe dönüş yüksek skor alır.
        goal_bearing = math.atan2(goal[1], goal[0])
        heading_err = _wrap_rad(goal_bearing - cand.yaw_rate * self.sim_time_s)
        heading_score = 1.0 - abs(heading_err) / math.pi

        # Koridor KISITI (waypoint birincil hedef): DWA adayı koridoru terk
        # etmesin — adayın sim_time_s sonundaki yanal pozisyonu koridor merkezine
        # yakınsa ödül, uzaksa ceza. Bu, "merkeze yönel" (goal-direction) bias'ından
        # FARKLIDIR: hedef waypoint kalır, ama koridor dışına çıkan aday cezalanır.
        # center_lateral_m > 0 (merkez sağda) -> araç sağa kaymalı; adayın ilerideki
        # yanal sapması (vx,yaw'dan) merkeze yakınsa yüksek skor.
        # confidence ile ölçeklenir — tek taraflı görüşte düşük güvenli tahmin
        # heading'i ezmesin (yanlış merkez -> dubalara sapma).
        # Adayın planlama ufku sonundaki yaklaşık body-frame konumu. Aynı
        # geometri hem mission-route zarfında hem ilerleme skorunda kullanılır.
        x_end = cand.vx * self.sim_time_s * math.cos(cand.yaw_rate * self.sim_time_s / 2.0)
        y_end = cand.vx * self.sim_time_s * math.sin(cand.yaw_rate * self.sim_time_s / 2.0)
        corridor_score = 0.0
        corr = getattr(self, "_corridor", None)
        if corr and corr.get("active"):
            center_lateral = float(corr.get("center_lateral_m", 0.0))
            center_forward = float(corr.get("center_forward_m", 6.0))
            confidence = float(corr.get("confidence", 0.0))
            if corr.get("source") == "mission_route":
                current_cross = float(corr.get("current_cross_track_m", 0.0))
                cross_forward = float(corr.get("cross_forward_coeff", 0.0))
                cross_right = float(corr.get("cross_right_coeff", 0.0))
                hard_half = float(corr.get("hard_half_width_m", 0.0))
                predicted_cross = (
                    current_cross
                    + x_end * cross_forward
                    + y_end * cross_right
                )
                current_abs = abs(current_cross)
                predicted_abs = abs(predicted_cross)
                # Parkur dışına doğru giden adayı veto et. Araç akıntı veya eski
                # komut yüzünden zaten dışarıdaysa yalnız |sapma| azaltan adaylar
                # açık kalır; böylece bütün adayları eleyip saf pivot deadlock'u
                # üretmeyiz.
                if (
                    hard_half > 0.0
                    and predicted_abs > hard_half
                    and predicted_abs >= current_abs - 0.05
                ):
                    cand.collides = True
                    cand.score = float("-inf")
                    return cand
                corridor_score = confidence * max(
                    0.0, 1.0 - predicted_abs / max(0.1, hard_half)
                )
            else:
                # Görsel koridorun eski davranışı P1 için aynen korunur.
                lateral_err = abs(y_end - center_lateral)
                corridor_score = confidence * max(0.0, 1.0 - lateral_err / 3.5)

        # İlerleme: aday son konumunun hedefe olan mesafe farkı.
        d_now = math.hypot(goal[0], goal[1])
        d_end = math.hypot(goal[0] - x_end, goal[1] - y_end)
        progress_score = clamp((d_now - d_end) / max(1.0, d_now), -1.0, 1.0)
        # Engel önünde progress negatif olup vx>0 adayları cezalanıyor (vx=0 kazanıyor).
        # Negatif progress'i 0'a çek — dönüş adayları hız bonusu koruyabilsin.
        progress_score = max(0.0, progress_score)

        # Hız ödülü (normalize): dönüşlü adaylarda hafif ceza.
        # Etkin max_speed'e göre normalize (governor daralttığında tavan 1.0'a yakın).
        speed_score = clamp(cand.vx / max(0.1, self.effective_max_speed()), 0.0, 1.0)
        if abs(cand.yaw_rate) > self.max_yaw_rate * 0.8:
            speed_score *= 0.7

        # Smoothness: önceki komuta yakınlık (vx ve yaw farkı).
        # vx normalize etkin max_speed'e göre — governor daralttığında da
        # tutarlı kalır (B6).
        if last is not None:
            smooth_score = clamp(
                1.0
                - abs(cand.vx - last.vx) / max(0.1, self.effective_max_speed())
                - abs(cand.yaw_rate - last.yaw_rate) / max(0.1, self.max_yaw_rate),
                0.0,
                1.0,
            )
        else:
            smooth_score = 0.5

        # unknown cezası: yol unknown hücreden geçiyorsa skor düşer -> yavaşla.
        unknown_score = clamp(1.0 - unknown_count / 30.0, 0.0, 1.0)

        # Swept cost -> skor: maliyet 0 en iyi (1.0), turuncu koridor hafif düşük,
        # sarı/engel zaten elendi. unknown maliyeti (4) orta skor (yavaş/dar).
        cost_score = clamp(1.0 - avg / 10.0, 0.0, 1.0)

        w = self.w
        score = (
            w["heading"] * heading_score
            + w["progress"] * progress_score
            + w["speed"] * speed_score
            + w["smooth"] * smooth_score
            + w["unknown"] * unknown_score
            + w["obstacle"] * cost_score
            + w["corridor"] * corridor_score
            - w["avoid"] * avoid  # sarıdan uzaklaş (smooth kaçınma; turuncu hariç)
        )
        cand.heading = heading_score
        cand.progress = progress_score
        cand.speed = speed_score
        cand.smooth = smooth_score
        cand.unknown = unknown_score
        cand.swept_cost = avg
        cand.corridor = corridor_score
        cand.avoid = avoid
        cand.score = score
        return cand

    # --- Planlama -------------------------------------------------------------------

    def plan(
        self,
        goal_body: Tuple[float, float],
        costmap: CostMap,
        last_cmd: Optional[DwaCommand],
        corridor: Optional[Dict[str, Any]] = None,
        *,
        allow_heading_align: bool = True,
        yaw_rate_limit_deg_s: Optional[float] = None,
    ) -> DwaCommand:
        """Tüm adayları skorlar; en iyisini döner. Tümü çarpışırsa recovery().

        ``corridor`` (opsiyonel): aktif koridor bilgisi (``active`` +
        ``center_lateral_m``); varsa skorlamaya corridor bias terimi eklenir.
        ``None`` (varsayılan) eski davranışı korur (test uyumlu).
        """
        # P2 may impose a call-scoped yaw ceiling below the final high-yaw
        # safety threshold.  DWA then evaluates the forward arc that can
        # actually be executed instead of selecting a 25-50 deg/s trajectory
        # which the final envelope converts into a stationary pivot.  Restore
        # every shared-planner value so P1 behavior remains byte-for-byte
        # configured by its original limits on the next call.
        if yaw_rate_limit_deg_s is not None:
            try:
                limit_deg = float(yaw_rate_limit_deg_s)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError(
                    "yaw_rate_limit_deg_s must be finite and positive"
                ) from exc
            if not math.isfinite(limit_deg) or limit_deg <= 0.0:
                raise ValueError("yaw_rate_limit_deg_s must be finite and positive")
            limit = min(self.max_yaw_rate, math.radians(limit_deg))
            original = (
                self.max_yaw_rate,
                self.recovery_yaw,
                self.align_yaw_rate,
            )
            self.max_yaw_rate = limit
            self.recovery_yaw = min(self.recovery_yaw, limit)
            self.align_yaw_rate = min(self.align_yaw_rate, limit)
            try:
                return self.plan(
                    goal_body,
                    costmap,
                    last_cmd,
                    corridor=corridor,
                    allow_heading_align=allow_heading_align,
                )
            finally:
                (
                    self.max_yaw_rate,
                    self.recovery_yaw,
                    self.align_yaw_rate,
                ) = original

        self._corridor = corridor
        self.last_cmd = last_cmd
        goal = (float(goal_body[0]), float(goal_body[1]))
        goal_bearing = math.atan2(goal[1], goal[0])
        # Büyük ilk heading hatasında minimum ileri itkiyle geniş yay çizmek
        # yerine önce yerinde hizalan. Eşik 0 ise geriye uyumlu/devre dışı.
        align_threshold = self.align_before_drive
        # 42° sınırında ileri/hizalan döngüsünü önle: bir kez hizalanmaya
        # başladıysak yaklaşık 23° altına inene kadar aynı fazda kal.
        if last_cmd is not None and last_cmd.action.startswith("dwa_align_waypoint"):
            align_threshold *= 0.55
        if (
            bool(allow_heading_align)
            and align_threshold > 0.0
            and abs(goal_bearing) >= align_threshold
        ):
            yaw_mag = max(self.min_turn_rate, self.align_yaw_rate)
            yaw_sign_source = goal_bearing
            # Tam arka eksende atan2 açısı +pi/-pi arasında, santimetrelik
            # pose gürültüsüyle işaret değiştirir. Önceki hizalama yönünü bu
            # belirsiz bölgede korumazsak tekne her tick sağ/sol komut arasında
            # kalır ve gerçekte hiç dönemeden WP üzerinde bekler. Hedef yan veya
            # ön yarım düzleme çıktığında bearing yeniden tek otoritedir.
            if (
                last_cmd is not None
                and last_cmd.action.startswith("dwa_align_waypoint")
                and abs(goal_bearing) >= math.radians(150.0)
                and abs(float(last_cmd.yaw_rate)) > 1e-9
            ):
                yaw_sign_source = float(last_cmd.yaw_rate)
            yaw = math.copysign(
                min(self.max_yaw_rate, yaw_mag), yaw_sign_source
            )
            return DwaCommand(
                vx=0.0,
                yaw_rate=yaw,
                action=(
                    "dwa_align_waypoint_right"
                    if yaw > 0.0 else "dwa_align_waypoint_left"
                ),
                score=0.0,
            )
        best: Optional[Candidate] = None
        for cand in self.candidates(goal, last_cmd):
            cand = self.score_candidate(cand, goal, last_cmd, costmap)
            if best is None or cand.score > best.score:
                best = cand
        if best is None or best.collides or not math.isfinite(best.score):
            return self.recovery(costmap, last_cmd, goal_body=goal)

        # KAÇINMA YÖN HİSTEREZİSİ (chatter önleme): sabit engelde v=0/pivot
        # ile v=0.7/arc arasındaki hızlı geçiş, en iyi adayın yaw işareti
        # değiştiğinde dahi eski yönü bir süre koruyarak bastırılır.
        # Yön değişimi "kararlı kanıt" ister: yeni yön en az latch tick boyunca
        # tekrar seçilirse latch kırılır. Yakın engel yoksa latch sıfırlanır.
        # KULLANICI GÖZLEMİ (2026-08-18): latch yönü goal bearing'e zıtsa
        # (parkur dışına çıkaracaksa) latch ANINDA kırılır — araç parkur içine
        # dönebilir. Aksi halde eski davranış (N tick kararlı kanıt) korunur.
        obstacle_near = self._nearest_obstacle_lateral(costmap) is not None
        if obstacle_near:
            best_dir = 1.0 if best.yaw_rate > 0.0 else (-1.0 if best.yaw_rate < 0.0 else 0.0)
            # Hedef bearing işareti: latch yönünün parkur dışına çıkıp
            # çıkmadığını belirler (hedef yönüne zıt = parkur dışı).
            goal_bearing_sign = 1.0 if goal[1] >= 0.0 else -1.0
            if best_dir == 0.0:
                # Düz aday: latch'e dokunma (ilerleme anında yön korunur).
                pass
            elif self._avoid_direction == 0.0:
                self._avoid_direction = best_dir
                self._avoid_ticks_remaining = self.avoid_direction_latch_ticks
            elif best_dir == self._avoid_direction:
                self._avoid_ticks_remaining = self.avoid_direction_latch_ticks
            else:
                # Zıt yön adayı üstün geldi: latch dolana kadar eski yön korunur.
                # AMA latch yönü hedefe zıtsa (parkur dışı) latch anında kırılır.
                if (self._avoid_ticks_remaining > 0
                        and self._avoid_direction * goal_bearing_sign >= 0.0):
                    self._avoid_ticks_remaining -= 1
                    best = self._best_in_direction(costmap, goal, last_cmd, self._avoid_direction, best)
                else:
                    # Kararlı kanıt veya parkur-dışı latch: yeni yön kabul edilir.
                    self._avoid_direction = best_dir
                    self._avoid_ticks_remaining = self.avoid_direction_latch_ticks
        else:
            self._avoid_direction = 0.0
            self._avoid_ticks_remaining = 0
        # NOT: PythonRobotics robot_stuck_flag_cons deseninin vx<0.1 AND
        # last_cmd.vx<0.1 dalı KALDIRILDI (T3, 2026-08-13): anlık iki-tick
        # hız düşüşü takılma değildir — approach_scale/yellow_brake ile doğal
        # yavaşlayan araç yanlışlıkla recovery'ye (spin) sokuluyordu (dur-devam).
        # Takılma tespiti artık ZAMAN tabanlıdır ve autonomy_node tarafında
        # yapılır (stuck_timeout_s + ground_speed integrali) — DwaPlanner saf
        # kalır, yalnızca "tüm adaylar çarpışıyor" durumunda recovery döner.
        return DwaCommand(
            vx=best.vx,
            yaw_rate=best.yaw_rate,
            action=f"dwa score={best.score:.3f} heading={best.heading:.2f} "
                   f"progress={best.progress:.2f} swept={best.swept_cost:.2f}",
            score=best.score,
        )

    def _best_in_direction(
        self,
        costmap: CostMap,
        goal: Tuple[float, float],
        last_cmd: Optional[DwaCommand],
        direction: float,
        fallback: Candidate,
    ) -> Candidate:
        """Latch'li yöndeki en iyi adayı döner (chatter yön stabilizasyonu).

        En iyi aday zıt yöne döndüyse ve latch dolmadıysa, latch'li yöndeki en
        iyi aday seçilir (hız/dönüş aynı yönde kalır). Latch'li yönde hiçbir
        güvenli aday yoksa fallback (zıt yön adayı) kullanılır — bu durumda
        engel tarafı gerçekten değişmiş olabilir.
        """
        best_in_dir: Optional[Candidate] = None
        for cand in self.candidates(goal, last_cmd):
            if cand.collides or not math.isfinite(cand.score):
                continue
            if direction > 0.0 and cand.yaw_rate <= 0.0:
                continue
            if direction < 0.0 and cand.yaw_rate >= 0.0:
                continue
            # ``candidates()`` returns raw, unscored candidates.  Comparing
            # their default score (0.0) would select the first yaw sample,
            # not the best safe candidate, defeating the latch's purpose and
            # potentially producing an unnecessarily slow/poor trajectory.
            cand = self.score_candidate(cand, goal, last_cmd, costmap)
            if cand.collides or not math.isfinite(cand.score):
                continue
            if best_in_dir is None or cand.score > best_in_dir.score:
                best_in_dir = cand
        if best_in_dir is None:
            return fallback
        return best_in_dir

    def recovery(self, costmap: CostMap, last_cmd: Optional[DwaCommand] = None, goal_body: Optional[Tuple[float, float]] = None) -> DwaCommand:
        """Tüm adaylar çarpışıyorsa ENGELLERDEN UZAKLAŞAN dönüş.

        Hedef yönelimli dönüş yanlıştı — hedefe dönerken dubaya da dönebiliyordu
        (debug: duba soldayken dwa_recovery_left dubaya çarptı). Doğru davranış:
        costmap'teki EN YAKIN engelin lateral'ine göre TERS yöne dön (dubadan
        uzaklaş). Engel yoksa sağa dön (eski davranış).
        """
        vx_eff = self.effective_max_speed()
        vx = min(self.recovery_vx, vx_eff)
        if last_cmd is not None:
            # Hızlanma limitine saygı (aniden hızlanma yok); etkin tavan kullanılır.
            vx = min(vx, clamp(last_cmd.vx + self.accel["max_accel_mps2"], 0.0, vx_eff))
        # En yakın engelin lateral'i (gövde frame) -> ters yöne dön (dubadan uzaklaş)
        # AMA hedefe doğru bileşeni de koru (yoksa araç waypoint'e dönmez).
        nearest_lateral = self._nearest_obstacle_lateral(costmap)
        if nearest_lateral is not None:
            # Engelden uzaklaşma yönü (engel solda -> sağa, sağda -> sola).
            obstacle_dir = -1.0 if nearest_lateral >= 0.0 else 1.0
            # Stabil P1/P2 tabanındaki sınırlı recovery hafızası: gürültülü
            # merkez geçişinde anında sağ/sol değiştirmez, fakat eski yönü
            # sonraki engel kümelerine taşıyıp parkur dışına da kilitlemez.
            if self._recovery_latch_remaining > 0 and self._recovery_direction != 0.0:
                obstacle_dir = self._recovery_direction
                self._recovery_latch_remaining -= 1
            else:
                self._recovery_direction = obstacle_dir
                self._recovery_latch_remaining = self.recovery_latch_ticks
            if goal_body is not None:
                # Hedef bearing yönü (DURUMA BAĞLI bileşim — kullanıcı gözlemi
                # 2026-08-18: "parkur dışından geri sokma" yön seçiminden kaynaklanıyor).
                # B8: TAM 4-kadran atan2 — max(goal[0], 0.1) hedef arkadayken
                # bearing'i ±90°'ye çekiyordu. Hedef arkadaysa ±180° -> doğru dönüş.
                goal_bearing = math.degrees(math.atan2(goal_body[1], goal_body[0]))
                goal_dir = 1.0 if goal_bearing >= 0.0 else -1.0
                if obstacle_dir * goal_dir > 0.0:
                    # Engelden uzaklaşma HEDEFLE AYNI YÖNDE -> güvenli kaçınma
                    # (engel parkur dışı tarafta). Engelden uzaklaş baskın (%60).
                    yaw = (0.6 * obstacle_dir * self.recovery_yaw
                           + 0.4 * clamp(goal_bearing, -self.recovery_yaw, self.recovery_yaw))
                else:
                    # Engel PARKUR İÇİ tarafta; engelden uzaklaşma hedefe zıt ve
                    # parkur DIŞINA çıkarır. HEDEF yönü baskın (%60) — araç parkur
                    # içinde kalarak engelden kaçar, dışarı savrulmaz.
                    yaw = (0.4 * obstacle_dir * self.recovery_yaw
                           + 0.6 * clamp(goal_bearing, -self.recovery_yaw, self.recovery_yaw))
            else:
                yaw = obstacle_dir * self.recovery_yaw
            # Ön şerit tüm araç genişliği boyunca kapalıysa ileri itki güvenli
            # değildir. Diferansiyel itkiyle yerinde dön, açıklık oluşunca DWA
            # normal ileri adaylarına geri döner.
            front_blocked = self._front_wall_blocked(costmap)
            if front_blocked:
                vx = 0.0
                yaw = obstacle_dir * max(self.recovery_yaw, self.min_turn_rate)
                action = "dwa_recovery_blocked_pivot_right" if yaw > 0 else "dwa_recovery_blocked_pivot_left"
            else:
                if 0.0 < abs(yaw) < self.min_turn_rate:
                    yaw = math.copysign(self.min_turn_rate, yaw)
                action = "dwa_recovery_right" if yaw > 0 else "dwa_recovery_left"
        else:
            self._recovery_direction = 0.0
            self._recovery_latch_remaining = 0
            # Engel yoksa hedefe dön (yoksa sağa).
            if goal_body is not None:
                # B8: 4-kadran atan2 (hedef arkadayken bearing ±180° -> doğru dönüş).
                goal_bearing = math.degrees(math.atan2(goal_body[1], goal_body[0]))
                yaw = clamp(goal_bearing * 0.5, -self.recovery_yaw, self.recovery_yaw)
                action = "dwa_recovery_right" if yaw > 0 else "dwa_recovery_left"
            else:
                yaw = self.recovery_yaw
                action = "dwa_recovery_right"
        return DwaCommand(vx=vx, yaw_rate=yaw, action=action, score=0.0)

    @staticmethod
    def _front_wall_blocked(costmap: CostMap) -> bool:
        """Yakın önde ayrı lidar kümelerinin oluşturduğu geniş duvar var mı?

        Şişirilmiş tek bir duba diski ``-0.75/0/+0.75`` örneklerinin üçünü de
        kapatabiliyordu; böylece tek duba duvar sanılıyor ve recovery yalnız
        yerinde pivota kilitleniyordu. Duvar kanıtı artık ham, ayrı sert lidar
        kümelerinden ve gerçek yanal yayılımdan gelir. Costmap çarpışma veto'su
        ile son güvenlik zarfı tekil engeller için etkin kalır.
        """

        laterals: List[float] = []
        for obstacle in getattr(costmap, "last_obstacles", []):
            if not isinstance(obstacle, dict) or not bool(
                obstacle.get("hard_obstacle", True)
            ):
                continue
            try:
                forward = float(
                    obstacle.get("forward_m", obstacle.get("x", 0.0))
                )
                lateral = float(
                    obstacle.get("lateral_m", obstacle.get("y", 0.0))
                )
            except (TypeError, ValueError, OverflowError):
                continue
            if not (math.isfinite(forward) and math.isfinite(lateral)):
                continue
            if 0.0 <= forward <= 3.0 and abs(lateral) <= 2.0:
                laterals.append(lateral)
        if len(laterals) < 3:
            return False
        return min(laterals) <= -0.6 and max(laterals) >= 0.6 and (
            max(laterals) - min(laterals)
        ) >= 1.5

    def _nearest_obstacle_lateral(self, costmap: CostMap) -> Optional[float]:
        """Costmap'teki en yakın ÖN engel hücresinin gövde lateral'i (m), yoksa None.

        Engelli hücreler: `inflated` > 0 (şişirilmiş engel). Bot merkezinden
        (n/2, n/2) YALNIZCA ÖN yarım kürede (forward >= 0, gövde-frame) en
        yakın engeli bulur, cell_to_world ile lateral'i döner.

        Ön-yarım-küre filtresi: 10m arama karesi arkayı da kapsıyordu; arkadaki
        engeller recovery'yi yanlış yöne çekiyordu (sarı öndeyken en yakın
        engel neredeyse bot merkezinde lateral bulunabiliyordu). Araç geri
        gitmez (vx >= 0), dolayısıyla arkadaki engel kaçınma yönünü belirlememeli.
        """
        center_col = costmap.n // 2
        center_row = costmap.n // 2
        best_lateral: Optional[float] = None
        best_dist = float("inf")
        radius_cells = max(1, int(round(10.0 / self.cell_m)))  # 10m arama
        # B14: yarım küre filtresi. world_to_cell(costmap.py:129-130):
        #   col = floor(forward/cell + n/2)  -> forward artışı = dc (col) artışı
        #   row = floor(-lateral/cell + n/2) -> lateral artışı = -dr (row) artışı
        # Eski kod `for dr in range(0, ...)` kullanıyordu — dr>=0 "lateral <= 0"
        # (sağ taraf) demekti, ön yarım küre DEĞİL. Doğru ön filtresi dc >= 0.
        for dc in range(0, radius_cells + 1):  # ön yarım küre: dc >= 0 (forward >= 0)
            for dr in range(-radius_cells, radius_cells + 1):
                c = center_col + dc
                r = center_row + dr
                if not (0 <= c < costmap.n and 0 <= r < costmap.n):
                    continue
                # S3 (parkur2_analysis): yalnız SARI/GERÇEK engel (>=8) sayılır —
                # turuncu koridor dubası (3) engel değil. Eski `inflated > 0`
                # turuncuyu da engel sayıp recovery'yi koridor DIŞINA döndürüyordu.
                if costmap.inflated[r][c] < COST_YELLOW:
                    continue
                dist = math.hypot(dc * self.cell_m, dr * self.cell_m)
                if dist < best_dist:
                    best_dist = dist
                    _, lateral = costmap.cell_to_world(c, r)
                    best_lateral = lateral
        return best_lateral

    # --- Yardımcılar -----------------------------------------------------------------

    def near_unknown(self, costmap: CostMap, radius_m: float = 4.0) -> bool:
        """Bot çevresinde (radius_m yarıçap) unknown maliyetli hücre var mı?

        Tespit: gövde-frame haritada bot merkezi (0,0) hücresi = (n/2, n/2).
        Çevresel taramada unknown maliyeti aranır; autonomy tick'inde
        ``slow_vx`` seçiminde kullanılır. Varsayılan radius 6m -> 4m:
        unknown'ın uzakta olması yavaşlama gerektirmez; 4m daha az hücre
        tarar (9x9=81 vs 13x13=169) — tick maliyetini düşürür.
        """
        center_col = costmap.n // 2
        center_row = costmap.n // 2
        radius_cells = max(1, int(round(radius_m / self.cell_m)))
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                c = center_col + dc
                r = center_row + dr
                if not (0 <= c < costmap.n and 0 <= r < costmap.n):
                    continue
                if costmap.cells[r][c] == COST_UNKNOWN:
                    return True
        return False


# ---------------------------------------------------------------------------
# Fonksiyonel API (testler ve autonomy entegrasyonu için)
# ---------------------------------------------------------------------------

def plan(
    goal_body: Tuple[float, float],
    costmap: CostMap,
    last_cmd: Optional[DwaCommand],
    **params: float,
) -> DwaCommand:
    """Tek çağrılık DWA: varsayılan parametrelerle bir plan yürütür.

    ``**params`` DwaPlanner yapıcı parametrelerini geçersiz kılar (örn.
    ``recovery_yaw_deg_s=35.0``). Durumsuz kullanım için idealdir; autonomy
    node'u kalıcı DwaPlanner örneği kullanır.
    """
    planner = DwaPlanner(**params)
    return planner.plan(goal_body, costmap, last_cmd)


def dynamic_window(last: Optional[DwaCommand], max_speed_mps: float, max_yaw_rate_deg_s: float, **params: float):
    """Bağımsız fonksiyon: dinamik pencere (hızlanma limitleri)."""
    planner = DwaPlanner(
        max_speed_mps=max_speed_mps,
        max_yaw_rate_deg_s=max_yaw_rate_deg_s,
        **params,
    )
    return planner.dynamic_window(last)


def recovery(costmap: CostMap, last_cmd: Optional[DwaCommand] = None, goal_body: Optional[Tuple[float, float]] = None, **params: float) -> DwaCommand:
    """Bağımsız fonksiyon: hedef yönelimli recovery (goal_body yoksa sağa)."""
    planner = DwaPlanner(**params)
    return planner.recovery(costmap, last_cmd, goal_body=goal_body)


def _wrap_rad(angle: float) -> float:
    """Radyan açıyı [-pi, pi] aralığına sarar."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
