"""Duba/enme temas sayacı (şartname: dubaya çarpma kriteri).

Şartname (src/_pdf_ozet.txt:840-843): her bir çarpma sayılır; aynı dubaya
30 saniye SÜREKLİ temas durumunda 2 defa çarpmış sayılır. Temas eşiği
şartnamede tanımsızdır (hakem + kamera kaydı), bu yüzden ``contact_radius_m``
parametrik tutulur.

``ContactCounter`` her duba/engel için bir ``ContactEvent`` tutar. Temas
adayları üç kaynaktan üretilir ve ``source`` etiketi hangi algıdan geldiğini
korur:
- lidar (``/perception/obstacles``): gövde (forward_m, lateral_m) mesafesi.
- camera (``/perception/buoys``): yalnız ``distance`` + ``confidence`` +
  renk (turuncu/sarı) — gerçek kamerada forward/lateral YOK
  (yolo_camera_node.py:209-227).
- camera_fused (sim): detection'da gövde (forward_m, lateral_m) varsa
  (perception_sim_node.py:98-108).

Yanlış-pozitif bastırma: ``min_confidence``, ``max_lateral_m``,
``max_distance_m`` filtreleri + ``drop_after_s``. Çifte sayım: ``_buoy_key``
(id öncelikli, lateral_m yedeği — planner.py:187-192 stratejisi) + lateral
eşleşmesiyle lidar+kamera aynı temasa indirgenir.

Determinizm: ``now`` parametresi her update çağrısında dışarıdan verilir
(ROS2 clock ya da sabit test zamanı); time.time KULLANILMAZ.
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

# Algı kaynağı etiketleri (state/debug JSON'da görünür).
SOURCE_LIDAR = "lidar"
SOURCE_CAMERA = "camera"
SOURCE_CAMERA_FUSED = "camera_fused"
VALID_SOURCES = (SOURCE_LIDAR, SOURCE_CAMERA, SOURCE_CAMERA_FUSED)

# Temas adayı olarak sayılan renkler (turuncu kenar + sarı engel).
CONTACT_COLORS = {"orange", "yellow"}


@dataclass
class ContactEvent:
    """Tek bir duba/engel için süregelen temas kaydı.

    ``key``: ``_buoy_key`` ile üretilen kararlı duba anahtarı (id öncelikli,
    lateral_m yedeği). ``counted`` 1 ya da 2'dir: ilk temas 1, ``last_t -
    start_t >= sustained_contact_s`` ise 2. Temas ``last_t``'den beri
    ``drop_after_s``'den uzun kesilirse olay kapatılır (``closed=True``).
    """

    key: str
    source: str
    parkur: int
    start_t: float
    last_t: float
    color: str = "orange"
    sustained: bool = False
    counted: int = 1  # 1 blok=2, 2 blok=4, ... (her 30 sn'lik sürekli blok +2)
    closed: bool = False


@dataclass
class ContactStats:
    """Bir update dönüşü: toplam temas ve kaynak/renk kırılımları.

    ``total`` P1'de Ç1, P2'de Ç2 olarak kullanılır. ``by_color`` turuncu
    (kenar) ve sarı (engel) ayrımını korur; ``by_source`` hangi algının
    temasa katkı verdiğini gösterir (izleme/itiraz için).
    """

    total: int = 0
    by_source: Dict[str, int] = field(default_factory=dict)
    by_color: Dict[str, int] = field(default_factory=dict)
    active_events: List[ContactEvent] = field(default_factory=list)


def _buoy_key(buoy: Dict[str, Any]) -> str:
    """Duba/engel için kararlı anahtar: id varsa id, yoksa lateral_m slotu.

    planner.py:187-192 ile aynı strateji: explicit ``id`` güvenilirdir;
    id yoksa (gerçek kamera) 0.5 m lateral slotu kullanılır. Key, duba
    kaynaktan (lidar/kamera) bağımsız olduğu için çifte sayım bastırılır.
    """
    buoy_id = buoy.get("id")
    if buoy_id is not None:
        return str(buoy_id)
    return f"L{round(float(buoy.get('lateral_m', 0.0)) * 2.0) / 2.0:.1f}"


def _obstacle_key(obstacle: Dict[str, Any]) -> str:
    """Lidar engeli anahtarı: id varsa id, yoksa lateral_m slotu (duba ile aynı).

    Lidar engeli ve kamera detektörü AYNI dubayı görüyorsa (ör. sarı engel)
    aynı ``_buoy_key`` üretilir -> tek temas sayılır. fark 0.5 m slotla
    sınırlandığı için geometrik anahtarlar çakışabilir; bu bilinçli bir
    toleranstır (kamera yanal konum veremez).
    """
    obstacle_id = obstacle.get("id")
    if obstacle_id is not None:
        return str(obstacle_id)
    return f"L{round(float(obstacle.get('lateral_m', 0.0)) * 2.0) / 2.0:.1f}"


def _camera_forward_m(buoy: Dict[str, Any], min_distance_m: float) -> float:
    """Kamera (yalnız distance) için kabul edilebilir forward/lateral yedeği.

    Gerçek kamera gövde konumu yayınlamaz; bu yüzden mesafenin tamamı
    forward olarak kabul edilir ve lateral ``0.0`` sayılır. Böylece
    ``max_lateral_m`` filtresi kamerayı dışlamaz, sadece ``max_distance_m``
    ve ``contact_radius_m`` uygulanır. ``distance`` negatif/NaN ise
    ``min_distance_m`` döner (aday elenir).
    """
    distance = buoy.get("distance")
    try:
        value = float(distance)
    except (TypeError, ValueError):
        return min_distance_m
    if math.isnan(value) or value < 0.0:
        return min_distance_m
    return value


def _safe_float(value: Any, default: float = 0.0) -> float:
    """float'a çevirir; NaN/Inf/çevrilemez değerlerde ``default`` döner."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(result) or math.isinf(result):
        return default
    return result


class ContactCounter:
    """Temasları sayar; P1/P2 ayrımını ``update(parkur)`` ile yapar.

    P1: yalnız turuncu kenar dubaları temas adayıdır. P2: turuncu kenar + sarı
    engel dubaları ayrı sayılır ama ``ContactStats.total`` ikisini toplar
    (şartname Ç2 tek sayacı, kenar/engel ayrımı YOK — yalnız izleme amaçlı
    kırılım tutulur).

    Süreklilik modeli: her anahtar için ``ContactEvent``. Temas sürerse
    ``last_t`` güncellenir; ``now - last_t > drop_after_s`` ise olay kapatılır.
    ``last_t - start_t >= sustained_contact_s`` ise ``counted=2``.
    """

    def __init__(
        self,
        contact_radius_m: float = 1.6,
        sustained_contact_s: float = 30.0,
        drop_after_s: float = 5.0,
        max_lateral_m: float = 15.0,
        max_distance_m: float = 3.0,
        min_confidence: float = 0.35,
        include_yellow_in_p2: bool = True,
    ) -> None:
        self.contact_radius_m = max(0.0, float(contact_radius_m))
        self.sustained_contact_s = max(0.0, float(sustained_contact_s))
        self.drop_after_s = max(0.0, float(drop_after_s))
        self.max_lateral_m = max(0.0, float(max_lateral_m))
        self.max_distance_m = max(0.0, float(max_distance_m))
        self.min_confidence = float(min_confidence)
        self.include_yellow_in_p2 = bool(include_yellow_in_p2)
        self._events: Dict[str, ContactEvent] = {}
        # Kapalı (bitmiş) olayların geçmişi: aynı key'li YENİ temas eski kapalı
        # olayın counted değerini EZMEZ (C2: 30sn blok + kopma + 30sn blok -> 4).
        self._history: List[ContactEvent] = []

    def reset(self) -> None:
        """Tüm temas olaylarını temizler; parkur geçişlerinde kullanılır.

        P1 çarpmaları P2'ye sızmaz: transition_to_p2/transition_to_p3
        sırasında çağrılır (autonomy_node.py:270-285).
        """
        self._events.clear()
        self._history.clear()

    def get_events(self) -> List[ContactEvent]:
        """Açık (devam eden) temas olaylarının listesi (zamana göre sıralı)."""
        return sorted(
            [e for e in self._events.values() if not e.closed],
            key=lambda e: e.start_t,
        )

    def get_events_all(self) -> List[ContactEvent]:
        """Açık + kapalı (geçmiş) tüm olaylar (izleme/CSV için)."""
        return sorted(
            list(self._events.values()) + list(self._history),
            key=lambda e: e.start_t,
        )

    def update(self, buoys: List[Dict[str, Any]], obstacles: List[Dict[str, Any]], parkur: int, now: float) -> ContactStats:
        """Mevcut algı karesiyle temas sayacını günceller.

        Args:
            buoys: ``/perception/buoys`` detections (kamera/sim; renk + mesafe).
            obstacles: ``/perception/obstacles`` (lidar; forward/lateral).
            parkur: 1 ya da 2 (P1: yalnız turuncu; P2: turuncu + sarı engel).
            now: deterministik zaman damgası (ROS2 clock ya da test değeri).

        Returns:
            Toplam temas ve kaynak/renk kırılımları. ``total`` P1 için Ç1,
            P2 için Ç2'ye karşılık gelir.
        """
        now = max(0.0, float(now))
        seen_keys: set[str] = set()
        stats = ContactStats(by_source={s: 0 for s in VALID_SOURCES}, by_color={"orange": 0, "yellow": 0})

        # --- Aday toplama (kaynak etiketiyle) -------------------------------
        for obstacle in obstacles:
            candidate = self._candidate_from_obstacle(obstacle, parkur, now)
            if candidate is not None:
                self._merge_or_create(candidate, stats, seen_keys, parkur, now)

        for buoy in buoys:
            candidate = self._candidate_from_buoy(buoy, parkur, now)
            if candidate is not None:
                self._merge_or_create(candidate, stats, seen_keys, parkur, now)

        # --- Bayat (kesilmiş) olayları kapat ve geçmişe taşı -----------------
        for key in list(self._events.keys()):
            event = self._events[key]
            if event.closed:
                continue
            if event.key not in seen_keys and now - event.last_t > self.drop_after_s:
                event.closed = True
                self._events.pop(key)
                self._history.append(event)

        stats.total = len(self._events)
        stats.by_source = {s: 0 for s in VALID_SOURCES}
        stats.by_color = {"orange": 0, "yellow": 0}
        active: List[ContactEvent] = []
        for event in self._events.values():
            active.append(event)
            stats.by_source[event.source] = stats.by_source.get(event.source, 0) + 1
            color = event.color if event.color in stats.by_color else "orange"
            stats.by_color[color] = stats.by_color.get(color, 0) + 1
        stats.active_events = active
        return stats

    # --- Aday üretimi -------------------------------------------------------

    def _candidate_from_obstacle(
        self, obstacle: Dict[str, Any], parkur: int, now: float
    ) -> Optional[Dict[str, Any]]:
        """Lidar engelini temas adayına çevirir (None = filtreye takıldı).

        Lidar engeli renk taşıyamaz: P1'de yalnız turuncu kenar sayılır, ama
        lidar rengi bilmediği için adayı her parkurda üretir; renk filtresi
        ``_merge_or_create``'te ``parkur``a göre uygulanır (P1'de sarı engel
        teması sayılmaz). Kamerayla aynı dubayı görüyorsa ``_obstacle_key``
        çakışır ve tek olay sayılır.
        """
        lateral = _safe_float(obstacle.get("lateral_m", obstacle.get("y", 0.0)))
        forward = _safe_float(obstacle.get("forward_m", obstacle.get("x", 0.0)))
        if abs(lateral) > self.max_lateral_m:
            return None
        distance = math.hypot(forward, lateral)
        if math.isnan(distance) or distance <= 0.0 or distance > self.max_distance_m:
            return None
        if distance > self.contact_radius_m:
            return None
        return {
            "key": _obstacle_key(obstacle),
            "source": SOURCE_LIDAR,
            "color": "yellow" if parkur == 2 else "orange",  # lidar renk bilmez
            "distance": distance,
            "lateral_m": lateral,
        }

    def _candidate_from_buoy(
        self, buoy: Dict[str, Any], parkur: int, now: float
    ) -> Optional[Dict[str, Any]]:
        """Kamera/sim duba deteksiyonunu temas adayına çevirir (None = filtre).

        Renk yalnız ``CONTACT_COLORS`` içindeyse aday olur (gerçek kamerada
        yalnız turuncu/sarı deteksiyon gelir; siyah/kırmızı/yeşil P3 hedefidir
        ve çarpma sayılmaz). Kamera yanal konum yayınlamadığından gövde
        mesafesi yalnız ``distance`` üzerinden hesaplanır; sim/fused
        deteksiyonda forward_m/lateral_m varsa gövde mesafesi kullanılır.
        """
        color = str(buoy.get("color", "")).lower()
        if color not in CONTACT_COLORS:
            return None
        if float(buoy.get("confidence", 0.0)) < self.min_confidence:
            return None

        forward_raw = buoy.get("forward_m")
        lateral_raw = buoy.get("lateral_m")
        if forward_raw is not None and lateral_raw is not None:
            # Sim/fused: gövde konumu verildi -> doğru mesafe ve lateral filtresi.
            forward = _safe_float(forward_raw)
            lateral = _safe_float(lateral_raw)
            if abs(lateral) > self.max_lateral_m:
                return None
            distance = math.hypot(forward, lateral)
            if math.isnan(distance) or distance <= 0.0 or distance > self.max_distance_m:
                return None
            if distance > self.contact_radius_m:
                return None
            source = SOURCE_CAMERA_FUSED
        else:
            # Gerçek kamera: yalnız distance; lateral yok -> 0 sayılır.
            distance = _camera_forward_m(buoy, self.max_distance_m)
            if distance > self.max_distance_m or distance > self.contact_radius_m:
                return None
            forward = distance
            lateral = 0.0
            source = SOURCE_CAMERA

        return {
            "key": _buoy_key(buoy),
            "source": source,
            "color": color,
            "distance": distance,
            "lateral_m": lateral,
        }

    # --- Olay birleştirme ----------------------------------------------------

    def _merge_or_create(
        self,
        candidate: Dict[str, Any],
        stats: ContactStats,
        seen_keys: set[str],
        parkur: int,
        now: float,
    ) -> None:
        """Adayı mevcut olaya birleştirir ya da yeni olay açar.

        Çifte sayım bastırma: aynı ``key`` hem lidar hem kameradan geliyorsa
        tek olay sayılır. Sarı engel yalnız ``include_yellow_in_p2`` ve
        ``parkur == 2`` iken P2'de sayılır; P1'de sarı aday yok sayılır
        (turuncu kenar filtresi).
        """
        color = candidate["color"]
        if color == "yellow" and not (parkur == 2 and self.include_yellow_in_p2):
            return
        if color == "orange" and parkur not in (1, 2):
            return

        key = candidate["key"]
        existing = self._events.get(key)
        if existing is None or existing.closed:
            # Yeni olay: ilk temas 1 çarpma sayılır (30 sn sürekli -> 2).
            # Aynı key'li KAPALI olay artık _events'te değil (_history'de),
            # dolayısıyla her yeni temas kendi counted değerini taşır.
            event = ContactEvent(
                key=key,
                source=candidate["source"],
                parkur=parkur,
                start_t=now,
                last_t=now,
                color=color,
                counted=1,
            )
            self._events[key] = event
            seen_keys.add(key)
            return

        # Mevcut olay: temas sürüyor (aynı duba yeni adayda görüldü).
        existing.last_t = now
        existing.source = candidate["source"]
        # counted, sustained_contact_s KATLARINA göre her karede yeniden
        # hesaplanır: 1 blok -> 2, 2 blok -> 4, 3 blok -> 6 (tek olayda
        # iki ayrı 30sn'lik blok = 4 çarpma). Blok sayısı floor(süre/eşik).
        if self.sustained_contact_s > 0.0:
            sustained_blocks = int((existing.last_t - existing.start_t) // self.sustained_contact_s)
            existing.sustained = sustained_blocks >= 1
            existing.counted = 1 if sustained_blocks < 1 else 2 * sustained_blocks
        else:
            # Eşik 0: her görülen temas anında sürekli sayılır (2 çarpma).
            existing.sustained = True
            existing.counted = 2
        seen_keys.add(key)

    def close_stale(self, now: float) -> None:
        """Kapalı olmayan olayları ``now`` anında kapatır (parkur sonu için).

        Parkur tamamlandığında kalan olayların (aktif temaslar) süreklilik
        penceresi kapanmalıdır; ``get_events_all``'da hâlâ görünürler ama
        ``closed=True`` işaretlenir (itiraz adayı olarak CSV'ye yazılabilir).
        """
        for key in list(self._events.keys()):
            event = self._events[key]
            if not event.closed:
                event.closed = True
                self._events.pop(key)
                self._history.append(event)

    def total_counted(self) -> int:
        """Şartname sayacı (Ç1/Ç2): geçmiş + yüksek-değerli AÇIK olayların toplamı.

        ``ContactStats.total`` anlık aktif temas sayısıdır; bu metot hakem
        mantığını izler: her kapalı (geçmiş) olay ``counted`` (1 ya da 2·blok)
        katkısı verir. ``counted >= 2`` olan AÇIK olaylar da eklenir — böylece
        30 sn'lik sürekli temas parkur bitmeden tahmini puana yansır (O1).
        """
        total = 0
        for event in self._events.values():
            if event.counted >= 2:
                total += event.counted
        for event in self._history:
            total += event.counted
        return total
