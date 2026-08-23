"""RPLidar tarama işleme — saf Python modülü (ida_perception).

``process_scan`` fonksiyonu (angle_deg, distance_m) nokta listesini alır ve
engel kümesi listesi üretir. Araç ileri yönü 0°'dir; açılar 0..359 derece
(saat yönünün tersine), mesafe metre cinsindendir.

Kümeleme mantığı: ardışık açıda birbirine ``cluster_gap_m``'den yakın olan
noktalar tek engel kabul edilir; yeterli noktası olmayan kümeler gürültü sayılıp
elenir. Sonuç formatı README /perception/obstacles kontratına uygundur:
[{"distance": m, "forward_m": m, "lateral_m": m, "width_m": m, "stamp": s}]
"""

import math
from typing import Any, Dict, List, Sequence, Tuple

# Ardışık ölçüm arası maksimum açı boşluğu (derece). Bu açıdan fazla boşluk
# varsa küme sonlanır (yeni engel başlar).
DEFAULT_MAX_ANGLE_GAP_DEG = 8.0


def _polar_to_body(angle_deg: float, distance_m: float) -> Tuple[float, float]:
    """Kutupsal (açı, mesafe) -> gövde (forward, lateral).

    Araç ileri yönü 0°; pozitif lateral sağa işaret eder.
    """
    theta = math.radians(angle_deg)
    forward = distance_m * math.cos(theta)
    lateral = distance_m * math.sin(theta)
    return forward, lateral


def _circular_mean_angle(angles_deg: Sequence[float]) -> float:
    """Açıların dairesel ortalamasını hesaplar (0..360).

    359° ve 1° gibi sarmalanmış açılar için lineer ortalama (180°) yanlış olur;
    birim vektörlerin toplamının atan2'si doğru sonucu verir (~0°).
    """
    if not angles_deg:
        return 0.0
    x = sum(math.cos(math.radians(a)) for a in angles_deg)
    y = sum(math.sin(math.radians(a)) for a in angles_deg)
    return math.degrees(math.atan2(y, x)) % 360.0


def _cluster_points(
    points: Sequence[Tuple[float, float]],
    cluster_gap_m: float,
    min_cluster_points: int,
    max_angle_gap_deg: float,
) -> List[List[Tuple[float, float]]]:
    """Açısal olarak ardışık noktaları mesafe yakınlığına göre kümelere ayırır.

    Kümeleme daireseldir: 358°..2° arası noktalar (0° civarı tek engel) ayrı
    kümelere bölünmez. En büyük açı boşluğundan başlanır ve o boşluk küme sınırı
    sayılır; böylece sarmalanmış (wrap) geçiş doğal olarak birleşik kalır.
    """
    valid = [
        (float(a), float(d))
        for a, d in points
        if math.isfinite(float(a)) and math.isfinite(float(d)) and float(d) > 0.0
    ]
    n = len(valid)
    if n == 0:
        return []
    valid.sort(key=lambda p: p[0])

    # Dairesel ardışık boşluklar: gaps[i] = valid[i-1] -> valid[i] (son -> ilk sarmalı dahil).
    gaps = [(valid[i][0] - valid[i - 1][0]) % 360.0 for i in range(n)]
    # En büyük boşluktan başla: wrap geçişi küme içinde kalır, kopuş sınırda olur.
    start = int(max(range(n), key=lambda i: gaps[i]))

    clusters: List[List[Tuple[float, float]]] = []
    current: List[Tuple[float, float]] = []
    prev: Tuple[float, float] | None = None

    for step in range(n):
        index = (start + step) % n
        angle, distance = valid[index]
        if prev is not None:
            angle_gap = gaps[(start + step) % n]
            if angle_gap > max_angle_gap_deg:
                if len(current) >= min_cluster_points:
                    clusters.append(current)
                current = []
            else:
                forward_p, lateral_p = _polar_to_body(prev[0], prev[1])
                forward_c, lateral_c = _polar_to_body(angle, distance)
                gap = math.hypot(forward_c - forward_p, lateral_c - lateral_p)
                if gap > cluster_gap_m:
                    if len(current) >= min_cluster_points:
                        clusters.append(current)
                    current = []
        current.append((angle, distance))
        prev = (angle, distance)

    if len(current) >= min_cluster_points:
        clusters.append(current)
    return clusters


def _cluster_span_deg(cluster: Sequence[Tuple[float, float]]) -> float:
    """Kümenin kapladığı açısal yay genişliği (derece).

    Dairesel formül: span = 360 - en_büyük_iç_boşluk. Sarmalanmış küme için de
    doğru sonuç verir (örn. [358°, 0°, 2°] -> 4°).
    """
    angles = sorted(a for a, _ in cluster)
    if len(angles) <= 1:
        return 0.0
    gaps = [(angles[i] - angles[i - 1]) % 360.0 for i in range(len(angles))]
    return 360.0 - max(gaps)


def process_scan(
    points: Sequence[Tuple[float, float]],
    lidar_range_m: float = 18.0,
    min_distance_m: float = 0.3,
    cluster_gap_m: float = 0.35,
    min_cluster_points: int = 3,
    max_angle_gap_deg: float = DEFAULT_MAX_ANGLE_GAP_DEG,
    stamp: float = 0.0,
) -> List[Dict[str, Any]]:
    """Tarama noktalarını engel listesine dönüştürür (saf fonksiyon).

    Args:
        points: (angle_deg, distance_m) ikilileri; açı 0..359, araç ileri yönü 0°.
        lidar_range_m: menzil dışı noktalar elenir.
        min_distance_m: çok yakın (motor gövdesi vb.) noktalar elenir.
        cluster_gap_m: küme içi nokta arası maksimum mesafe.
        min_cluster_points: küme sayılması için gereken minimum nokta.
        max_angle_gap_deg: ardışık nokta arası maksimum açı boşluğu.
        stamp: tespit zaman damgası (node get_clock().now() geçer).

    Returns:
        README kontratına uygun engel sözlükleri listesi.
    """
    # Menzil ve minimum mesafe filtrele.
    filtered: List[Tuple[float, float]] = []
    for angle, distance in points:
        if distance is None:
            continue
        try:
            angle_f = float(angle)
            dist_f = float(distance)
        except (TypeError, ValueError):
            continue
        if dist_f < min_distance_m or dist_f > lidar_range_m:
            continue
        if not (0.0 <= angle_f <= 360.0):
            continue
        filtered.append((angle_f, dist_f))

    # Açıya göre sırala (kümeleme ardışık açı gerektirir).
    filtered.sort(key=lambda p: p[0])

    clusters = _cluster_points(filtered, cluster_gap_m, min_cluster_points, max_angle_gap_deg)

    obstacles: List[Dict[str, Any]] = []
    for cluster in clusters:
        # Küme merkezi: dairesel ortalama açı + ortalama mesafe.
        mean_angle = _circular_mean_angle([p[0] for p in cluster])
        mean_distance = sum(p[1] for p in cluster) / len(cluster)
        forward, lateral = _polar_to_body(mean_angle, mean_distance)
        # Arka noktaları (forward < 0.3 m) ele: araç ileri yönüne bakıyoruz.
        if forward < 0.3:
            continue
        # Genişlik: kümenin dairesel açısal yayı * ortalama mesafe.
        span_deg = _cluster_span_deg(cluster)
        width_arc = mean_distance * math.radians(span_deg)
        obstacles.append(
            {
                "distance": round(mean_distance, 3),
                "forward_m": round(forward, 3),
                "lateral_m": round(lateral, 3),
                "width_m": round(width_arc, 3),
                "stamp": stamp,
            }
        )
    return obstacles


def demo_scan(
    lidar_range_m: float = 18.0,
    forward_targets: Sequence[Tuple[float, float]] = ((5.0, 0.0), (8.0, 30.0)),
) -> List[Tuple[float, float]]:
    """dry_run için örnek nokta bulutu üretir (deterministik).

    ``forward_targets``: (distance_m, bearing_deg) hedefleri. Her hedefin
    etrafına 3'er nokta eklenir; gerçek RPLidar gibi 0..359 açılarında.
    """
    scan: List[Tuple[float, float]] = []
    for distance, bearing in forward_targets:
        for offset in (-2, 0, 2):
            angle = (bearing + offset) % 360.0
            scan.append((angle, float(distance)))
    return scan
