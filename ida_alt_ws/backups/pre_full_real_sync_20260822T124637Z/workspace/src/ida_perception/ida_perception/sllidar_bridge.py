"""sllidar_ros2 köprüsü — saf Python modülü (rclpy'siz, test edilebilir).

sllidar_ros2 paketinin ``/scan`` topic'inden gelen sensor_msgs/LaserScan
mesajını dict olarak alır, engel noktalarına çevirir ve ``ida_perception.rplidar``
kümeleme zincirine iletir. Böylece donanım sürücüsü (sllidar_node) ile işleme
mantığı (process_scan) tek çağrı noktasında birleşir; simülasyon kontratı
(ida_perception_sim) bozulmadan aynı /perception/obstacles JSON formatı korunur.
"""

import math
from typing import Any, Dict, List, Sequence, Tuple

from ida_perception.rplidar import process_scan


def transform_obstacles_to_base(
    obstacles: Sequence[Dict[str, Any]],
    *,
    sensor_forward_offset_m: float = 0.0,
    sensor_lateral_right_offset_m: float = 0.0,
) -> List[Dict[str, Any]]:
    """Translate sensor-frame clusters into the vehicle-centre body frame.

    The legacy perception convention is ``+x`` forward and ``+y`` right.
    Offsets are the sensor position measured from the vehicle centre in that
    same frame. Width is invariant under translation; distance is recomputed
    from the translated cluster centre.
    """
    forward_offset = float(sensor_forward_offset_m)
    lateral_offset = float(sensor_lateral_right_offset_m)
    if not math.isfinite(forward_offset) or not math.isfinite(lateral_offset):
        raise ValueError("lidar sensor offsets must be finite")

    transformed: List[Dict[str, Any]] = []
    for obstacle in obstacles:
        try:
            forward = float(obstacle["forward_m"]) + forward_offset
            lateral = float(obstacle["lateral_m"]) + lateral_offset
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(forward) or not math.isfinite(lateral):
            continue
        item = dict(obstacle)
        item["forward_m"] = forward
        item["lateral_m"] = lateral
        item["distance"] = math.hypot(forward, lateral)
        transformed.append(item)
    return transformed


def obstacles_to_raw_clusters(
    obstacles: Sequence[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """Convert legacy right-positive obstacles to deterministic left-positive clusters."""
    ordered = sorted(
        obstacles,
        key=lambda item: (
            math.atan2(
                float(item.get("lateral_m", 0.0)),
                float(item.get("forward_m", 0.0)),
            ),
            float(item.get("distance", 0.0)),
            float(item.get("width_m", 0.0)),
            float(item.get("forward_m", 0.0)),
            float(item.get("lateral_m", 0.0)),
        ),
    )
    clusters: List[Dict[str, Any]] = []
    for index, obstacle in enumerate(ordered):
        clusters.append(
            {
                "id": f"s2_cluster_{index:03d}",
                "distance_m": float(obstacle.get("distance", 0.0)),
                "forward_m": float(obstacle.get("forward_m", 0.0)),
                # Legacy rplidar convention is right-positive. Fusion uses the
                # standard body convention left-positive.
                "lateral_left_m": -float(obstacle.get("lateral_m", 0.0)),
                "width_m": float(obstacle.get("width_m", 0.0)),
            }
        )
    return clusters


def _angle_deg(angle_min: float, increment: float, index: int) -> float:
    """LaserScan radyan dizinini 0..360 derece açıya çevirir.

    Negatif sonuçlar 360 ile normalize edilir (mod 360), böylece rplidar
    kümeleme zincirinin beklediği 0..359 aralığı garanti edilir.
    """
    return (math.degrees(angle_min + index * increment)) % 360.0


def transform_scan_angle_deg(
    raw_angle_deg: float,
    angle_offset_deg: float = 0.0,
    mirror_scan: bool = False,
) -> float:
    """Transform one sensor bearing into the vehicle-forward frame.

    ``mirror_scan`` is applied before the mounting yaw, matching the physical
    transform ``vehicle = (-sensor if mirrored else sensor) + yaw``.
    """
    angle = -float(raw_angle_deg) if mirror_scan else float(raw_angle_deg)
    return (angle + float(angle_offset_deg)) % 360.0


def inside_front_fov(angle_deg: float, front_fov_deg: float = 360.0) -> bool:
    """Return whether a vehicle-frame bearing is inside the forward sector."""
    fov = float(front_fov_deg)
    if not math.isfinite(fov) or not 0.0 < fov <= 360.0:
        return False
    if fov >= 360.0 - 1e-9:
        return True
    signed = (float(angle_deg) + 180.0) % 360.0 - 180.0
    return abs(signed) <= fov / 2.0 + 1e-9


def laserscan_to_points(
    scan: Dict[str, Any],
    angle_offset_deg: float = 0.0,
    mirror_scan: bool = False,
    front_fov_deg: float = 360.0,
    min_distance_m: float = 0.3,
    max_distance_m: float = 18.0,
) -> List[Tuple[float, float]]:
    """LaserScan dict'ini (angle_deg, distance_m) nokta listesine çevirir.

    Args:
        scan: sensor_msgs/LaserScan alanları — angle_min, angle_max,
            angle_increment, ranges (liste), range_min, range_max.
        angle_offset_deg: sensör montaj hattı sapması (derece, saat yönünün
            tersine pozitif). 0° araç ileri yönüdür.
        mirror_scan: sensör baş aşağı/ters açı yönünde monte edilmişse True.
        front_fov_deg: araç önü merkezli kabul edilen sektör; 360 filtreyi
            kapatır, 160 yalnız ±80° ön sektörü bırakır.
        min_distance_m: çok yakın (motor gövdesi vb.) noktalar elenir.
        max_distance_m: bu mesafenin üstündeki noktalar elenir (sürücü
            range_max değerinden bağımsız üst sınır).

    Returns:
        (angle_deg, distance_m) ikilileri; açı 0..359, araç ileri yönü 0°.
    """
    try:
        angle_min = float(scan["angle_min"])
        angle_max = float(scan.get("angle_max", angle_min))
        angle_increment = float(scan["angle_increment"])
        ranges = scan["ranges"]
        range_min = float(scan.get("range_min", 0.0))
        range_max = float(scan.get("range_max", max_distance_m))
    except (KeyError, TypeError, ValueError):
        # Eksik/bozuk alan: istisna fırlatma, boş nokta listesi dön.
        return []
    # Bozuk scan guard: artmayan/sıfır increment tüm noktaları tek açıya yığar.
    if not math.isfinite(angle_increment) or angle_increment <= 0.0:
        return []
    if not math.isfinite(angle_max) or angle_max <= angle_min:
        return []

    points: List[Tuple[float, float]] = []
    # len(ranges) güvenli döngü: increment uyuşmazlığında taşma/istisna yok.
    for i in range(len(ranges)):
        distance = ranges[i]
        if distance is None:
            continue
        try:
            dist_f = float(distance)
        except (TypeError, ValueError):
            continue
        # NaN/inf ve menzil dışı ölçümler elenir.
        if not math.isfinite(dist_f):
            continue
        if dist_f <= 0.0:
            continue
        if dist_f < range_min or dist_f > range_max:
            continue
        if dist_f < min_distance_m or dist_f > max_distance_m:
            continue
        angle = transform_scan_angle_deg(
            _angle_deg(angle_min, angle_increment, i),
            angle_offset_deg=angle_offset_deg,
            mirror_scan=mirror_scan,
        )
        if inside_front_fov(angle, front_fov_deg):
            points.append((angle, dist_f))
    return points


def points_to_raw_body_contract(
    points: List[Tuple[float, float]],
    sensor_forward_offset_m: float = 0.0,
    sensor_lateral_right_offset_m: float = 0.0,
    limit: int = 720,
) -> List[Dict[str, float]]:
    """Convert polar scan returns to the bounded raw UI/fusion body contract.

    ``laserscan_to_points`` uses the stack's vehicle bearing convention where
    positive angles point right.  The sensor-fusion raw contract deliberately
    uses ROS-style ``lateral_left_m``; the sign conversion happens here once.
    Even downsampling preserves the full angular sweep instead of keeping only
    one side of a dense scan.
    """
    if isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0:
        raise ValueError("limit must be a positive integer")
    if not math.isfinite(sensor_forward_offset_m) or not math.isfinite(
        sensor_lateral_right_offset_m
    ):
        raise ValueError("sensor offsets must be finite")
    selected = points
    if len(points) > limit:
        step = len(points) / limit
        selected = [points[min(len(points) - 1, int(index * step))] for index in range(limit)]
    result: List[Dict[str, float]] = []
    for angle_deg, distance_m in selected:
        if not math.isfinite(angle_deg) or not math.isfinite(distance_m):
            continue
        angle_rad = math.radians(angle_deg)
        forward = distance_m * math.cos(angle_rad) + sensor_forward_offset_m
        lateral_right = distance_m * math.sin(angle_rad) + sensor_lateral_right_offset_m
        result.append({"forward_m": forward, "lateral_left_m": -lateral_right})
    return result


def scan_to_obstacles(
    scan: Dict[str, Any],
    angle_offset_deg: float,
    lidar_range_m: float,
    min_distance_m: float,
    cluster_gap_m: float,
    min_cluster_points: int,
    max_angle_gap_deg: float,
    stamp: float,
    mirror_scan: bool = False,
    front_fov_deg: float = 360.0,
    sensor_forward_offset_m: float = 0.0,
    sensor_lateral_right_offset_m: float = 0.0,
) -> List[Dict[str, Any]]:
    """LaserScan dict'inden engel listesi üretir (tek çağrı noktası).

    ``laserscan_to_points`` ile menzil filtresi uygular, ardından
    ``ida_perception.rplidar.process_scan`` kümeleme zincirini çağırır.
    process_scan çıktısı zaten /perception/obstacles kontrat formatındadır:
    {distance, forward_m, lateral_m, width_m, stamp}.

    Args:
        scan: sensor_msgs/LaserScan alanları (dict olarak).
        angle_offset_deg: sensör montaj hattı sapması (derece).
        mirror_scan: açı eksenini yaw ofsetinden önce ters çevirir.
        front_fov_deg: araç önü merkezli izin verilen sektör.
        sensor_forward_offset_m: lidarın araç merkezinden ileri ofseti.
        sensor_lateral_right_offset_m: lidarın araç merkezinden sağ ofseti.
        lidar_range_m: üst menzil sınırı (process_scan'e aktarılır).
        min_distance_m: alt mesafe sınırı.
        cluster_gap_m: küme içi nokta arası maksimum mesafe.
        min_cluster_points: küme sayılması için gereken minimum nokta.
        max_angle_gap_deg: ardışık nokta arası maksimum açı boşluğu.
        stamp: tespit zaman damgası (node get_clock().now() geçer).

    Returns:
        README kontratına uygun engel sözlükleri listesi.
    """
    points = laserscan_to_points(
        scan,
        angle_offset_deg=angle_offset_deg,
        mirror_scan=mirror_scan,
        front_fov_deg=front_fov_deg,
        min_distance_m=min_distance_m,
        max_distance_m=lidar_range_m,
    )
    obstacles = process_scan(
        points,
        lidar_range_m=lidar_range_m,
        min_distance_m=min_distance_m,
        cluster_gap_m=cluster_gap_m,
        min_cluster_points=min_cluster_points,
        max_angle_gap_deg=max_angle_gap_deg,
        stamp=stamp,
    )
    return transform_obstacles_to_base(
        obstacles,
        sensor_forward_offset_m=sensor_forward_offset_m,
        sensor_lateral_right_offset_m=sensor_lateral_right_offset_m,
    )
