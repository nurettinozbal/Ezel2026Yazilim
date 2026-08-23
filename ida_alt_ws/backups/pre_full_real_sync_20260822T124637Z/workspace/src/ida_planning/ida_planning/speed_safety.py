"""Hıza bağlı güvenlik yarıçapı + hız sınırlama — saf fonksiyonlar.

idaws (kardeş repo) ``collision_avoidance_node._update_radii``'den taşındı:
sabit şişirme yarıçapı yerine, aracın anlık durma mesafesine bağlı güvenlik
yarıçapı kullanılır. Sabit yarıçap yüksek hızda engeli ancak çok kısa süre
kala fark ettiriyordu; hıza bağlı yarıçap tepki süresini korur.

Formüller (idaws collision_avoidance_node.py:125-142, 85-87):
    stopping = v * t_tepki + v^2 / (2 * a_yavaşlama)
    safety   = max(taban, min(taban + stopping, tavan))
    v_max    = -a*t + sqrt((a*t)^2 + 2*a*(tavan - taban))   [pozitif kök]

Tüm fonksiyonlar saf Python'dur (rclpy yok) — test_saf.py deseniyle test
edilir. ``max_decel_mps2 <= 0`` koruması: durma mesafesi sonsuz (anında
duramaz) — güvenli taraf.
"""

import math

# Varsayılanlar (autonomy.yaml varsayılanlarıyla aynı).
DEFAULT_BASE_SAFETY_M = 0.5      # duruştaki taban güvenlik yarıçapı (m)
DEFAULT_MAX_SAFETY_M = 10.0      # üst sınır (LiDAR menzili 18m'nin altında)
DEFAULT_REACTION_TIME_S = 1.0    # algı + komut gecikmesi (s)
DEFAULT_MAX_DECEL_MPS2 = 1.5     # etkin yavaşlama (m/s^2)


def stopping_distance(v: float, reaction_time_s: float, max_decel_mps2: float) -> float:
    """Anlık durma mesafesi: v*t_tepki + v^2/(2*a).

    v < 0 ise 0 (geri gitmiyoruz); a <= 0 ise sonsuz (anında duramaz).
    """
    if v <= 0.0:
        return 0.0
    if max_decel_mps2 <= 0.0:
        return float("inf")
    t = max(0.0, reaction_time_s)
    return v * t + (v * v) / (2.0 * max_decel_mps2)


def safety_radius(
    v: float,
    base_m: float,
    max_m: float,
    reaction_time_s: float,
    max_decel_mps2: float,
) -> float:
    """Hıza bağlı güvenlik yarıçapı: max(base, min(base + stopping, max_m)).

    max_m <= base ise base döner (tavan tabanın altına düşemez). v <= 0 ise base.
    """
    base = max(0.0, float(base_m))
    max_r = max(0.0, float(max_m))
    if max_r <= base:
        return base
    stop = stopping_distance(v, reaction_time_s, max_decel_mps2)
    return max(base, min(base + stop, max_r))


def speed_for_safety_radius(
    max_m: float,
    base_m: float,
    reaction_time_s: float,
    max_decel_mps2: float,
) -> float:
    """Sensör menzilinin izin verdiği azami hız.

    ``v*t + v^2/(2a) = (max_m - base)`` denkleminin pozitif kökü:
    ``v = -a*t + sqrt((a*t)^2 + 2*a*pay)`` (idaws:85-87). pay <= 0 ise 0.
    max_decel <= 0 ise sonsuz (limit yok) döner.
    """
    pay = float(max_m) - float(base_m)
    a = float(max_decel_mps2)
    t = float(reaction_time_s)
    # NaN/Inf koruması: girdilerden biri sonlu değilse limit yok (inf) dön —
    # sessiz NaN yayılımını engelle (güvenli taraf: hız sınırlanmaz).
    if not (math.isfinite(pay) and math.isfinite(a) and math.isfinite(t)):
        return float("inf")
    pay = max(0.0, pay)
    t = max(0.0, t)
    if a <= 0.0:
        return float("inf")
    at = a * t
    return -at + math.sqrt(at * at + 2.0 * a * pay)
