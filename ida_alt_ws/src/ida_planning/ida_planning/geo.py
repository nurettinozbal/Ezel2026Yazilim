import math
from dataclasses import dataclass

EARTH_RADIUS_M = 6371000.0


@dataclass
class LocalPoint:
    x: float
    y: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2.0) ** 2
    )
    return EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2_rad)
    y = (
        math.cos(lat1_rad) * math.sin(lat2_rad)
        - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    )
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def latlon_to_local_m(origin_lat: float, origin_lon: float, lat: float, lon: float) -> LocalPoint:
    dlat = math.radians(lat - origin_lat)
    dlon = math.radians(lon - origin_lon)
    x_north = dlat * EARTH_RADIUS_M
    y_east = dlon * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return LocalPoint(x=x_north, y=y_east)


def local_m_to_latlon(origin_lat: float, origin_lon: float, x_north: float, y_east: float):
    lat = origin_lat + math.degrees(x_north / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(y_east / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat))))
    return lat, lon


def world_to_body(dx_north: float, dy_east: float, heading_deg: float) -> LocalPoint:
    """Convert world N/E offset into body forward/right offset."""
    h = math.radians(heading_deg)
    forward = math.cos(h) * dx_north + math.sin(h) * dy_east
    right = -math.sin(h) * dx_north + math.cos(h) * dy_east
    return LocalPoint(x=forward, y=right)


def body_to_world(forward: float, right: float, heading_deg: float) -> LocalPoint:
    h = math.radians(heading_deg)
    north = math.cos(h) * forward - math.sin(h) * right
    east = math.sin(h) * forward + math.cos(h) * right
    return LocalPoint(x=north, y=east)


def heading_error_deg(heading_deg: float, target_bearing_deg: float) -> float:
    """Signed heading error (deg) needed to reach the target bearing.

    Positive means the boat must turn right (starboard). The result is wrapped
    into [-180, 180] so a 10 deg target behind the bow reads as 170 deg error.
    """
    return normalize_angle_deg(target_bearing_deg - heading_deg)


def bearing_delta_deg(from_bearing_deg: float, to_bearing_deg: float) -> float:
    """Signed angular difference from one bearing to another, wrapped [-180, 180]."""
    return normalize_angle_deg(to_bearing_deg - from_bearing_deg)


def angular_separation_deg(bearing_a_deg: float, bearing_b_deg: float) -> float:
    """Absolute angular separation between two bearings, always in [0, 180]."""
    return abs(normalize_angle_deg(bearing_b_deg - bearing_a_deg))


def perpendicular_distance_m(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    """Perpendicular distance from point P to the infinite line through A and B.

    The result is unsigned; use signed_crossing_side_m when a side sign is needed.
    """
    abx = bx - ax
    aby = by - ay
    denom = math.hypot(abx, aby)
    if denom < 1e-12:
        return math.hypot(px - ax, py - ay)
    return abs(abx * (ay - py) - aby * (ax - px)) / denom


def signed_crossing_side_m(boat_n: float, boat_e: float, left_n: float, left_e: float, right_n: float, right_e: float) -> float:
    """Signed side of the boat relative to the right->left gate line.

    The gate line is defined from the right buoy to the left buoy. A positive
    result means the boat is on the left side of that line (i.e. closer to the
    left buoy side), negative means the right side. Units are meters (cross
    product / line length).
    """
    dir_n = left_n - right_n
    dir_e = left_e - right_e
    denom = math.hypot(dir_n, dir_e)
    if denom < 1e-12:
        return 0.0
    # Cross product of (dir) x (boat - right_buoy); positive => left side.
    return (dir_n * (boat_e - right_e) - dir_e * (boat_n - right_n)) / denom
