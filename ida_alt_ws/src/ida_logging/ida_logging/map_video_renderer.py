"""OpenCV renderer for the referee local-costmap video.

The renderer is ROS-free so it can be unit-tested and reused by offline tools.
Body-frame coordinates follow the stack contract: +X forward/up, +Y right.
"""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
except ImportError:  # pragma: no cover - minimal build hosts
    cv2 = None
    np = None


FRAME_WIDTH = 960
FRAME_HEIGHT = 720
MAX_CELLS = 20_000


def renderer_available() -> bool:
    return cv2 is not None and np is not None


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return default
    return number if math.isfinite(number) else default


def body_to_pixel(
    forward_m: Any,
    lateral_right_m: Any,
    *,
    center_x: int,
    center_y: int,
    pixels_per_meter: float,
) -> tuple[int, int]:
    """Project body-frame metres to a heading-up image coordinate."""
    return (
        int(round(center_x + _finite(lateral_right_m) * pixels_per_meter)),
        int(round(center_y - _finite(forward_m) * pixels_per_meter)),
    )


def _text(frame: Any, text: str, x: int, y: int, *, scale: float = 0.46,
          color: tuple[int, int, int] = (220, 226, 235), thickness: int = 1) -> None:
    cv2.putText(frame, str(text)[:110], (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thickness, cv2.LINE_AA)


def _safe_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _draw_legend(frame: Any, x: int, y: int) -> None:
    entries = (
        ("Turuncu duba", (0, 145, 255)),
        ("Sari duba", (0, 230, 255)),
        ("Hedef", (255, 210, 45)),
        ("Lidar engeli", (65, 65, 245)),
        ("Sisirilmis alan", (45, 45, 125)),
        ("Bilinmeyen", (130, 135, 145)),
    )
    _text(frame, "HARITA GOSTERIMI", x, y, scale=0.5, color=(255, 255, 255))
    for index, (label, color) in enumerate(entries, start=1):
        yy = y + index * 20
        cv2.rectangle(frame, (x, yy - 10), (x + 14, yy + 3), color, -1)
        _text(frame, label, x + 22, yy + 1, scale=0.38)


def render_costmap_frame(
    *,
    stamp: float,
    telemetry: Mapping[str, Any],
    autonomy: Mapping[str, Any],
    command: Mapping[str, Any],
    cells: Sequence[Any],
    costmap_meta: Mapping[str, Any],
    obstacles: Sequence[Any],
    buoys: Sequence[Any],
    score: Mapping[str, Any],
    width: int = FRAME_WIDTH,
    height: int = FRAME_HEIGHT,
) -> Any:
    """Render one deterministic BGR frame for ``map.mp4``."""
    del score  # Reserved for a future referee overlay; kept in the stable API.
    if not renderer_available():
        raise RuntimeError("map.mp4 requires python3-opencv and python3-numpy")
    if width < 640 or height < 480:
        raise ValueError("map video frame must be at least 640x480")

    frame = np.full((height, width, 3), (15, 18, 24), dtype=np.uint8)
    map_left, map_top = 24, 66
    map_size = min(height - 94, width - 330)
    map_right, map_bottom = map_left + map_size, map_top + map_size
    center_x = (map_left + map_right) // 2
    center_y = (map_top + map_bottom) // 2
    size_m = min(100.0, max(5.0, _finite(costmap_meta.get("size_m"), 30.0)))
    cell_m = min(5.0, max(0.05, _finite(costmap_meta.get("cell_m"), 0.25)))
    pixels_per_meter = map_size / size_m
    cell_px = max(2, int(math.ceil(cell_m * pixels_per_meter)))

    cv2.rectangle(frame, (map_left, map_top), (map_right, map_bottom), (24, 29, 38), -1)
    for metres in range(-int(size_m // 2), int(size_m // 2) + 1, 5):
        px, _ = body_to_pixel(0.0, metres, center_x=center_x, center_y=center_y,
                              pixels_per_meter=pixels_per_meter)
        _, py = body_to_pixel(metres, 0.0, center_x=center_x, center_y=center_y,
                              pixels_per_meter=pixels_per_meter)
        if map_left <= px <= map_right:
            cv2.line(frame, (px, map_top), (px, map_bottom), (46, 52, 63), 1)
        if map_top <= py <= map_bottom:
            cv2.line(frame, (map_left, py), (map_right, py), (46, 52, 63), 1)

    valid_cells = 0
    for cell in list(cells)[:MAX_CELLS]:
        if not isinstance(cell, (list, tuple)) or len(cell) < 4:
            continue
        forward = _finite(cell[0], float("nan"))
        lateral = _finite(cell[1], float("nan"))
        cost = _finite(cell[2])
        tag = str(cell[3]).lower()[:32]
        if not (math.isfinite(forward) and math.isfinite(lateral)):
            continue
        px, py = body_to_pixel(forward, lateral, center_x=center_x, center_y=center_y,
                               pixels_per_meter=pixels_per_meter)
        if not (map_left <= px <= map_right and map_top <= py <= map_bottom):
            continue
        if tag == "orange":
            color = (0, 145, 255)
        elif tag == "yellow":
            color = (0, 230, 255)
        elif tag == "goal" or int(cost) == 3:
            color = (255, 210, 45)
        elif tag == "inflated":
            color = (45, 45, 125)
        elif tag in ("unknown", "hard_unknown") or int(cost) == 4:
            color = (130, 135, 145)
        else:
            color = (65, 65, 245) if cost >= 8 else (105, 110, 125)
        radius = max(2, cell_px // 2)
        cv2.rectangle(frame, (px - radius, py - radius), (px + radius, py + radius), color, -1)
        valid_cells += 1

    # Actual footprint: 1.18 m long and 0.76 m wide.
    half_width = max(7, int(0.38 * pixels_per_meter))
    half_length = max(13, int(0.59 * pixels_per_meter))
    cv2.rectangle(frame, (center_x - half_width, center_y - half_length),
                  (center_x + half_width, center_y + half_length), (235, 235, 235), 2)
    cv2.arrowedLine(frame, (center_x, center_y), (center_x, center_y - half_length - 18),
                    (80, 235, 150), 2, tipLength=0.35)

    vx = _finite(command.get("vx"))
    vy = _finite(command.get("vy"))
    magnitude = math.hypot(vx, vy)
    if magnitude > 1e-3:
        arrow_m = min(4.0, max(1.2, magnitude * 3.0))
        cmd_x, cmd_y = body_to_pixel(vx / magnitude * arrow_m, vy / magnitude * arrow_m,
                                     center_x=center_x, center_y=center_y,
                                     pixels_per_meter=pixels_per_meter)
        cv2.arrowedLine(frame, (center_x, center_y), (cmd_x, cmd_y), (255, 225, 80), 3,
                        tipLength=0.22)

    cv2.rectangle(frame, (map_left, map_top), (map_right, map_bottom), (95, 110, 130), 1)
    _text(frame, "+X ILERI", center_x - 34, map_top - 10, color=(80, 235, 150))
    _text(frame, "+Y SAG", map_right - 57, center_y - 8, color=(80, 235, 150))
    _text(frame, "IDA LOCAL COSTMAP KAYDI", 24, 34, scale=0.72,
          color=(245, 248, 252), thickness=2)
    _text(frame, f"ROS stamp: {_finite(stamp):.3f}", 650, 34, scale=0.45,
          color=(160, 172, 190))

    panel_x = map_right + 22
    _text(frame, "GOREV", panel_x, 84, scale=0.54, color=(255, 255, 255))
    _text(frame, f"Durum: {autonomy.get('state', '-')}", panel_x, 112)
    _text(frame, f"WP: {autonomy.get('current_waypoint', '-')}", panel_x, 136)
    action = str(autonomy.get("action", "-"))
    _text(frame, f"Karar: {action[:30]}", panel_x, 160, scale=0.4)
    if len(action) > 30:
        _text(frame, action[30:60], panel_x, 181, scale=0.4)

    _text(frame, "HAREKET", panel_x, 218, scale=0.54, color=(255, 255, 255))
    _text(frame, f"vx: {vx:+.2f} m/s", panel_x, 246)
    _text(frame, f"vy: {vy:+.2f} m/s", panel_x, 270)
    _text(frame, f"yaw: {_finite(command.get('yaw_rate')):+.2f} rad/s", panel_x, 294)
    _text(frame, "TELEMETRI", panel_x, 331, scale=0.54, color=(255, 255, 255))
    _text(frame, f"Lat: {_finite(telemetry.get('lat')):.7f}", panel_x, 359, scale=0.42)
    _text(frame, f"Lon: {_finite(telemetry.get('lon')):.7f}", panel_x, 382, scale=0.42)
    _text(frame, f"Heading: {_finite(telemetry.get('heading_deg')):.1f} deg", panel_x, 405)
    _text(frame, f"Hiz: {_finite(telemetry.get('ground_speed')):.2f} m/s", panel_x, 429)
    _text(frame, "ALGI", panel_x, 466, scale=0.54, color=(255, 255, 255))
    _text(frame, f"Costmap hucre: {valid_cells}", panel_x, 494)
    _text(frame, f"Lidar engel: {len(_safe_list(obstacles))}", panel_x, 518)
    _text(frame, f"Duba: {len(_safe_list(buoys))}", panel_x, 542)
    _text(frame, f"Harita: {size_m:.0f} x {size_m:.0f} m", panel_x, 566)
    if not valid_cells:
        _text(frame, "COSTMAP VERISI BEKLENIYOR", map_left + 90, center_y,
              scale=0.72, color=(80, 100, 235), thickness=2)
    _draw_legend(frame, panel_x, 584)
    return frame


