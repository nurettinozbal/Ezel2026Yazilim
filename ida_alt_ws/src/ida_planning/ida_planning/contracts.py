"""JSON topic contracts used by the autonomy and simulation packages.

The first integration layer intentionally uses std_msgs/String with JSON payloads.
That keeps the stack easy to run before custom ROS2 interfaces are frozen.
"""

import json
import time
from typing import Any, Dict

# Şartname renk sabitleri (src/_pdf_ozet.txt:655-674).
# Kenar dubaları turuncu (RAL 2003), engel dubaları sarı (RAL 1026),
# hedef dubaları siyah (RAL 9005) / kırmızı (RAL 3026) / yeşil (RAL 6037).
COLORS = {
    "edge_buoy": "orange",      # RAL 2003 — P1/P2 kenar dubası
    "obstacle_buoy": "yellow",  # RAL 1026 — P2 engel dubası
    "target_black": "black",    # RAL 9005 — P3 hedef dubası
    "target_red": "red",        # RAL 3026 — P3 hedef dubası
    "target_green": "green",    # RAL 6037 — P3 hedef dubası
}
# Saha testinde seçilebilen hedef renkleri. Bilinmeyen/renksiz lidar nesnesi
# hedef sayılmaz; yalnız kameranın doğruladığı bu beş sınıf angajmana girebilir.
VALID_TARGET_COLORS = {"black", "red", "green", "orange", "yellow"}


def now_stamp() -> float:
    return time.time()


def dumps(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=True, separators=(",", ":"))


def loads(data: str, default: Any = None) -> Any:
    try:
        return json.loads(data)
    except Exception:
        return default


def telemetry_state(lat: float, lon: float, heading_deg: float, ground_speed: float, mode: str) -> Dict[str, Any]:
    return {
        "stamp": now_stamp(),
        "lat": lat,
        "lon": lon,
        "heading_deg": heading_deg,
        "ground_speed": ground_speed,
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "mode": mode,
    }


def mission_waypoints(waypoints):
    return {"stamp": now_stamp(), "waypoints": waypoints}
