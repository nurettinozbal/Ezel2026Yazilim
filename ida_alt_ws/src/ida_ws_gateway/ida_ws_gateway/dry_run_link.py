"""Simüle MAVLink bağlantısı (dry_run modu).

pymavlink olmadan çalışan, gerçek bir seri port yerine örnek NAMED_VALUE_INT ve
MISSION_ITEM_INT mesajları üreten bir sahte "link" katmanıdır. Amaç: gateway'in
ROS2 tarafındaki davranışını (hedef rengi + waypoint yayını, transient_local
QoS) donanım olmadan geliştirilebilir kılmak.
"""

import json
from typing import Any, Dict, List, Optional

from ida_ws_gateway.mavlink_parser import MAV_COORD_SCALE

# Demo waypoint'ler: Tuzla/TEKNOFEST parkur alanı civarı, 7 ondalık derece.
DEFAULT_DEMO_WAYPOINTS = [
    {"lat": 40.8630500, "lon": 29.2599500, "parkur": 1},
    {"lat": 40.8631500, "lon": 29.2600500, "parkur": 1},
    {"lat": 40.8632500, "lon": 29.2601500, "parkur": 2},
]


class DryRunLink:
    """pymavlink arayüzünü taklit eden statik bir sahte bağlantı.

    ``recv_batch()`` çağrısı her seferinde ya hedef rengi mesajını ya da
    waypoint mesajlarını döndürür; tüketilmiş mesajlar sonraki çağrılarda
    tekrarlanmaz (küçük bir FIFO kuyruğu). Böylece gerçek MAVLink'te bir kez
    gelen TARGET_COLOR'un kaybolmaması davranışı birebir test edilir.
    """

    def __init__(
        self,
        target_color_param_id: str = "TARGET_COLOR",
        demo_target_color: str = "green",
        demo_waypoints: Optional[List[Dict[str, Any]]] = None,
        color_int_map: Optional[Dict[str, str]] = None,
    ) -> None:
        self.target_color_param_id = target_color_param_id
        self.demo_target_color = demo_target_color
        self.demo_waypoints = demo_waypoints if demo_waypoints is not None else DEFAULT_DEMO_WAYPOINTS
        # "green" -> 2 gibi ters eşleme: NAMED_VALUE_INT değerini (int) seç.
        self.color_int_map = color_int_map or {}
        self.color_to_int = self._build_color_to_int()
        self._queue: List[Dict[str, Any]] = []
        self._target_sent = False

    def _build_color_to_int(self) -> Dict[str, int]:
        mapping: Dict[str, int] = {}
        for int_str, color in self.color_int_map.items():
            try:
                mapping[color.lower()] = int(int_str)
            except (TypeError, ValueError):
                continue
        return mapping

    def connect(self) -> None:
        # Sahte bağlantı: gerçek MAVLink serial açılışı burada yoktur.
        self._seed_messages()

    def _seed_messages(self) -> None:
        # Önce hedef rengi (bir kez), ardından waypoint'ler. FIFO sırası korunur.
        color_int = self.color_to_int.get(self.demo_target_color.lower())
        if color_int is not None:
            self._queue.append(
                {
                    "mavpackettype": "NAMED_VALUE_INT",
                    "param_id": self.target_color_param_id,
                    "value": color_int,
                }
            )
            self._target_sent = True
        for wp in self.demo_waypoints:
            lat = float(wp["lat"]) * MAV_COORD_SCALE
            lon = float(wp["lon"]) * MAV_COORD_SCALE
            self._queue.append(
                {
                    "mavpackettype": "MISSION_ITEM_INT",
                    "x": int(round(lat)),
                    "y": int(round(lon)),
                    "z": 0.0,
                    "command": int(wp.get("parkur", 16)),
                }
            )

    def recv_batch(self) -> List[Dict[str, Any]]:
        """Bekleyen mesajların tamamını döndürür ve kuyruğu boşaltır."""
        batch = list(self._queue)
        self._queue.clear()
        return batch

    def put_named_value(self, name: str, value: int) -> None:
        """Ters yön: İDA otonomi durumunu sahte hatta yazar.

        Gateway `_flush_ida_autonomy_status` dry-run'da buraya yazar;
        bir sonraki `recv_batch` (tick) bu mesajı okur ve hedef rengi
        akışıyla aynı yoldan işler.
        """
        self._queue.append(
            {"mavpackettype": "NAMED_VALUE_INT", "param_id": name, "value": int(value)}
        )

    def close(self) -> None:
        self._queue.clear()


def parse_demo_waypoints(raw: str) -> List[Dict[str, Any]]:
    """demo_waypoints parametresindeki JSON dizeyi waypoint listesine çevirir.

    Boş dize veya geçersiz JSON -> built-in demo waypoint'ler kullanılır.
    Açık JSON ``[]`` ise mission yayını bilinçli olarak kapatılır;
    bu, simdeki authoritative mission publisher ile yarışı önler.
    """
    if not raw or not raw.strip():
        return list(DEFAULT_DEMO_WAYPOINTS)
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        return list(DEFAULT_DEMO_WAYPOINTS)
    if not isinstance(parsed, list):
        return list(DEFAULT_DEMO_WAYPOINTS)
    if not parsed:
        return []
    waypoints = []
    for item in parsed:
        if not isinstance(item, dict) or "lat" not in item or "lon" not in item:
            continue
        waypoints.append(
            {
                "lat": float(item["lat"]),
                "lon": float(item["lon"]),
                "parkur": int(item.get("parkur", 1)),
            }
        )
    return waypoints if waypoints else list(DEFAULT_DEMO_WAYPOINTS)
