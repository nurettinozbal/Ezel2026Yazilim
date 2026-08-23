"""Safe (pure-python) MAVLink message parsing for the ida_ws_gateway package.

pymavlink is an optional runtime dependency on the vehicle; this module must
stay importable and testable on any developer machine. It parses generic
``dict`` representations of raw MAVLink messages (as produced by
``mavutil.recv_match().to_dict()`` or ``pymavlink.dialects.v20.common`` field
dumps) without importing pymavlink itself.
"""

import math
from typing import Any, Dict, List, Optional, Tuple

from ida_planning.contracts import COLORS, VALID_TARGET_COLORS

# MISSION_ITEM_INT'te koordinatlar int32 olarak 1e7 (1e-7 derece) çarpanıyla saklanır.
# MAVLink arayüzü şartnamesi: x = lat * 1e7, y = lon * 1e7 (WGS84, derece).
MAV_COORD_SCALE = 1e7

# Gerçek YKİ kontratı (ezel-yazilim/arayuz/backend/services/target_manager.py:15-16):
# NAMED_VALUE_INT.name alanı MAVLink char[10]'dur; "TARGET_COLOR" (12 karakter) hatta
# "TARGET_COL" olarak kırpılır. Her iki biçim de kabul edilir (target_manager.py:18-20).
TARGET_COLOR_FIELD_NAMES = {"TARGET_COLOR", "TARGET_COL"}

# Hedef renk kodu eşlemesi. Mevcut P3 kodları korunur; 3=TURUNCU ve 5=SARI
# saha testinde beş sınıfın her birini hedef olarak doğrulamak için ayrılmıştır.
# Renk adları tek kaynaktan (contracts.COLORS) beslenir; autonomy "green" gibi
# küçük harf İngilizce bekler.
DEFAULT_COLOR_INT_MAP = {
    "1": COLORS["target_red"],      # KIRMIZI -> "red"   (RAL 3026)
    "2": COLORS["target_green"],    # YEŞİL  -> "green"  (RAL 6037)
    "3": COLORS["edge_buoy"],       # TURUNCU -> "orange" (RAL 2003)
    "4": COLORS["target_black"],    # SİYAH  -> "black"  (RAL 9005)
    "5": COLORS["obstacle_buoy"],   # SARI -> "yellow"    (RAL 1026)
}


def normalize_named_value_field(raw_name: Any) -> str:
    """NAMED_VALUE_INT.name alanını karşılaştırılabilir hale getirir.

    pymavlink mesajlarında name bazen bytes (b"TARGET_COL\\x00\\x00") olarak gelir;
    MAVLink char[10] sabit uzunluklu olduğu için boş dolguyu temizlemek gerekir.
    (ezel-yazilim target_manager.normalize_named_value_field ile aynı desen.)
    """
    if isinstance(raw_name, (bytes, bytearray)):
        raw_name = raw_name.decode("ascii", errors="replace")
    return str(raw_name or "").replace("\x00", "").strip().upper()


def map_color_int(value: int, color_int_map: Optional[Dict[str, str]] = None) -> Optional[str]:
    """MAVLink NAMED_VALUE_INT.value (int) -> renk adı (küçük harf).

    Eşleme verilmezse gerçek YKİ kontratındaki varsayılan tablo kullanılır
    (DEFAULT_COLOR_INT_MAP). Bilinmeyen kod None döner.
    """
    table = dict(color_int_map) if color_int_map else DEFAULT_COLOR_INT_MAP
    return table.get(str(int(value)))


def is_target_color_field(raw_name: Any) -> bool:
    """Verilen NAMED_VALUE_INT.name alanı hedef rengi mi? (normalize edilerek bakar.)"""
    return normalize_named_value_field(raw_name) in TARGET_COLOR_FIELD_NAMES

# Motor kapalı/hedefsiz geçiş komutu (MISSION_ITEM_COMMAND). Şartnamede parkur
# bilgisi olarak kullanılır; 0 ise ya da tanınmıyorsa parkur=1 varsayılır.
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_DO_JUMP = 177

# ---------------------------------------------------------------------------
# İDA otonomi durumu -> YKİ NAMED_VALUE_INT kontratı.
# Bu tablolar YKİ backend/services/autonomy_contract.py ile BİREBİR aynı olmalı.
# Tüm alan adları <= 10 karakter (MAVLink char[10] asla kırpmaz).
# Kodlama: semantic (state/parkur/action) 1..99; algı/log sayıcıları >= 100.
# ---------------------------------------------------------------------------

# AUTO_ST (state kodu). autonomy_node state string'leri.
# NOT: char[10] sınırı — "AUTONOMY_ST" (11) kırpılır; AUTO_ST (7) güvenli.
STATE_CODE_MAP = {
    "WAIT_MISSION": 1,
    "MISSION_READY": 2,
    "PARKUR_1_NAV": 3,
    "PARKUR_2_AVOIDANCE": 4,
    "PARKUR_3_TARGET_LOCK": 5,
    "ENGAGE": 6,
    "FAILSAFE": 7,
    "COMPLETE": 8,
}

# AUTO_AC (action kodu). autonomy command.action sabit string'leri.
ACTION_CODE_MAP = {
    "idle": 1,
    "hold": 1,
    "waypoint": 2,
    "corridor": 3,
    "avoid": 4,
    "target_hold": 5,
    "search_target_360": 5,
    "target_lock": 5,
    # P3 hedefi doğrulanırken/hizalanırken ve hedef kısa süre kaybolduğunda
    # YKİ'de HEDEF KİLİT gösterilir. ENGAGE önekiyle başlayan iki güvenli
    # duruş da temas değil yeniden-kilitlenme durumudur; uzun önek eşleşmesi
    # bunların genel "engage" koduna düşmesini engeller.
    "target_confirming": 5,
    "target_align": 5,
    "target_loss_grace": 5,
    "engage_target_lost_stop": 5,
    "engage_alignment_lost_stop": 5,
    "engage": 6,
    "failsafe": 7,
    "complete": 8,
}

# PARKUR (parkur kodu): state'ten türetilir (P1=1, P2=2, P3/ENGAGE=3).
PARKUR_FROM_STATE = {
    "PARKUR_1_NAV": 1,
    "PARKUR_2_AVOIDANCE": 2,
    "PARKUR_3_TARGET_LOCK": 3,
    "ENGAGE": 3,
}

# Algı/log sayıcıları: 100 + N (kind alana gömülü). LOG_CNT 100+n sade kodlama.
PERC_DET_OFFSET = 100
PERC_OBS_OFFSET = 100
LOG_CNT_BASE = 100


def encode_perception_count(count: int, offset: int = PERC_DET_OFFSET) -> int:
    """Algı sayısını NAMED_VALUE_INT değerine çevirir (>= 100)."""
    try:
        n = int(count)
    except (TypeError, ValueError):
        return 0
    return offset + max(0, n)


def encode_log_count(active_loggers: int) -> int:
    """Aktif loglayıcı sayısını LOG_CNT değerine çevirir (100 + n)."""
    try:
        n = int(active_loggers)
    except (TypeError, ValueError):
        return 0
    return LOG_CNT_BASE + max(0, n)


def encode_parkur_from_state(state: str) -> int:
    """Otonomi state'inden parkur kodunu türetir (bilinmeyen -> 0)."""
    return PARKUR_FROM_STATE.get(str(state or "").upper(), 0)


def encode_action(action: str) -> int:
    """Autonomy action string'ini sabit koda çevirir (bilinmeyen -> 0).

    Önek eşleşmesi "waypoint distance=4.2 ..." gibi float ekli stringleri,
    sonek eşleşmesi "mission_complete" gibi kod adıyla bitenleri yakalar.
    """
    key = str(action or "").lower().strip()
    for candidate in sorted(ACTION_CODE_MAP, key=len, reverse=True):
        if key.startswith(candidate) or key.endswith(candidate):
            return ACTION_CODE_MAP[candidate]
    return 0


def round_to_deg7(value: float) -> float:
    """7 ondalıklı dereceye yuvarla (dd.ddddddd — ~1 cm çözünürlük)."""
    return round(float(value), 7)


def parse_named_value_int(raw: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """NAMED_VALUE_INT mesajından (param_id, value) çiftini çıkarır.

    ``raw`` bir dict olmalı ve en azından ``msg_type``/``mavpackettype``
    alanında "NAMED_VALUE_INT" bulundurmalıdır. Sağlanan veri uygun değilse
    ``None`` döner (şüpheli veriyi sessizce atla).
    """
    if not isinstance(raw, dict):
        return None
    msg_type = str(raw.get("mavpackettype", raw.get("msg_type", raw.get("type", "")))).upper()
    if msg_type != "NAMED_VALUE_INT":
        return None
    param_id = raw.get("param_id")
    value = raw.get("value")
    if param_id is None or value is None:
        return None
    try:
        int_value = int(value)
    except (TypeError, ValueError):
        return None
    return {"param_id": normalize_named_value_field(param_id), "value": int_value}


def parse_mission_item_int(raw: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """MISSION_ITEM_INT mesajından bir waypoint çıkarır.

    x/y alanları int32 (derece * 1e7) olarak saklanır; float64'e çevrilip
    7 ondalık derece formatına yuvarlanır. Sonuçta parkur bilgisi ``command``
    üzerinden sezgisel olarak belirlenir: MISSION_ITEM_COMMAND türündeki
    (DO_SET_MODE vb.) maddeler parkur geçiş maddesi kabul edilir, NAV_WAYPOINT
    ise seyir noktasıdır.
    """
    if not isinstance(raw, dict):
        return None
    msg_type = str(raw.get("mavpackettype", raw.get("msg_type", raw.get("type", "")))).upper()
    if msg_type != "MISSION_ITEM_INT":
        return None
    try:
        x = float(raw["x"])
        y = float(raw["y"])
        command = int(raw.get("command", 0))
    except (KeyError, TypeError, ValueError):
        return None
    lat = round_to_deg7(x / MAV_COORD_SCALE)
    lon = round_to_deg7(y / MAV_COORD_SCALE)
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return None
    parkur = int(command)
    return {"lat": lat, "lon": lon, "parkur": parkur}


def parse_mission_item_int_list(raw_items: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Bir dizi MISSION_ITEM_INT dict'ini waypoint listesine çevirir."""
    waypoints: List[Dict[str, Any]] = []
    for raw in raw_items:
        wp = parse_mission_item_int(raw)
        if wp is not None:
            waypoints.append(wp)
    return waypoints


def classify_parkur(command: int) -> int:
    """Komut kodundan parkur numarasını sezgisel olarak çıkarır.

    NAV_WAYPOINT ve bilinmeyenler -> parkur 1; DO_SET_MODE -> parkur 2;
    DO_JUMP -> parkur 3. Gerçek misyon sıralaması ilerleyen aşamada kontrat
    gereği waypoint'lerdeki parkur alanıyla netleşir.
    """
    if command == MAV_CMD_NAV_WAYPOINT:
        return 1
    if command == MAV_CMD_DO_SET_MODE:
        return 2
    if command == MAV_CMD_DO_JUMP:
        return 3
    return 1


def make_mission_waypoints(raw_items: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Ham MISSION_ITEM_INT listesini README waypoint kontratına çevirir.

    Çıktı formatı: [{"lat": 40.8630500, "lon": 29.2599500, "parkur": 1}, ...]
    """
    waypoints: List[Dict[str, Any]] = []
    for raw in raw_items:
        wp = parse_mission_item_int(raw)
        if wp is None:
            continue
        parkur = classify_parkur(int(wp.get("parkur", MAV_CMD_NAV_WAYPOINT)))
        waypoints.append({"lat": wp["lat"], "lon": wp["lon"], "parkur": parkur})
    return waypoints


def extract_named_and_mission(raw_messages: List[Dict[str, Any]]) -> Tuple[
    Optional[Dict[str, int]], List[Dict[str, Any]]
]:
    """Bir dizi ham mesajdan hedef rengi ve waypoint listesini çıkarır.

    Dönen tuple: (target_color_dict ya da None, waypoint listesi).
    target_color_dict formatı: {"param_id": str, "value": int}.
    """
    target_color: Optional[Dict[str, int]] = None
    raw_items: List[Dict[str, Any]] = []
    for raw in raw_messages:
        if not isinstance(raw, dict):
            continue
        named = parse_named_value_int(raw)
        if named is not None and is_target_color_field(named["param_id"]):
            target_color = named
            continue
        if parse_mission_item_int(raw) is not None:
            raw_items.append(raw)
    waypoints = make_mission_waypoints(raw_items)
    return target_color, waypoints
