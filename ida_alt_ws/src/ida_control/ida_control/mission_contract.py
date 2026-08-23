"""Safe (pure-python) mission/target contract helpers for ida_control.

rclpy/MAVSDK yokken import edilip test edilebilir (diğer paketlerdeki saf
modüllerle aynı desen; eski seri gateway'e çalışma zamanı bağımlılığı yoktur).

Hedef rengi akışı: YKİ PARAM_SET ile takım stack'inden ayrı SCR_USER4'e renk kodunu
yazar (1=KIRMIZI, 2=YEŞİL, 3=TURUNCU, 4=SİYAH, 5=SARI); Jetson MAVSDK ``param.get_param_float`` ile
okur. Görev noktaları Pixhawk'tan ``mission_raw.download_mission()`` ile indirilir
(MissionItem.x = lat * 1e7, .y = lon * 1e7 — MISSION_ITEM_INT ile aynı kodlama).

Komut yolu: ArduRover'da MAVSDK ``offboard`` desteklenmediği için gövde hızı
komutları pymavlink ``SET_POSITION_TARGET_LOCAL_NED`` ile GUIDED modda
gönderilir. Üretimde kullanılan 0x05E7 speed + turn-rate dalı vx'i signed
forward speed olarak yorumlar; ``body_to_ned`` bu dala uygulanmaz. Yardımcı
yalnız gerçek NED-vektör komutları ve bunların saf testleri içindir.
"""

from __future__ import annotations

import math

# MISSION_ITEM_INT / MAVSDK MissionItem koordinat çarpanı (derece * 1e7, int32).
MAV_COORD_SCALE = 1e7

# Hedef renk kodu eşlemesi — YKİ kontratıyla aynı
# Mevcut 1/2/4 kodları geriye uyumludur; 3 ve 5 saha test renkleridir.
COLOR_INT_MAP = {1: "red", 2: "green", 3: "orange", 4: "black", 5: "yellow"}

# MAV_CMD komutlarından parkur sezgisel sınıflaması (mavlink_parser.classify_parkur
# ile aynı): NAV_WAYPOINT=16 -> 1, DO_SET_MODE=176 -> 2, DO_JUMP=177 -> 3.
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_DO_JUMP = 177

# ArduRover SET_POSITION_TARGET_LOCAL_NED kontratı. Bunlar MAVLink'in tekil
# x/y/z bitlerinden oluşan 12-bit maskeleridir; grup başına tek bit değildir.
# 0x05E7: signed forward speed (vx) + turn rate; 0x0FC7: NED velocity only.
ROVER_SPEED_TURN_RATE_MASK = 0x05E7
ROVER_VELOCITY_ONLY_MASK = 0x0FC7

# ArduPilot yalnız SCR_USER1..6 sağlar. Takım 1/2/3'ü kullandığı için canonical
# stack 4=renk, 5=paketlenmiş P1/P2 sayaçları, 6=mission mailbox kullanır.
MISSION_COUNT_RADIX = 1001
MISSION_MAILBOX_BASE = 8_000_000
MISSION_CONTROL_MAX_SEQUENCE = 1_999_999


def exact_mission_control_token(value) -> int | None:
    """Return an exact nonnegative float32-safe mailbox value, else ``None``."""
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(number):
        return None
    token = int(round(number))
    max_value = MISSION_MAILBOX_BASE + 4 * MISSION_CONTROL_MAX_SEQUENCE
    if abs(number - token) > 1e-6 or not MISSION_MAILBOX_BASE <= token <= max_value:
        return None
    return token


def decode_mission_mailbox(value) -> tuple[str, int, bool] | None:
    """Decode SCR_USER6 as ``(kind, sequence, start)``.

    Relative codes per sequence are START_CMD=4n-3, STOP_CMD=4n-2,
    START_ACK=4n-1 and STOP_ACK=4n. BASE+0 is the initial STOP_ACK(seq=0).
    """
    token = exact_mission_control_token(value)
    if token is None:
        return None
    code = token - MISSION_MAILBOX_BASE
    if code == 0:
        return "ack", 0, False
    residue = code % 4
    if residue == 1:
        return "command", (code + 3) // 4, True
    if residue == 2:
        return "command", (code + 2) // 4, False
    if residue == 3:
        return "ack", (code + 1) // 4, True
    return "ack", code // 4, False


def next_mission_control(mailbox_value, *, start: bool) -> tuple[int, int] | None:
    """Return ``(command, expected_ack)`` only from an idle ACK mailbox."""
    state = decode_mission_mailbox(mailbox_value)
    if state is None or state[0] != "ack":
        return None
    sequence = state[1] + 1
    if sequence > MISSION_CONTROL_MAX_SEQUENCE:
        sequence = 1
    relative_command = 4 * sequence - (3 if start else 2)
    relative_ack = 4 * sequence - (1 if start else 0)
    return (
        MISSION_MAILBOX_BASE + relative_command,
        MISSION_MAILBOX_BASE + relative_ack,
    )


def pending_mission_control(mailbox_value) -> tuple[int, bool, int] | None:
    """Return ``(command, start, expected_ack)`` for a pending mailbox."""
    state = decode_mission_mailbox(mailbox_value)
    if state is None or state[0] != "command":
        return None
    _kind, sequence, start = state
    ack = MISSION_MAILBOX_BASE + 4 * sequence - (1 if start else 0)
    return exact_mission_control_token(mailbox_value), start, ack


def mission_command_is_next(mailbox_value, last_sequence: int | None) -> bool:
    """Reject live reordered/gapped commands once an ACK sequence was observed."""
    state = decode_mission_mailbox(mailbox_value)
    if state is None or state[0] != "command":
        return False
    if last_sequence is None:
        return True  # pending command may legitimately survive a Jetson reboot
    expected = last_sequence + 1
    if expected > MISSION_CONTROL_MAX_SEQUENCE:
        expected = 1
    return state[1] == expected


def pack_mission_counts(p1_count: int, p2_count: int) -> int | None:
    """Pack two 0..1000 counts into SCR_USER5 (P1 must be non-empty)."""
    if (
        isinstance(p1_count, bool)
        or isinstance(p2_count, bool)
        or not isinstance(p1_count, int)
        or not isinstance(p2_count, int)
        or not 1 <= p1_count <= 1000
        or not 0 <= p2_count <= 1000
    ):
        return None
    return p1_count * MISSION_COUNT_RADIX + p2_count


def unpack_mission_counts(value) -> tuple[int, int] | None:
    """Decode exact SCR_USER5 metadata into ``(p1_count, p2_count)``."""
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(number):
        return None
    packed = int(round(number))
    if abs(number - packed) > 1e-6:
        return None
    p1_count, p2_count = divmod(packed, MISSION_COUNT_RADIX)
    return (p1_count, p2_count) if pack_mission_counts(p1_count, p2_count) == packed else None


def map_target_color(value) -> str | None:
    """Param/MAVLink renk kodunu (int) kontrat rengine (küçük harf) çevirir.

    Sayısal olmayan/geçersiz giriş None döner (şüpheli değeri sessizce atla).
    """
    try:
        code = int(value)
    except (TypeError, ValueError):
        return None
    return COLOR_INT_MAP.get(code)


def mission_items_to_waypoints(
    items, p1_count: int | None = None, p2_count: int | None = None
) -> list[dict]:
    """MAVSDK MissionItem listesini README waypoint kontratına çevirir.

    Girdi: ``mission_raw.download_mission()`` dönüşü (iterable; her öğede ``seq``,
    ``x``, ``y``, ``command`` alanları). Çıktı formatı (README birebir):
    [{"lat": dd.ddddddd, "lon": dd.ddddddd, "parkur": 1}, ...]

    Canonical gerçek akışta YKİ'nin doğrulanmış ``SCR_USER5``/``SCR_USER6``
    adetleri verilir; bütün Pixhawk item'ları güvenli NAV_WAYPOINT kalır ve parkur
    sınırı sıra üzerinden atanır. Metadata yoksa yalnız legacy/sim komut kodu
    sınıflaması kullanılır.
    """
    metadata_mode = p1_count is not None or p2_count is not None
    if metadata_mode:
        if (
            isinstance(p1_count, bool)
            or isinstance(p2_count, bool)
            or not isinstance(p1_count, int)
            or not isinstance(p2_count, int)
            or p1_count < 1
            or p2_count < 0
        ):
            return []
    waypoints = []
    invalid_item = False
    for item in items:
        seq = int(getattr(item, "seq", 0))
        lat = float(getattr(item, "x", 0)) / MAV_COORD_SCALE
        lon = float(getattr(item, "y", 0)) / MAV_COORD_SCALE
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            invalid_item = True
            continue
        command = int(getattr(item, "command", MAV_CMD_NAV_WAYPOINT))
        parkur = _classify_parkur(command)
        waypoints.append(
            {"seq": seq, "lat": round(lat, 7), "lon": round(lon, 7), "parkur": parkur}
        )
    waypoints.sort(key=lambda wp: wp["seq"])
    if metadata_mode:
        mission_count = p1_count + p2_count
        # ArduPilot download listesinde seq=0 HOME kaydı bulunur. YKİ
        # uploader gerçek N waypoint'i seq=1..N'e koyduğunda liste N+1 olur;
        # HOME otonomi rotası değildir ve parkur sayımına girmemelidir.
        if waypoints and waypoints[0]["seq"] == 0:
            waypoints = waypoints[1:]
        if invalid_item or mission_count > len(waypoints):
            return []
        for index, waypoint in enumerate(waypoints):
            if index < p1_count:
                waypoint["parkur"] = 1
            elif index < p1_count + p2_count:
                waypoint["parkur"] = 2
            else:
                waypoint["parkur"] = 3
    return [
        {"lat": wp["lat"], "lon": wp["lon"], "parkur": wp["parkur"]}
        for wp in waypoints
    ]


def _classify_parkur(command: int) -> int:
    """Komut kodundan parkur numarasını çıkarır (NAV_WAYPOINT -> 1)."""
    if command == MAV_CMD_NAV_WAYPOINT:
        return 1
    if command == MAV_CMD_DO_SET_MODE:
        return 2
    if command == MAV_CMD_DO_JUMP:
        return 3
    return 1


# --- pymavlink GUIDED + SET_POSITION_TARGET_LOCAL_NED yardımcıları --------------

def body_to_ned(forward: float, right: float, heading_deg: float) -> tuple[float, float]:
    """Gövde hızını (forward/right) dünya NED (north/east) bileşenlerine çevirir.

    `ida_planning.geo.body_to_world` ile birebir aynı formül (sim ile tutarlılık:
    telemetry_sim_node da aynı dönüşümü kullanır). heading 0 = kuzey (north).
    ArduRover 0x05E7 speed + turn-rate komut yolunda kullanılmaz; o yol vx'i
    signed forward speed olarak yorumlar.
    """
    h = math.radians(heading_deg)
    north = math.cos(h) * forward - math.sin(h) * right
    east = math.sin(h) * forward + math.cos(h) * right
    return north, east


def set_position_target_local_ned_mask(enable_yaw_rate: bool = True) -> int:
    """SET_POSITION_TARGET_LOCAL_NED type_mask'i üretir.

    ArduRover için yaw-rate etkinse speed + turn-rate maskesi (0x05E7),
    değilse standart NED velocity-only maskesi (0x0FC7) döner.
    """
    if enable_yaw_rate:
        return ROVER_SPEED_TURN_RATE_MASK
    return ROVER_VELOCITY_ONLY_MASK


def ned_command_to_dict(vx: float, vy: float, yaw_rate_rad_s: float, heading_deg: float) -> dict:
    """Gövde komutunu NED komut dict'ine çevirir (mesaj kurulumunda kullanılır).

    `body_to_ned` dönüşümünü uygular; çıktı alanları:
    ``{"vx_north", "vy_east", "yaw_rate"}`` (rad/s).
    """
    north, east = body_to_ned(vx, vy, heading_deg)
    return {"vx_north": north, "vy_east": east, "yaw_rate": yaw_rate_rad_s}
