#!/usr/bin/env python3
"""sim_upload_mission.py — scenario waypoint'lerini SITL'e yükler + GUIDED.

ArduPilot'ta AUTO modu mission gerektirir; mission yoksa "no mission, can't set
auto" hatasıyla mod geçişi reddedilir. Bu script senaryodaki 5 waypoint'i
MISSION_ITEM_INT ile SITL'e yükler, mission'ı alındığını doğrular ve AUTO moduna
geçirir — böylece SET_POSITION_TARGET_LOCAL_NED hız komutu uygulanabilir.

Kullanım: python3 sim_upload_mission.py --scenario-file full_mission.yaml
"""

import argparse
import json
import math
import time
from pathlib import Path

EARTH_RADIUS_M = 6371000.0


def load_waypoints(path: str) -> list[tuple[float, float]]:
    """Tek otorite scenario dosyasından coğrafik waypoint üretir."""
    text = Path(path).read_text(encoding="utf-8")
    try:
        scenario = json.loads(text)
    except json.JSONDecodeError:
        import yaml

        scenario = yaml.safe_load(text)
    origin = scenario["origin"]
    origin_lat = float(origin["lat"])
    origin_lon = float(origin["lon"])
    cos_lat = math.cos(math.radians(origin_lat))
    result = []
    for item in scenario["waypoints"]:
        lat = origin_lat + math.degrees(float(item["x"]) / EARTH_RADIUS_M)
        lon = origin_lon + math.degrees(float(item["y"]) / (EARTH_RADIUS_M * cos_lat))
        result.append((lat, lon))
    if not result:
        raise ValueError("scenario waypoint listesi boş")
    return result


def wait_for_mode(conn, mavutil, expected: str, timeout_s: float = 6.0) -> str:
    """Queued eski heartbeat'leri atlayıp beklenen moda kadar bekle.

    SET_MODE hemen ardından okunan ilk heartbeat, komuttan önce kuyruğa girmiş
    MANUAL olabilir. Tek örnekle karar vermek gerçekte GUIDED'a geçen SITL
    zincirini yanlış başarısız sayıyordu.
    """

    deadline = time.monotonic() + max(0.0, float(timeout_s))
    last_mode = "?"
    while time.monotonic() < deadline:
        heartbeat = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.75)
        if heartbeat is None:
            continue
        last_mode = mavutil.mode_string_v10(heartbeat)
        if last_mode == expected:
            return last_mode
    return last_mode


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser(description="SITL'e mission yükler + AUTO")
    parser.add_argument("--address", default="udp:127.0.0.1:14541")
    parser.add_argument("--scenario-file", required=True)
    args = parser.parse_args(argv)
    waypoints = load_waypoints(args.scenario_file)

    from pymavlink import mavutil

    conn = mavutil.mavlink_connection(args.address)
    conn.wait_heartbeat(timeout=5)
    print(f"Bağlandı: sys={conn.target_system} comp={conn.target_component}")

    # 1) Mission'ı temizle + sayı bildir (MISSION_COUNT)
    n = len(waypoints)
    conn.mav.mission_count_send(conn.target_system, conn.target_component, n)
    time.sleep(0.5)
    print(f"Mission sayısı bildirildi: {n}")

    # 2) Her waypoint'i MISSION_ITEM_INT olarak gönder.
    #    current: ilk item 1 (aktif), diğerleri 0 — ArduPilot ilk item'da
    #    current=1 bekler; 0 olursa MISSION_ACK INVALID (type=14) döner.
    #    mission_type: MAV_MISSION_TYPE_MISSION (0) — pymavlink default.
    for i, (lat, lon) in enumerate(waypoints):
        conn.mav.mission_item_int_send(
            conn.target_system, conn.target_component,
            i,           # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            1 if i == 0 else 0,  # current: ilk item aktif
            1,           # autocontinue: sonraki waypoint'e otomatik geç
            0, 0, 0,     # param1-3 (hold, accept radius, pass radius)
            0,           # param4 (yaw)
            int(lat * 1e7),
            int(lon * 1e7),
            0,           # z (alt)
        )
        time.sleep(0.2)
        print(f"  waypoint {i}: {lat:.7f}, {lon:.7f}")

    # 3) Mission ACK bekle
    ack = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    if ack:
        print(f"MISSION_ACK: type={ack.type}")
        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"UYARI: mission kabul edilmedi (type={ack.type})", file=__import__("sys").stderr)
    else:
        print("MISSION_ACK zaman aşımı", file=__import__("sys").stderr)

    time.sleep(1.0)

    # 4) GUIDED (15) moduna geç — otonomi velocity komutlarını uygular.
    #    (AUTO=10 mission takibi yapar, dış velocity'yi yok sayar; GUIDED=15
    #    SET_POSITION_TARGET_LOCAL_NED komutlarını kabul eder.)
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        15,  # GUIDED
    )
    mode = wait_for_mode(conn, mavutil, "GUIDED", timeout_s=6.0)
    print(f"Mod: {mode}")
    if mode != "GUIDED":
        print(f"UYARI: GUIDED'a geçilemedi (mod={mode})", file=__import__("sys").stderr)
        return 1

    # 5) Mission'ı BAŞLATMA — autonomy kendi DWA'sıyla kontrol eder.
    #    MAV_CMD_MISSION_START göndermek ArduPilot'u AUTO takibine geçirir
    #    (duba algısı yok, "deli dana" davranışı). GUIDED'da kalınır; autonomy
    #    /mission/start (ROS2) ile göreve başlar ve DWA komutları uygulanır.
    print("Mission yüklendi + GUIDED — autonomy /mission/start ile başlatır")
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
