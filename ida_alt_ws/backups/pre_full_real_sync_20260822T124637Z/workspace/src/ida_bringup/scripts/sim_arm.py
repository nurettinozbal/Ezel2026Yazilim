#!/usr/bin/env python3
"""sim_arm.py — SITL'de aracı ARM eder (yalnız simülasyon testi için).

Gerçek araçta ARM YKİ'den (CommandGate -> MAVLink) yapılır; şartname gereği
otonomi/yer istasyonu ARM'i kendisi göndermez. Ancak sim'de YKİ yoktur ve
ArduRover disarmed araçta GUIDED velocity komutunu UYGULAMAZ -> motorlar
dönmez. Bu script sim'e özeldir:

1. SITL heartbeat'ini bekler (launch paralel başlatır — ayağa kalkma 30-60s).
2. ARMING_CHECK=0 ayarlar (SITL'de RC yok) — ancak bu, ArduPilot'un kritik
   pre-arm kontrollerini (IMU/AHRS kalibrasyonu, GPS fix) ATLAMAZ. Bu yüzden
   GPS fix >= 3 (3D) + EKF attitude/velocity/pos_horiz_rel bitleri beklenir.
3. MAV_CMD_COMPONENT_ARM_DISARM gönderir ve HEARTBEAT.armed bayrağından sonucu
   doğrular; reddedilirse STATUSTEXT'ten pre-arm nedenini yazdırır.

Kullanım: python3 sim_arm.py [--address udp:127.0.0.1:14541] [--timeout 10]
"""

import argparse
import sys
import time


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser(description="SITL aracını ARM eder (sim testi)")
    parser.add_argument("--address", default="udp:127.0.0.1:14541",
                        help="pymavlink bağlantı adresi (mavproxy köprüsü)")
    parser.add_argument("--timeout", type=float, default=10.0, help="heartbeat bekleme süresi")
    parser.add_argument("--arming-check", type=int, default=0,
                        help="ARMING_CHECK değeri (0 = pre-arm kapalı; sim'de RC yok)")
    args = parser.parse_args(argv)

    try:
        from pymavlink import mavutil
    except ImportError as exc:
        print(f"pymavlink yok: {exc}", file=sys.stderr)
        return 1

    print(f"Bağlanıyor: {args.address}")
    conn = mavutil.mavlink_connection(args.address)
    # SITL ayağa kalkması 30-60s sürebilir (launch paralel başlatır) — heartbeat
    # gelene kadar döngüyle bekle (max 120s). Aksi halde script SITL'den önce
    # koşar ve ARM başarısız olur (yarınki sim kritiği).
    hb = None
    deadline = time.time() + 120.0
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=5.0)
        if hb:
            break
        print("Heartbeat bekleniyor (SITL hazır değil)...", file=sys.stderr)
    if not hb:
        print("Heartbeat alınamadı — SITL bağlantısı yok (120s aşıldı)", file=sys.stderr)
        return 1
    print(f"Bağlandı: sys={conn.target_system} comp={conn.target_component}")

    # Pre-arm kontrollerini kapat (SITL'de RC yok; gerçek araçta YKİ ARM yapar).
    # NOT: ARMING_CHECK=0 kritik kalibrasyonları (IMU/AHRS, GPS fix) ATLAMAZ —
    # ArduPilot yine de "AHRS not healthy" / "GPS fix" bekler. Aşağıda bunların
    # hazır olması beklenir.
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        b"ARMING_CHECK", float(args.arming_check),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    time.sleep(0.5)
    print(f"ARMING_CHECK={args.arming_check} ayarlandı")

    # IMU/GPS kalibrasyonu + AHRS/EKF healthy bekle (SITL'de genelde 10-20s).
    # EKF_STATUS_REPORT flags bit0=attitude_ok, bit3=velocity_ok, bit5=pos_horiz_rel_ok.
    # GPS_RAW_INT fix_type >= 3 (3D fix). İkisi de olmadan ARM reddedilir.
    deadline = time.time() + 120.0
    gps_fix = 0
    ekf_flags = 0
    print("GPS fix + EKF healthy bekleniyor (IMU kalibrasyonu)...", file=sys.stderr)
    while time.time() < deadline:
        msg = conn.recv_match(
            type=["GPS_RAW_INT", "EKF_STATUS_REPORT", "STATUSTEXT"],
            blocking=True, timeout=2.0,
        )
        if msg is None:
            continue
        if msg.get_type() == "GPS_RAW_INT":
            gps_fix = msg.fix_type
        elif msg.get_type() == "EKF_STATUS_REPORT":
            ekf_flags = msg.flags
        elif msg.get_type() == "STATUSTEXT" and "ARM" in (msg.text or "").upper():
            print(f"SITL: {msg.text}", file=sys.stderr)
        # EKF: attitude(0) + velocity(3) + pos_horiz_rel(5) bitleri açık.
        required = (1 << 0) | (1 << 3) | (1 << 5)
        if gps_fix >= 3 and (ekf_flags & required) == required:
            print(f"GPS fix={gps_fix}, EKF flags=0x{ekf_flags:04x} — hazır")
            break
    else:
        print(
            f"UYARI: GPS fix={gps_fix}, EKF flags=0x{ekf_flags:04x} — 120s'de hazır "
            f"olmadı; ARM denenecek (başarısız olabilir)",
            file=sys.stderr,
        )

    # ARM: MAV_CMD_COMPONENT_ARM_DISARM, param1=1.
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0,
    )
    print("MAV_CMD_COMPONENT_ARM_DISARM (ARM) gönderildi")

    # ARM sonucunu HEARTBEAT.armed bayrağından doğrula (ACK her zaman dönmez).
    # ArduPilot ARM reddederse STATUSTEXT "Arm: <neden>" ile pre-arm nedenini söyler.
    deadline = time.time() + 10.0
    armed = False
    while time.time() < deadline:
        msg = conn.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=2.0)
        if msg is None:
            continue
        if msg.get_type() == "HEARTBEAT" and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            armed = True
            break
        if msg.get_type() == "STATUSTEXT":
            text = (msg.text or "").upper()
            if "ARM" in text and ("FAIL" in text or "DENIED" in text or "REFUSE" in text):
                print(f"ARM REDDEDİLDİ (SITL): {msg.text}", file=sys.stderr)
    if armed:
        print("ARM başarılı (HEARTBEAT SAFETY_ARMED)")
        return 0
    print("ARM sonucu alınamadı / reddedildi", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
