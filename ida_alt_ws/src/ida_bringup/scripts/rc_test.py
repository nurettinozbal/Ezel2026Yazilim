#!/usr/bin/env python3
"""rc_test.py — ArduRover'da RC override ile motor testi.

ArduRover'da throttle/yaw kanal eşlemesi RCMAP_* parametreleriyle belirlenir;
RCMAP_THROTTLE=0 ise ch3 throttle olarak tanınmaz -> RC override motor sürmez.
Bu script RCMAP_* ayarlar, MANUAL modda RC override (ch3 throttle) gönderir ve
aracın hareket edip etmediğini doğrular.

Kullanım: python3 rc_test.py [--address udp:127.0.0.1:14541]
"""

import argparse
import time


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default="udp:127.0.0.1:14541")
    parser.add_argument("--duration_s", type=float, default=5.0)
    parser.add_argument("--throttle", type=int, default=1600)
    args = parser.parse_args(argv)

    from pymavlink import mavutil

    conn = mavutil.mavlink_connection(args.address)
    conn.wait_heartbeat(timeout=5)

    # RCMAP kanal eşlemesi: throttle=ch3, roll=ch1, pitch=ch2, yaw=ch4.
    for name, ch in [("RCMAP_THROTTLE", 3), ("RCMAP_ROLL", 1), ("RCMAP_PITCH", 2), ("RCMAP_YAW", 4)]:
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            name.encode(), float(ch), mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        )
        time.sleep(0.3)
    print("RCMAP ayarlandi: throttle=ch3, yaw=ch4")

    # MANUAL moda geç (RC override her modda çalışır ama MANUAL en net).
    conn.mav.set_mode_send(conn.target_system, 1, 0)
    time.sleep(1.5)
    hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    mode = mavutil.mode_string_v10(hb) if hb else "?"
    print(f"mod: {mode}")

    # ARMED kontrolü
    hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else False
    print(f"ARMED: {armed}")
    if not armed:
        print("UYARI: araç ARMED değil — once sim_arm.py calistir")
        return 1

    n = int(args.duration_s / 0.1)
    print(f"RC override ch3={args.throttle} {args.duration_s} sn...")
    for _ in range(n):
        conn.mav.rc_channels_override_send(
            conn.target_system, conn.target_component,
            0, 0, args.throttle, 0, 0, 0, 0, 0,
        )
        time.sleep(0.1)

    vfr = conn.recv_match(type="VFR_HUD", blocking=True, timeout=2)
    if vfr:
        print(f"groundspeed={vfr.groundspeed:.2f} throttle={vfr.throttle}")
    # Durdur
    conn.mav.rc_channels_override_send(conn.target_system, conn.target_component, 0, 0, 1500, 0, 0, 0, 0, 0)
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
