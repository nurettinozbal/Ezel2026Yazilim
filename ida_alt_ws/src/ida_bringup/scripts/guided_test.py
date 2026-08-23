#!/usr/bin/env python3
"""guided_test.py — SITL'de GUIDED moda geçip velocity komutu test eder.

Hata ayıklama aracı: mavsdk_bridge'in GUIDED'a geçemediği durumlarda (mod
MANUAL'da kalıyor) mod geçişini ve SET_POSITION_TARGET_LOCAL_NED akışını
doğrular. Yalnız sim testi içindir.

Kullanım: python3 guided_test.py [--address udp:127.0.0.1:14541]
"""

import argparse
import time


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default="udp:127.0.0.1:14541")
    parser.add_argument("--iters", type=int, default=5)
    args = parser.parse_args(argv)

    from pymavlink import mavutil

    conn = mavutil.mavlink_connection(args.address)
    conn.wait_heartbeat(timeout=5)

    def get_mode():
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        return mavutil.mode_string_v10(hb) if hb else "?"

    print("mevcut mod:", get_mode())
    # GUIDED'a geç (SET_MODE, custom_mode = GUIDED = 15 — Rover)
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        15,
    )
    time.sleep(1.5)
    mode = get_mode()
    print("mod:", mode)
    if mode != "GUIDED":
        print("GUIDED degil — velocity gonderilmiyor")
        return 1

    print("GUIDED! velocity gonderiliyor...")
    for _ in range(args.iters):
        conn.mav.set_position_target_local_ned_send(
            0,
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # velocity + yaw_rate etkin
            0, 0, 0,  # konum
            0.8, 0, 0,  # vx=+0.8 signed forward speed; vy/vz unused by Rover
            0, 0, 0,  # ivme
            0, 0,  # yaw, yaw_rate
        )
        time.sleep(0.2)
    print(f"{args.iters}x velocity gonderildi (vx=0.8)")
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
