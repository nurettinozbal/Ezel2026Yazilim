#!/usr/bin/env python3
"""vel_test.py — GUIDED'da sürekli velocity komutunu test eder.

ArduRover GUIDED modunda SET_POSITION_TARGET_LOCAL_NED (LOCAL_NED frame,
speed + yaw_rate) komutunu uygular. 0x05E7 Rover dalında vx bir kuzey hızı
değil, işaretli ileri hızdır. Bu script 5 sn boyunca sürekli vx=0.8 gönderir
ve aracın hareket edip etmediğini doğrular (groundspeed/throttle).

Kullanım: python3 vel_test.py [--address udp:127.0.0.1:14541]
"""

import argparse
import time


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default="udp:127.0.0.1:14541")
    parser.add_argument("--duration_s", type=float, default=5.0)
    parser.add_argument("--vx", type=float, default=0.8)
    args = parser.parse_args(argv)

    from pymavlink import mavutil

    conn = mavutil.mavlink_connection(args.address)
    conn.wait_heartbeat(timeout=5)
    hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    mode = mavutil.mode_string_v10(hb) if hb else "?"
    print(f"mod: {mode}")
    if mode != "GUIDED":
        print(f"GUIDED degil ({mode}) — once GUIDED moduna gec")
        return 1

    n = int(args.duration_s / 0.1)
    print(f"{args.duration_s} sn surekli velocity (vx={args.vx}, LOCAL_NED)...")
    for _ in range(n):
        conn.mav.set_position_target_local_ned_send(
            0,
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # velocity + yaw_rate etkin
            0, 0, 0,  # konum
            args.vx, 0, 0,  # vx=signed forward speed; vy/vz unused by Rover
            0, 0, 0,  # ivme
            0, 0,  # yaw, yaw_rate
        )
        time.sleep(0.1)

    vfr = conn.recv_match(type="VFR_HUD", blocking=True, timeout=2)
    gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if vfr:
        print(f"groundspeed={vfr.groundspeed:.2f} throttle={vfr.throttle}")
    if gps:
        print(f"lat={gps.lat / 1e7:.7f} lon={gps.lon / 1e7:.7f}")
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
