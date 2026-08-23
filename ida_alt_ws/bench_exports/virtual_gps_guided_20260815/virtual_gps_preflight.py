#!/usr/bin/env python3
"""Static MAVLink GPS_INPUT preflight. No arm/mode/motor/parameter commands."""
import argparse, json, time
from pymavlink import mavutil

_original_add_message=mavutil.add_message
def _safe_add_message(messages, mtype, msg):
    if messages.get(mtype) is None:
        messages.pop(mtype, None)
    return _original_add_message(messages, mtype, msg)
mavutil.add_message=_safe_add_message

ap=argparse.ArgumentParser()
ap.add_argument("--lat",type=float,required=True); ap.add_argument("--lon",type=float,required=True)
ap.add_argument("--alt-m",type=float,default=1015.0); ap.add_argument("--seconds",type=float,default=20.0)
args=ap.parse_args()
if not (-90 <= args.lat <=90 and -180<=args.lon<=180): raise SystemExit("invalid coordinate")
c=mavutil.mavlink_connection("/dev/ttyACM0",baud=115200,source_system=255,source_component=190,autoreconnect=False)
hb=c.recv_match(type="HEARTBEAT",blocking=True,timeout=7)
if hb is None: raise SystemExit("no heartbeat")
if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED: raise SystemExit("ABORT: armed")
start=time.monotonic(); next_send=start; latest={}
while time.monotonic()-start < args.seconds:
    now=time.monotonic()
    if now>=next_send:
        c.mav.gps_input_send(
            int(time.time()*1_000_000), 0, 0, 0, 0, 3,
            int(round(args.lat*1e7)), int(round(args.lon*1e7)), args.alt_m,
            0.7, 1.0, 0.0, 0.0, 0.0, 0.3, 0.8, 1.0, 12, 0)
        next_send=now+0.2
    m=c.recv_match(blocking=True,timeout=.03)
    if m and m.get_type() in ("GPS_RAW_INT","GLOBAL_POSITION_INT","EKF_STATUS_REPORT","HEARTBEAT","STATUSTEXT"):
        latest[m.get_type()]=m.to_dict()
out={"lat":args.lat,"lon":args.lon,"alt_m":args.alt_m,"seconds":args.seconds,
     "armed":bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
     "gps_raw":latest.get("GPS_RAW_INT"),"global_position":latest.get("GLOBAL_POSITION_INT"),
     "ekf":latest.get("EKF_STATUS_REPORT"),"statustext":latest.get("STATUSTEXT")}
print(json.dumps(out,indent=2,sort_keys=True,default=str))
c.close()
