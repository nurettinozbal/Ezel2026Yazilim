#!/usr/bin/env python3
"""One-time, reversible virtual GPS preparation. No arm/mode/motor commands."""
import json, os, time
from pathlib import Path
from pymavlink import mavutil

DEVICE="/dev/ttyACM0"
BAUD=115200
BACKUP_DIR=Path("/home/ezelproject/ida_alt_ws/bench_backups")
PARAMS=["GPS1_TYPE","GPS2_TYPE","GPS_AUTO_SWITCH","GPS_INJECT_TO",
        "AHRS_EKF_TYPE","EK3_SRC1_POSXY","EK3_SRC1_VELXY","EK3_SRC1_POSZ","EK3_SRC1_YAW",
        "FRAME_CLASS","FRAME_TYPE","SERVO9_FUNCTION","SERVO11_FUNCTION",
        "SERVO9_REVERSED","SERVO11_REVERSED","ARMING_CHECK"]

def get_param(conn, name, timeout=3.0):
    conn.mav.param_request_read_send(conn.target_system, conn.target_component, name.encode("ascii"), -1)
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        m=conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
        if not m: continue
        raw=m.param_id.decode("ascii","ignore") if isinstance(m.param_id,bytes) else str(m.param_id)
        if raw.rstrip("\0")==name:
            return float(m.param_value)
    return None

def main():
    conn=mavutil.mavlink_connection(DEVICE, baud=BAUD, source_system=255, source_component=190, autoreconnect=False)
    hb=conn.recv_match(type="HEARTBEAT", blocking=True, timeout=6)
    if hb is None: raise SystemExit("ABORT: no heartbeat")
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise SystemExit("ABORT: vehicle is armed; no configuration was changed")
    before={name:get_param(conn,name) for name in PARAMS}
    old=before.get("GPS1_TYPE")
    if old is None: raise SystemExit("ABORT: GPS1_TYPE could not be read")
    if int(round(old)) != 9:
        raise SystemExit(f"ABORT: GPS1_TYPE expected 9 (DroneCAN), got {old}; no change")
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    stamp=time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    path=BACKUP_DIR/f"virtual_gps_backup_{stamp}.json"
    backup={"created_utc":stamp,"device":DEVICE,"target_system":conn.target_system,
            "target_component":conn.target_component,"before":before,
            "restore_instruction":"Set GPS1_TYPE back to 9, reboot Pixhawk, then verify GPS fix."}
    with path.open("x", encoding="utf-8") as f: json.dump(backup,f,indent=2,sort_keys=True)
    conn.mav.param_set_send(conn.target_system, conn.target_component, b"GPS1_TYPE", 14.0,
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    end=time.monotonic()+5
    confirmed=None
    while time.monotonic()<end:
        m=conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
        if not m: continue
        raw=m.param_id.decode("ascii","ignore") if isinstance(m.param_id,bytes) else str(m.param_id)
        if raw.rstrip("\0")=="GPS1_TYPE":
            confirmed=float(m.param_value)
            if int(round(confirmed))==14: break
    if confirmed is None or int(round(confirmed))!=14:
        raise SystemExit(f"ABORT: GPS1_TYPE set not confirmed ({confirmed}); backup kept at {path}")
    print(json.dumps({"backup":str(path),"gps1_type_before":old,"gps1_type_after":confirmed,
                      "action":"rebooting Pixhawk to apply GPS1_TYPE=14; no arm/mode/motor command sent"}, sort_keys=True))
    conn.mav.command_long_send(conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1,0,0,0,0,0,0)
    conn.close()

if __name__=="__main__":
    main()
