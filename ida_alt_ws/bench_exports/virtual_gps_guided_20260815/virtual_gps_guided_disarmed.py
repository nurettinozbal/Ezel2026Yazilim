#!/usr/bin/env python3
"""Verify disarmed GUIDED using temporary GPS_INPUT, then restore MANUAL. No arm/motor command."""
import json,time
from pymavlink import mavutil
LAT=37.8729947; LON=32.4871510; ALT=1015.0
_orig=mavutil.add_message
def safe(messages,mtype,msg):
    if messages.get(mtype) is None: messages.pop(mtype,None)
    return _orig(messages,mtype,msg)
mavutil.add_message=safe
c=mavutil.mavlink_connection("/dev/ttyACM0",baud=115200,source_system=255,source_component=190,autoreconnect=False)
hb=c.recv_match(type="HEARTBEAT",blocking=True,timeout=7)
if hb is None: raise SystemExit("NO_HEARTBEAT")
if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED: raise SystemExit("ABORT: ARMED")
mapping=c.mode_mapping() or {}
if "GUIDED" not in mapping or "MANUAL" not in mapping: raise SystemExit("ABORT: missing Rover GUIDED/MANUAL mapping: "+repr(mapping))
def gps():
    c.mav.gps_input_send(int(time.time()*1_000_000),0,0,0,0,3,int(LAT*1e7),int(LON*1e7),ALT,0.7,1.0,0,0,0,0.3,0.8,1.0,12,0)
def wait_mode(mode,seconds):
    end=time.monotonic()+seconds; last=None
    while time.monotonic()<end:
        gps()
        m=c.recv_match(type="HEARTBEAT",blocking=True,timeout=.15)
        if m:
            last=m
            if m.custom_mode==mode and not (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True,last
    return False,last
warm=time.monotonic()
while time.monotonic()-warm<5:
    gps(); c.recv_match(blocking=True,timeout=.03)
c.mav.set_mode_send(c.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mapping["GUIDED"])
guided_ok,guided_hb=wait_mode(mapping["GUIDED"],5)
c.mav.set_mode_send(c.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mapping["MANUAL"])
manual_ok,manual_hb=wait_mode(mapping["MANUAL"],5)
print(json.dumps({"mode_mapping":{"GUIDED":mapping["GUIDED"],"MANUAL":mapping["MANUAL"]},
 "guided_accepted_disarmed":guided_ok,"manual_restored_disarmed":manual_ok,
 "guided_heartbeat":guided_hb.to_dict() if guided_hb else None,
 "final_heartbeat":manual_hb.to_dict() if manual_hb else None,
 "motor_command_sent":False,"arm_command_sent":False},indent=2,sort_keys=True,default=str))
c.close()
if not (guided_ok and manual_ok): raise SystemExit("MODE_VERIFICATION_FAILED")
