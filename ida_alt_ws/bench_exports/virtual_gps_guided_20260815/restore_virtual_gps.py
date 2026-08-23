#!/usr/bin/env python3
import json,time
from pathlib import Path
from pymavlink import mavutil
BACKUP=Path("/home/ezelproject/ida_alt_ws/bench_backups/virtual_gps_backup_20260815T092602Z.json")
OUTDIR=BACKUP.parent
_orig=mavutil.add_message
def safe(messages,mtype,msg):
    old=messages.get(mtype)
    if old is None or getattr(old,"_instances",{}) is None: messages.pop(mtype,None)
    return _orig(messages,mtype,msg)
mavutil.add_message=safe
backup=json.loads(BACKUP.read_text(encoding="utf-8"))
wanted=float(backup["before"]["GPS1_TYPE"])
if int(round(wanted))!=9: raise SystemExit("ABORT: backup GPS1_TYPE is not 9")
c=mavutil.mavlink_connection("/dev/ttyACM0",baud=115200,source_system=255,source_component=190,autoreconnect=False)
hb=c.recv_match(type="HEARTBEAT",blocking=True,timeout=7)
if hb is None: raise SystemExit("ABORT: no heartbeat")
if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED: raise SystemExit("ABORT: vehicle armed")
def get(name,seconds=4):
    c.mav.param_request_read_send(c.target_system,c.target_component,name.encode(),-1)
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        m=c.recv_match(type="PARAM_VALUE",blocking=True,timeout=.25)
        if not m: continue
        n=m.param_id.decode("ascii","ignore") if isinstance(m.param_id,bytes) else str(m.param_id)
        if n.rstrip("\0")==name:return float(m.param_value)
    return None
current=get("GPS1_TYPE")
if current is None or int(round(current))!=14: raise SystemExit(f"ABORT: expected temporary GPS1_TYPE=14, got {current}")
c.mav.param_set_send(c.target_system,c.target_component,b"GPS1_TYPE",wanted,mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
end=time.monotonic()+6; confirmed=None
while time.monotonic()<end:
    m=c.recv_match(type="PARAM_VALUE",blocking=True,timeout=.25)
    if not m:continue
    n=m.param_id.decode("ascii","ignore") if isinstance(m.param_id,bytes) else str(m.param_id)
    if n.rstrip("\0")=="GPS1_TYPE":
        confirmed=float(m.param_value)
        if int(round(confirmed))==9:break
if confirmed is None or int(round(confirmed))!=9: raise SystemExit(f"ABORT: restore not confirmed ({confirmed})")
stamp=time.strftime("%Y%m%dT%H%M%SZ",time.gmtime())
record={"created_utc":stamp,"backup":str(BACKUP),"gps1_type_before":current,"gps1_type_restored":confirmed,
        "armed":False,"action":"Pixhawk reboot requested; no arm/mode/motor command sent"}
path=OUTDIR/f"virtual_gps_restore_{stamp}.json"
with path.open("x",encoding="utf-8") as f:json.dump(record,f,indent=2,sort_keys=True)
print(json.dumps({"restore_record":str(path),**record},sort_keys=True),flush=True)
c.mav.command_long_send(c.target_system,c.target_component,mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,0,1,0,0,0,0,0,0)
c.close()
