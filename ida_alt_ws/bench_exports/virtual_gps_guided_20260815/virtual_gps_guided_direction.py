#!/usr/bin/env python3
"""Bounded virtual-GPS GUIDED directional bench pulses."""
import argparse,json,time,traceback
from pathlib import Path
from pymavlink import mavutil
LAT=37.8729947; LON=32.4871510; ALT=1015.0; MASK=0x05E7
LOG_DIR=Path("/home/ezelproject/ida_alt_ws/bench_backups")
_orig=mavutil.add_message
def safe(messages,mtype,msg):
    old=messages.get(mtype)
    if old is None or getattr(old,"_instances",{}) is None: messages.pop(mtype,None)
    return _orig(messages,mtype,msg)
mavutil.add_message=safe
ap=argparse.ArgumentParser()
ap.add_argument("--label",required=True,choices=("forward","reverse","right","left"))
ap.add_argument("--vx",type=float,required=True); ap.add_argument("--yaw-rate",type=float,required=True)
ap.add_argument("--cycles",type=int,default=3); ap.add_argument("--pulse-s",type=float,default=2.0)
ap.add_argument("--pause-s",type=float,default=3.0)
a=ap.parse_args()
if isinstance(a.cycles,bool) or not 1<=a.cycles<=3: raise SystemExit("cycles out of range")
if not -0.35<=a.vx<=0.35 or not -0.35<=a.yaw_rate<=0.35: raise SystemExit("command envelope exceeded")
if not 0.1<=a.pulse_s<=2.0 or not 3.0<=a.pause_s<=8.0: raise SystemExit("timing envelope exceeded")
c=None; connected=False
result={"label":a.label,"vx_mps":a.vx,"yaw_rate_rps":a.yaw_rate,"cycles":a.cycles,
 "pulse_s":a.pulse_s,"pause_s":a.pause_s,"mask":hex(MASK),"source_system":255,
 "events":[],"cycle_summaries":[],"success":False}
def event(name,**extra):
    row={"event":name,"monotonic":time.monotonic()}; row.update(extra); result["events"].append(row)
    print(name,extra,flush=True)
def gps():
    c.mav.gps_input_send(int(time.time()*1_000_000),0,0,0,0,3,int(LAT*1e7),int(LON*1e7),ALT,
        0.7,1.0,0,0,0,0.3,0.8,1.0,12,0)
def command(vx,yaw):
    c.mav.set_position_target_local_ned_send(int(time.monotonic()*1000)&0xffffffff,
        c.target_system,c.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,MASK,
        0,0,0,vx,0,0,0,0,0,0,yaw)
def loop(seconds,vx=0.0,yaw=0.0,collect=False):
    end=time.monotonic()+seconds; next_send=0; hb=None; fix=0; gp=None; samples=[]
    while time.monotonic()<end:
        now=time.monotonic()
        if now>=next_send:
            gps(); command(vx,yaw); next_send=now+0.1
        m=c.recv_match(blocking=True,timeout=.03)
        if not m: continue
        t=m.get_type()
        if t=="HEARTBEAT": hb=m
        elif t=="GPS_RAW_INT": fix=max(fix,int(m.fix_type))
        elif t=="GLOBAL_POSITION_INT": gp=m
        elif t=="SERVO_OUTPUT_RAW" and collect:
            samples.append({"t":now,"servo9":int(m.servo9_raw),"servo11":int(m.servo11_raw)})
    return hb,fix,gp,samples
def wait_mode(mode,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        hb,_,_,_=loop(min(.25,end-time.monotonic()))
        if hb and hb.custom_mode==mode: return True
    return False
def wait_arm(want,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        hb,_,_,_=loop(min(.25,end-time.monotonic()))
        if hb and bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)==want: return True
    return False
try:
    c=mavutil.mavlink_connection("/dev/ttyACM0",baud=115200,source_system=255,source_component=190,autoreconnect=False)
    hb=c.recv_match(type="HEARTBEAT",blocking=True,timeout=7)
    if hb is None: raise RuntimeError("no heartbeat")
    connected=True
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED: raise RuntimeError("already armed")
    mapping=c.mode_mapping() or {}
    if "GUIDED" not in mapping or "MANUAL" not in mapping: raise RuntimeError("mode mapping missing")
    result["mode_mapping"]={"GUIDED":mapping["GUIDED"],"MANUAL":mapping["MANUAL"]}
    event("GPS_WARMUP"); _,fix,gp,_=loop(7)
    if fix<3 or gp is None or gp.lat==0 or gp.lon==0: raise RuntimeError(f"GPS not healthy fix={fix}")
    result["gps_fix"]=fix
    event("SET_GUIDED_DISARMED")
    c.mav.set_mode_send(c.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mapping["GUIDED"])
    if not wait_mode(mapping["GUIDED"],5): raise RuntimeError("GUIDED rejected")
    event("ARM_REQUEST")
    c.mav.command_long_send(c.target_system,c.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
    if not wait_arm(True,5): raise RuntimeError("ARM rejected")
    event("ARMED_ZERO_SETTLE"); loop(1.2)
    for n in (3,2,1): event("COUNTDOWN",seconds=n); loop(1)
    for cycle in range(1,a.cycles+1):
        event("PULSE_START",cycle=cycle,vx=a.vx,yaw_rate=a.yaw_rate,duration_s=a.pulse_s)
        _,_,_,samples=loop(a.pulse_s,a.vx,a.yaw_rate,True)
        summary={"cycle":cycle,"sample_count":len(samples)}
        if samples:
            for ch in ("servo9","servo11"):
                summary[f"{ch}_min"]=min(s[ch] for s in samples); summary[f"{ch}_max"]=max(s[ch] for s in samples)
        result["cycle_summaries"].append(summary); event("PULSE_END",**summary)
        if cycle<a.cycles: event("NEUTRAL_PAUSE",seconds=a.pause_s); loop(a.pause_s)
    loop(1.5); result["success"]=True
except Exception as exc:
    result["error"]=str(exc); result["traceback"]=traceback.format_exc()
finally:
    if c is not None and connected:
        try: event("CLEANUP_ZERO"); loop(1.0)
        except Exception as exc: result["cleanup_zero_error"]=str(exc)
        try:
            event("DISARM_REQUEST")
            for _ in range(3):
                c.mav.command_long_send(c.target_system,c.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
                gps(); time.sleep(.15)
            result["disarmed_confirmed"]=wait_arm(False,4)
        except Exception as exc: result["disarm_error"]=str(exc)
        try:
            if "MANUAL" in (c.mode_mapping() or {}):
                c.mav.set_mode_send(c.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,(c.mode_mapping() or {})["MANUAL"])
                result["manual_confirmed"]=wait_mode((c.mode_mapping() or {})["MANUAL"],4)
        except Exception as exc: result["manual_error"]=str(exc)
    if c is not None: c.close()
    LOG_DIR.mkdir(parents=True,exist_ok=True); stamp=time.strftime("%Y%m%dT%H%M%SZ",time.gmtime())
    path=LOG_DIR/f"guided_direction_{a.label}_{stamp}.json"; result["log_path"]=str(path)
    with path.open("x",encoding="utf-8") as f: json.dump(result,f,indent=2,sort_keys=True,default=str)
    print("FINAL_RESULT",json.dumps({k:result.get(k) for k in ("label","success","error","gps_fix","cycle_summaries","disarmed_confirmed","manual_confirmed","log_path")},sort_keys=True),flush=True)
