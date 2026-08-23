#!/usr/bin/env python3
"""
SCR_USER1'i dogrudan Pixhawk'a yazar (web UI'nin '10 * Parkur 1' butonuyla
AYNI islemi yapar). NOT: idaws_ws ROS2 stack'i (ros2 launch idaws_bringup
idaws_launch.py ...) ayni anda calismiyorsa bu tek basina hicbir sey
tetiklemez -- sadece parametreyi yazar, kimse okumaz.

Kullanim:
  python3 set_scr_user1.py            # SCR_USER1 = 10 (Parkur 1)
  python3 set_scr_user1.py 15         # SCR_USER1 = 15 (Parkur 2)
  python3 set_scr_user1.py 10 /dev/ttyACM0
"""
import sys
import time
from pymavlink import mavutil

VALUE = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
PORT = sys.argv[2] if len(sys.argv) > 2 else '/dev/idaws_pixhawk'
BAUD = 115200
PARAM_NAME = 'SCR_USER1'

print(f'Baglaniliyor: {PORT}@{BAUD} ...')
conn = mavutil.mavlink_connection(PORT, baud=BAUD)

# pymavlink_controller_node.py'daki not: bu araclarda USB hattinda ikinci
# (ADS-B pass-through, autopilot=INVALID) bir heartbeat daha akiyor.
# Gercek arac heartbeat'ini (autopilot gecerli) bulup system/component'i
# elle sabitliyoruz -- aksi halde komutlar sessizce yok sayilabiliyor.
print('Gercek arac heartbeat bekleniyor...')
deadline = time.monotonic() + 15
real_hb = None
while time.monotonic() < deadline:
    hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
    if hb is None:
        continue
    if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        real_hb = hb
        break

if real_hb is None:
    print('HATA: 15 sn icinde gercek arac heartbeat i gelmedi. Port/kablo/guc kontrol et.')
    sys.exit(1)

conn.target_system = conn.source_system if False else real_hb.get_srcSystem()
conn.target_component = real_hb.get_srcComponent()
print(f'Arac bulundu: sysid={conn.target_system} compid={conn.target_component}')

print(f'{PARAM_NAME} = {VALUE} yaziliyor...')
conn.mav.param_set_send(
    conn.target_system,
    conn.target_component,
    PARAM_NAME.encode('utf-8'),
    VALUE,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
)

ack = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5.0)
if ack is not None and ack.param_id.strip('\x00') == PARAM_NAME:
    print(f'DOGRULANDI: {PARAM_NAME} = {ack.param_value}')
else:
    print('UYARI: PARAM_VALUE onayi 5 sn icinde gelmedi (yine de yazilmis olabilir).')
