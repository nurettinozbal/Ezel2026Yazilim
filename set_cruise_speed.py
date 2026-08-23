#!/usr/bin/env python3
"""
CRUISE_SPEED (ve istersen WP_SPEED) otopilot parametresini PARAM_SET ile
yazar. Bu SCR_USER1-4 gibi bir yer istasyonu/gorev anahtari DEGIL — WAYPOINT
fazinda ArduPilot'un AUTO navigasyonunun kullandigi sabit "seyir hizi"
(m/s). collision_avoidance/kamikaze'nin RC_CHANNELS_OVERRIDE ile surdugu
DRIVETEST/KAMIKAZE/kacis fazlarini ETKILEMEZ (onlar zaten forward_speed/
approach_speed/ram_speed uzerinden hizlandirildi) -- SADECE WAYPOINT fazinda
otopilotun kendi kendine surdugu duz seyir hizini degistirir.

Genelde YARISMA GUNU DEGIL, tezgahtaki tek seferlik bir ayardir (Mission
Planner/QGC Full Parameter List'ten de ayni islem yapilabilir) -- bu script
sadece Mission Planner/QGC acik degilse hizli bir alternatif.

ONCE 'idaws stop' calistir -- otonomi calisirken Pixhawk seri portunu
tutuyor, bu script ayni porta baglanamaz.

Kullanim:
  python3 set_cruise_speed.py <m/s>                    # /dev/idaws_pixhawk
  python3 set_cruise_speed.py <m/s> /dev/ttyACM0        # port belirt
"""
import sys
from pymavlink import mavutil

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

NEW_SPEED = float(sys.argv[1])
PORT = sys.argv[2] if len(sys.argv) > 2 else '/dev/idaws_pixhawk'
BAUD = 115200

print(f'Baglaniliyor: {PORT}@{BAUD} ...')
conn = mavutil.mavlink_connection(PORT, baud=BAUD)

print('Gercek arac heartbeat bekleniyor...')
hb = conn.wait_heartbeat(timeout=15)
if hb is None:
    print('HATA: 15 sn icinde heartbeat gelmedi.')
    sys.exit(1)

conn.target_system = conn.target_system or hb.get_srcSystem()
conn.target_component = conn.target_component or hb.get_srcComponent()
print(f'Arac bulundu: sysid={conn.target_system} compid={conn.target_component}')


def set_param(name, value):
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        name.encode('utf-8'), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    ack = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5.0)
    if ack and ack.param_id.strip('\x00') == name:
        print(f'  {name} = {ack.param_value}  (dogrulandi)')
        return True
    print(f'  UYARI: {name} icin PARAM_VALUE onayi gelmedi (5 sn), yine de yazilmis olabilir.')
    return False


set_param('CRUISE_SPEED', NEW_SPEED)
# WP_SPEED sifirsa ArduPilot CRUISE_SPEED'i kullanir; sifir DEGILSE WP_SPEED
# CRUISE_SPEED'i EZER. Tutarli olsun diye ayni degeri iki parametreye de yaz.
set_param('WP_SPEED', NEW_SPEED)

print('\nBitti. Dogrulamak icin Mission Planner/QGC Full Parameter List\'ten de kontrol edebilirsin.')
