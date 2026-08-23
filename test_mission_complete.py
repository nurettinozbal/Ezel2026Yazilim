#!/usr/bin/env python3
"""
WAYPOINT_KAMIKAZE (SCR_USER1=20) mantigini dogrulamak icin tanilama araci.
Gercek Pixhawk'tan MISSION_COUNT / MISSION_CURRENT / MISSION_ITEM_REACHED
mesajlarini dinleyip ekrana basar -- pymavlink_controller_node'un kullandigi
mantigin AYNISI, ama sadece OKUR, SCR_USER1'e HICBIR SEY YAZMAZ.

ONCE 'idaws stop' calistir -- otonomi calisirken Pixhawk seri portunu tutuyor,
bu script ayni porta baglanamaz.

Kullanim:
  python3 test_mission_complete.py                # /dev/idaws_pixhawk, sonsuz dinle
  python3 test_mission_complete.py /dev/ttyACM0 30 # port + saniye siniri
"""
import sys
import time
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/idaws_pixhawk'
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else None
BAUD = 115200

print(f'Baglaniliyor: {PORT}@{BAUD} ...')
conn = mavutil.mavlink_connection(PORT, baud=BAUD)

print('Gercek arac heartbeat bekleniyor...')
deadline = time.monotonic() + 15
real_hb = None
while time.monotonic() < deadline:
    hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
    if hb and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        real_hb = hb
        break
if real_hb is None:
    print('HATA: 15 sn icinde gercek arac heartbeat i gelmedi.')
    sys.exit(1)

conn.target_system = real_hb.get_srcSystem()
conn.target_component = real_hb.get_srcComponent()
print(f'Arac bulundu: sysid={conn.target_system} compid={conn.target_component}')

print('MISSION_COUNT isteniyor (mission_request_list)...')
conn.mav.mission_request_list_send(conn.target_system, conn.target_component)

print('Dinleniyor -- gercek/simule bir gorevde ilerle, Ctrl-C ile durdur.\n')
start = time.monotonic()
mission_total = 0

try:
    while DURATION is None or (time.monotonic() - start) < DURATION:
        msg = conn.recv_match(
            type=['MISSION_COUNT', 'MISSION_CURRENT', 'MISSION_ITEM_REACHED'],
            blocking=True, timeout=1.0)
        if msg is None:
            continue
        t = msg.get_type()

        if t == 'MISSION_COUNT':
            mission_total = int(msg.count)
            print(f'[MISSION_COUNT] count={mission_total}')

        elif t == 'MISSION_CURRENT':
            state = getattr(msg, 'mission_state', None)
            total = getattr(msg, 'total', None)
            note = ''
            if state == 5:
                note = '  <-- MISSION_STATE_COMPLETE (otomatik gecis burada tetiklenirdi)'
            print(f'[MISSION_CURRENT] seq={msg.seq} mission_state={state} '
                  f'total={total}{note}')

        elif t == 'MISSION_ITEM_REACHED':
            note = ''
            if mission_total > 0 and msg.seq >= mission_total - 1:
                note = f'  <-- SON item (mission_total={mission_total}) - otomatik gecis burada tetiklenirdi'
            print(f'[MISSION_ITEM_REACHED] seq={msg.seq}{note}')

except KeyboardInterrupt:
    pass

print('\nBitti.')
