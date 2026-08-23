#!/usr/bin/env python3
"""
Bench/kapali alan testi icin EKF/GPS moda-gecis engellerini gevsetir.

⚠⚠⚠ SADECE BENCH/IC MEKAN TESTI ICINDIR. ⚠⚠⚠
Bu parametreler EKF/GPS kalite kontrollerini VE EKF failsafe TEPKISINI
devre disi birakir -- gercek suda calisirken EKF gercekten bozulursa
(GPS kaybi, glitch, vs) hicbir koruma kalmaz, arac beklenmedik davranabilir.
YARISMA/SU USTU calismasindan ONCE bu script'in bastigi "ORIJINAL DEGERLER"
listesiyle GERI YAZILMALI.

Sadece BU sorunla ilgili olanlar degistirilir (EK3_GPS_CHECK, FS_EKF_ACTION,
ARMING_CHECK) -- RC/pil failsafe'lerine DOKUNULMAZ, onlar bu sorunla ilgisiz
ve bench testte de faydali kalmaya devam eder.

ONCE 'idaws stop' calistir -- otonomi calisirken Pixhawk seri portunu
tutuyor, bu script ayni porta baglanamaz.

Kullanim:
  python3 disable_bench_failsafes.py            # /dev/idaws_pixhawk
  python3 disable_bench_failsafes.py /dev/ttyACM0
"""
import sys
import time
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/idaws_pixhawk'
BAUD = 115200

# (parametre adi, bench-test degeri, aciklama)
TARGETS = [
    ('EK3_GPS_CHECK', 0, 'EKF3 GPS kalite on-kontrolleri (HDOP/hiz/pozisyon dogrulugu) kapatilir'),
    ('FS_EKF_ACTION', 0, 'EKF failsafe TEPKISI (ucusta HOLD/RTL zorlamasi) kapatilir'),
    ('ARMING_CHECK', 0, 'Tum ARM-oncesi kontroller kapatilir'),
]

print(f'Baglaniliyor: {PORT}@{BAUD} ...')
conn = mavutil.mavlink_connection(PORT, baud=BAUD)
hb = conn.wait_heartbeat(timeout=15)
if hb is None:
    print('HATA: 15 sn icinde heartbeat gelmedi.')
    sys.exit(1)
conn.target_system = conn.target_system or hb.get_srcSystem()
conn.target_component = conn.target_component or hb.get_srcComponent()
print(f'Arac bulundu: sysid={conn.target_system} compid={conn.target_component}\n')


def read_param(name):
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component, name.encode('utf-8'), -1)
    msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5.0)
    return msg.param_value if msg else None


def write_param(name, value):
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        name.encode('utf-8'), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    ack = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5.0)
    return ack.param_value if ack and ack.param_id.strip('\x00') == name else None


print('=== ORIJINAL DEGERLER (yarisma/su ustu oncesi BUNLARA geri don) ===')
originals = {}
for name, _new, _desc in TARGETS:
    val = read_param(name)
    originals[name] = val
    print(f'  {name} = {val}')

print('\n=== BENCH DEGERLERI YAZILIYOR ===')
for name, new_val, desc in TARGETS:
    result = write_param(name, new_val)
    status = 'OK' if result == new_val else f'UYARI: dogrulanamadi (donen: {result})'
    print(f'  {name} = {new_val}  [{desc}]  -- {status}')

print('\n=== OZET: GERI ALMAK ICIN ===')
for name, val in originals.items():
    print(f'  {name} = {val}')
print('\nBu script'"'"'i tekrar calistirip yukaridaki orijinal degerleri elle')
print('yazabilir ya da Mission Planner Full Parameter List'"'"'ten geri alabilirsin.')
print('\n⚠ YARISMA/SU USTU calismasindan ONCE bunlari MUTLAKA geri yaz.')
