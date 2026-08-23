import asyncio
import websockets
import json
import time
from pymavlink import mavutil

master = None

# Arayüzden gelen görev noktalarını tutacağımız liste
mission_waypoints = []

telemetry_data = {
    "ida": {
        "sys_id": 1, "speed": 0, "battery_percent": 0, "voltage": 0.0, "current": 0.0,
        "mode": "DISARMED", "lat": 41.025, "lon": 28.974, "heading": 0, "current_wp": 0,
        "dist_to_wp": 0, "armed": False, "connected": False, "last_heartbeat": 0,
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        
        # --- MÜHENDİSLİK GRAFİKLERİ VE GÖSTERGELER İÇİN EKLENEN VERİLER ---
        "target_heading": 0, 
        "target_speed": 0, 
        "motor_left_pwm": 1500, 
        "motor_right_pwm": 1500,
        "motor_left_pct": 0, 
        "motor_right_pct": 0,
        "rpm_left": 0, 
        "rpm_right": 0
    },
    "iha": {
        "sys_id": 2, "battery": 0, "voltage": 0.0, "alt": 0, "detected_color": 'BEKLENİYOR',
        "lat": 41.026, "lon": 28.975, "armed": False, "connected": False, "last_update": 0,
    },
    "system": {
        "rssi": -99, "gps_sats": 0, "hdop": 9.9, "failsafe_status": 'UNKNOWN',
        "ida_link": 'DISCONNECTED', "iha_link": 'DISCONNECTED', "emergency_active": False,
        "target_delivery_status": 'not_sent', "target_delivery_message": '',
        "mission_uploaded": False, "mission_waypoint_count": 0,
        "command_auth_required": True, "command_auth_configured": False,
        "logs": ["[SİSTEM] GCS Başlatıldı, Pixhawk bekleniyor..."] 
    }
}

# --- ARAYÜZ (REACT) TERMİNALİNE LOG GÖNDERME FONKSİYONU ---
def add_sys_log(msg_text):
    time_str = time.strftime("%H:%M:%S")
    formatted_msg = f"[{time_str}] {msg_text}"
    print(formatted_msg) 
    telemetry_data["system"]["logs"].append(formatted_msg) 
    
    if len(telemetry_data["system"]["logs"]) > 20:
        telemetry_data["system"]["logs"].pop(0)

async def gcs_heartbeat():
    while True:
        if master:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
        await asyncio.sleep(1)

async def read_mavlink():
    global master, telemetry_data, mission_waypoints
    print("Telemetri modülüne bağlanılıyor... (COM21)")
    
    try:
        master = mavutil.mavlink_connection('COM21', baud=57600)
        master.wait_heartbeat()
        add_sys_log("MAVLink Bağlantısı Başarılı! Veriler akıyor...")
        
        telemetry_data["system"]["ida_link"] = "CONNECTED"
        telemetry_data["ida"]["connected"] = True

        while True:
            msg = master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == 'ATTITUDE':
                    telemetry_data["ida"]["roll"] = round(msg.roll, 3)
                    telemetry_data["ida"]["pitch"] = round(msg.pitch, 3)
                    telemetry_data["ida"]["yaw"] = round(msg.yaw, 3)
                
                elif msg_type == 'VFR_HUD':
                    telemetry_data["ida"]["speed"] = round(msg.groundspeed, 1)
                    telemetry_data["ida"]["heading"] = msg.heading
                
                elif msg_type == 'NAV_CONTROLLER_OUTPUT':
                    telemetry_data["ida"]["target_heading"] = msg.nav_bearing
                    telemetry_data["ida"]["dist_to_wp"] = msg.wp_dist
                    telemetry_data["ida"]["target_speed"] = 1.5 if msg.wp_dist > 2 else 0.0

                elif msg_type == 'SERVO_OUTPUT_RAW':
                    # --- GÜVENLİ PWM OKUMA (Hata Çözümü) ---
                    # MAVLink versiyonuna göre 9 ve 11 numaralı kanallar olmayabilir.
                    # getattr(obje, 'nitelik', varsayılan_değer) ile güvenli okuma yapıyoruz.
                    s1 = getattr(msg, 'servo1_raw', 0)
                    s3 = getattr(msg, 'servo3_raw', 0)
                    s9 = getattr(msg, 'servo9_raw', 0)
                    s11 = getattr(msg, 'servo11_raw', 0)
                    
                    pwm_left = s9 if s9 > 0 else s1
                    pwm_right = s11 if s11 > 0 else s3
                    
                    # Eğer kanallarda hiç değer yoksa nötr (1500) kabul et
                    if pwm_left == 0: pwm_left = 1500
                    if pwm_right == 0: pwm_right = 1500
                    
                    telemetry_data["ida"]["motor_left_pwm"] = pwm_left
                    telemetry_data["ida"]["motor_right_pwm"] = pwm_right
                    
                    left_pct = (pwm_left - 1500) / 5.0
                    right_pct = (pwm_right - 1500) / 5.0
                    
                    telemetry_data["ida"]["motor_left_pct"] = round(left_pct, 1)
                    telemetry_data["ida"]["motor_right_pct"] = round(right_pct, 1)

                    MAX_RPM = 5000 
                    telemetry_data["ida"]["rpm_left"] = int((abs(left_pct) / 100) * MAX_RPM)
                    telemetry_data["ida"]["rpm_right"] = int((abs(right_pct) / 100) * MAX_RPM)

                elif msg_type == 'MISSION_CURRENT':
                    telemetry_data["ida"]["current_wp"] = msg.seq

                elif msg_type == 'SYS_STATUS':
                    telemetry_data["ida"]["voltage"] = round(msg.voltage_battery / 1000.0, 2)
                    telemetry_data["ida"]["battery_percent"] = msg.battery_remaining
                
                elif msg_type == 'HEARTBEAT':
                    if msg.get_srcComponent() == 1: 
                        telemetry_data["ida"]["armed"] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        
                        rover_modes = {
                            0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 
                            5: "LOITER", 6: "FOLLOW", 7: "SIMPLE", 10: "AUTO", 
                            11: "RTL", 12: "SMART_RTL", 15: "GUIDED"
                        }
                        telemetry_data["ida"]["mode"] = rover_modes.get(msg.custom_mode, f"MOD_{msg.custom_mode}")

                elif msg_type == 'COMMAND_ACK':
                    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        sonuc_mesaji = {
                            0: "BAŞARILI (Kabul Edildi)",
                            3: "REDDEDİLDİ (Desteklenmiyor / Yanlış Mod)",
                            4: "GEÇİCİ OLARAK REDDEDİLDİ (Pre-Arm Hatası / Sensör Hazır Değil)"
                        }.get(msg.result, f"Bilinmeyen Kod: {msg.result}")
                        add_sys_log(f"PIXHAWK YANITI -> ARM/DISARM Komutu: {sonuc_mesaji}")

                elif msg_type in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                    seq = msg.seq
                    if seq < len(mission_waypoints):
                        wp = mission_waypoints[seq]
                        master.mav.mission_item_int_send(
                            master.target_system,
                            master.target_component,
                            seq,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1, 0, 0, 0, 0,
                            int(wp['lat'] * 1e7),
                            int(wp['lon'] * 1e7),
                            float(wp.get('alt', 0.0))
                        )

                elif msg_type == 'MISSION_ACK':
                    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        add_sys_log("GÖREV BAŞARIYLA YÜKLENDİ!")
                        telemetry_data["system"]["mission_uploaded"] = True
                        telemetry_data["system"]["mission_waypoint_count"] = len(mission_waypoints)
                    else:
                        add_sys_log(f"GÖREV YÜKLEME HATASI! Pixhawk Red Kodu: {msg.type}")

                elif msg_type == 'GLOBAL_POSITION_INT':
                    telemetry_data["ida"]["lat"] = msg.lat / 1e7
                    telemetry_data["ida"]["lon"] = msg.lon / 1e7
                elif msg_type == 'GPS_RAW_INT':
                    telemetry_data["system"]["gps_sats"] = msg.satellites_visible
                    telemetry_data["ida"]["sats"] = msg.satellites_visible
                    telemetry_data["ida"]["fix_type"] = msg.fix_type

            await asyncio.sleep(0.01)
            
    except Exception as e:
        print(f"Bağlantı hatası: {e}")
        telemetry_data["system"]["ida_link"] = "DISCONNECTED"
        telemetry_data["ida"]["connected"] = False

async def websocket_handler(websocket):
    print("Müjde! Ezel GCS Arayüzü (Frontend) Bağlandı! GERÇEK OPERASYON MODU AKTİF.")
    
    async def consumer():
        global mission_waypoints
        async for message in websocket:
            try:
                data = json.loads(message)
                if data.get("type") == "command":
                    cmd = data.get("command")
                    
                    if cmd == "ARM_IDA" and master:
                        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                        await websocket.send(json.dumps({"type": "command_result", "payload": {"command": cmd, "ok": True, "status": "success", "message": "ARM Sinyali İletildi."}}))
                        
                    elif cmd == "DISARM_IDA" and master:
                        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
                        await websocket.send(json.dumps({"type": "command_result", "payload": {"command": cmd, "ok": True, "status": "success", "message": "DISARM Sinyali İletildi."}}))
                    
                    elif cmd == "UPLOAD_IDA_MISSION" and master:
                        mission_waypoints = data.get("payload", {}).get("waypoints", [])
                        add_sys_log(f"ARAYÜZDEN GÖREV GELDİ: {len(mission_waypoints)} Adet Nokta Yükleniyor...")
                        
                        master.mav.mission_clear_all_send(master.target_system, master.target_component)
                        master.mav.mission_count_send(master.target_system, master.target_component, len(mission_waypoints))
                        
                        await websocket.send(json.dumps({"type": "command_result", "payload": {"command": cmd, "ok": True, "status": "success", "message": "Görev Yükleme Başlatıldı."}}))

                    elif cmd == "CLEAR_IDA_MISSION":
                        mission_waypoints = []
                        if master:
                            master.mav.mission_clear_all_send(master.target_system, master.target_component)
                            telemetry_data["system"]["mission_waypoint_count"] = 0
                            add_sys_log("Hafızadaki eski görevler SİLİNDİ!")
                        await websocket.send(json.dumps({"type": "command_result", "payload": {"command": cmd, "ok": True, "status": "success", "message": "Hafıza Temizlendi."}}))

                    elif cmd == "START_IDA_MISSION" and master:
                        add_sys_log("GÖREV BAŞLATILIYOR (AUTO MODUNA GEÇİLİYOR)")
                        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 10, 0, 0, 0, 0, 0)
                        await websocket.send(json.dumps({"type": "command_result", "payload": {"command": cmd, "ok": True, "status": "success", "message": "Otonom Görev Başlatıldı (AUTO Mod)!"}}))

            except Exception as e:
                print(f"Komut işleme hatası: {e}")

    async def producer():
        while True:
            paket = {"type": "telemetry", "payload": telemetry_data}
            await websocket.send(json.dumps(paket))
            await asyncio.sleep(0.1)

    consumer_task = asyncio.create_task(consumer())
    producer_task = asyncio.create_task(producer())
    done, pending = await asyncio.wait([consumer_task, producer_task], return_when=asyncio.FIRST_COMPLETED)
    for task in pending: task.cancel()

async def main():
    asyncio.create_task(read_mavlink())
    asyncio.create_task(gcs_heartbeat())
    print("WebSocket Sunucusu başlatılıyor... (ws://localhost:5000)")
    async with websockets.serve(websocket_handler, "localhost", 5000):
        await asyncio.Future()

if __name__ == "__main__":
    raise SystemExit(
        "GUVENLIK: telemetri_oku.py eski ve ACK'siz komut yoludur; "
        "calistirilamaz. Kanonik backend: uvicorn main:app --host 0.0.0.0 --port 5000"
    )
