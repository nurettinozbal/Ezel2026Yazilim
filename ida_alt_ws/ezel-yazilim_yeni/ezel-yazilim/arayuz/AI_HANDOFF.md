# EZEL GCS AI Handoff

Bu doküman, projeyi başka bir mühendise veya başka bir AI araca devrederken okunacak ana özet dosyasıdır. Kaynak kodlarla birlikte gönderildiğinde karşı tarafın AI'ı önce bu dosyayı, sonra `AGENTS.md`, `docs/`, `backend/README.md` ve kritik kaynak dosyaları okumalıdır.

## Kısa Özet

EZEL GCS, React/Vite tabanlı yer kontrol arayüzü ve Python FastAPI/pymavlink tabanlı MAVLink backend'inden oluşur.

Backend iki ayrı RFD/Pixhawk bağlantısını yönetmek üzere tasarlanmıştır:

- İDA Pixhawk: `SYS_ID=1`
- İHA Pixhawk: `SYS_ID=2`
- YKİ/GCS: `SYS_ID=255`

Frontend backend'e WebSocket üzerinden bağlanır:

```text
ws://localhost:5000/ws
```

Frontend varsayılan olarak gerçek backend modundadır. Mock mod yalnız `VITE_USE_MOCK=true` ile açılır.

## Kaynak Gerçekler

Kaynak gerçek dosyalar:

- `src/`: React frontend kaynak kodu
- `backend/`: FastAPI, MAVLink, command gate ve servisler
- `docs/`: mimari, risk, kalite kapısı ve test senaryoları
- `backend/README.md`: backend kurulum ve saha notları
- `package.json`: frontend script/dependency gerçekleri

Kaynak gerçek sayılmaması gerekenler:

- `node_modules/`
- `dist/`
- `backend/logs/`
- `yazilmis_kodlar.txt`

`yazilmis_kodlar.txt` yalnız AI'a hızlı snapshot göstermek için üretilir. Karar verirken gerçek kaynak dosyalar okunmalıdır.

## Çalıştırma

Frontend:

```bash
npm install
npm run dev
```

Backend:

```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 5000
```

Saha/komut güvenliği için backend ve frontend token aynı olmalıdır:

```bash
export EZEL_WS_TOKEN="uzun-rastgele-saha-token"
```

Frontend `.env.local`:

```bash
VITE_WS_URL=ws://localhost:5000/ws
VITE_WS_TOKEN=uzun-rastgele-saha-token
```

Mock frontend için:

```bash
VITE_USE_MOCK=true
```

## Kritik Mimari Akışlar

Telemetry akışı:

```text
Pixhawk/RFD -> MAVLinkVehicle -> TelemetryState -> FastAPI WebSocket -> telemetryService -> VehicleContext -> UI
```

Komut akışı:

```text
UI -> telemetryService -> WebSocket auth -> CommandGate -> MAVLinkVehicle -> Pixhawk
```

Mission upload akışı:

```text
MissionPlanner(P1/P2/P3) -> VehicleContext -> UPLOAD_IDA_MISSION
  -> MissionManager -> MAVLink NAV_WAYPOINT mission protocol
  -> SCR_USER5=(P1*1001+P2) paketli metadata -> PARAM_VALUE readback
  -> Jetson mission_raw + doğrulanmış adetlerle parkur ayrımı
  -> SCR_USER6 command/ACK mailbox ile GUIDED START/HOLD+STOP el sıkışması
```

Target akışı (manuel):

```text
Dashboard manual color -> LOCK_TARGET -> SEND_TARGET_TO_IDA
  -> IDA Pixhawk PARAM_SET(SCR_USER4) -> PARAM_VALUE readback
  -> Jetson reads SCR_USER4 -> TGT_ACK -> state_verified
```

Target akışı (İHA otonom, şartname §5.5.3.1):

```text
İHA NAMED_VALUE_INT -> MAVLinkVehicle._handle_named_value_int -> handler thread
  -> targets.lock_target(source="IHA") -> CommandGate SEND_TARGET_TO_IDA -> İDA
```

## İHA Otonom Hedef Tespiti — MAVLink Kontratı

**İHA tarafındaki bilgisayarın uygulaması gereken sözleşme budur.** Bu mesaj
yayınlanmadığı sürece YKİ'de hedef rengi `MANUEL` kaynağıyla, yani operatörün elle
girdiği değer olarak görünür.

İHA (`SYS_ID=2`), plakayı tespit ettiğinde şu MAVLink mesajını yayınlar:

| Alan | Değer |
|---|---|
| Mesaj | `NAMED_VALUE_INT` |
| `name` | `"TARGET_COLOR"` |
| `value` | `1=KIRMIZI`, `2=YEŞİL`, `4=SİYAH` |

```python
master.mav.named_value_int_send(
    int(time.monotonic() * 1000) & 0xFFFFFFFF,
    b"TARGET_COLOR",
    1,  # KIRMIZI
)
```

Dikkat edilmesi gerekenler:

- **`name` alanı MAVLink'te `char[10]`'dur.** `"TARGET_COLOR"` 12 karakter olduğu için
  hat üzerinde `"TARGET_COL"` olarak kırpılır. Backend her iki biçimi de kabul eder
  (`services/target_manager.py: TARGET_COLOR_FIELD_NAMES`); İHA tarafında ekstra bir
  şey yapmak gerekmez.
- Mesaj **tekrar tekrar yayınlanabilir**; backend yalnız renk *değiştiğinde* işler.
- **İlk tespit kazanır.** Hedef bir kez kilitlendikten sonra ikinci bir tespit
  yok sayılır — şartname §5.5.3.1 hedef bilgisinin İDA harekete başladıktan sonra
  aktarılmasını yasakladığı için bu davranış kasıtlıdır.
- Renk kodları tek kaynaktan gelir: `backend/services/target_manager.py: COLOR_CODES`.
  İDA Parkur-3 kontratı kırmızı/yeşil/siyah olduğundan başka kod kabul edilmez.

## YKİ -> İDA hedef teslimi

İHA ile İDA arasında doğrudan bağlantı yoktur. İHA yukarıdaki mesajı yalnız kendi
RFD hattından YKİ'ye yollar. YKİ hedefi kilitledikten sonra İDA'nın ayrı RFD
bağlantısı üzerinden Pixhawk `SCR_USER4` parametresine `PARAM_SET` gönderir.
Takım stack'inin `SCR_USER1/2/3` alanları canonical akış tarafından yazılmaz.
`PARAM_VALUE` geri-okuması yalnız `acked` sayılır. Jetson hedefi gerçekten okuyup
ROS görev kontratına yayınlayınca routed MAVLink üzerinden `TGT_ACK` yollar; ancak
bu eşleşen kod geldiğinde teslim `state_verified` olur.

Jetson tarafında eski `ida_ws_gateway` kullanılmaz. Jetson Pixhawk portunu yalnız
`mavlink-routerd` açar; MAVSDK 14540, ida_control pymavlink/YKİ durumu 14541
endpoint'indedir. Bu karar çift seri-port sahibini ve telemetri ACK yarışını önler.

## Kritik Dosyalar

Backend:

- `backend/main.py`: FastAPI app, `/health`, `/ws`, WebSocket auth, broadcast loop
- `backend/config.py`: COM port, SYS_ID, emergency action, token/env ayarları
- `backend/links/mavlink_vehicle.py`: MAVLink bağlantı, reconnect, COMMAND_ACK, mission upload, emergency reassert
- `backend/services/command_gate.py`: tüm frontend komutları için merkezi güvenlik kapısı
- `backend/services/telemetry_state.py`: normalize telemetri ve bağlantı/armed/mode state'i
- `backend/services/mission_manager.py`: son başarılı mission upload durumu
- `backend/services/target_manager.py`: hedef kilidi ve hedef delivery status
- `backend/services/logger.py`: telemetry/command/system logları

Frontend:

- `src/services/telemetryService.js`: WebSocket/mock transport, reconnect, auth message, command send
- `src/context/VehicleContext.jsx`: telemetry merge, UI command intent, logs, emergency/mission/target actions
- `src/context/useVehicle.jsx`: yalnız paylaşılan context nesnesi + `useVehicle` hook'u (Provider tanımlamaz)
- `src/features/EmergencySystem/components/KillSwitch.jsx`: emergency stop ve controlled reset UI
- `src/features/MissionControl/components/MissionPlanner.jsx`: waypoint upload UI
- `src/features/MissionControl/components/BottomControlBar.jsx`: ARM ve mission start UI
- `src/features/TelemetryPanel/components/DashboardView.jsx`: manuel hedef renk girişi

## WebSocket Mesajları

Backend -> frontend telemetry:

```json
{
  "type": "telemetry",
  "data": {
    "ida": {},
    "iha": {},
    "system": {}
  }
}
```

Frontend -> backend auth:

```json
{
  "type": "auth",
  "token": "VITE_WS_TOKEN"
}
```

Frontend -> backend command:

```json
{
  "type": "command",
  "command": "ARM_IDA",
  "payload": {}
}
```

Backend -> frontend command result:

```json
{
  "type": "command_result",
  "data": {
    "ok": true,
    "command": "ARM_IDA",
    "message": "Açıklama",
    "status": "state_verified"
  }
}
```

Önemli `status` anlamları:

- `state_verified`: telemetri state'i komutu doğruladı
- `acked`: Pixhawk `COMMAND_ACK` kabul etti
- `acked_unverified`: ACK geldi ama telemetry state henüz doğrulamadı
- `sent_unconfirmed`: MAVLink hattına yazıldı ama araçtan onay yok
- `rejected`: command gate veya Pixhawk reddetti
- `timeout`: beklenen ACK/mission yanıtı gelmedi
- `unauthorized`: WebSocket command auth yok veya token eşleşmedi
- `blocked_unverified`: bir sonraki güvenli adım için önceki state doğrulanamadı

## Güvenlik Kararları

- Araç komutları token auth olmadan çalışmaz.
- Telemetri auth'suz görülebilir; komutlar auth ister.
- `EMERGENCY_STOP_ALL` her zaman izinli command gate komutudur.
- Emergency aktifken ARM, mission start ve mission upload reddedilir.
- Emergency aktifken link geri gelirse backend emergency action'ı yeniden dener.
- `RESET_EMERGENCY` sadece iki araç bağlı ve ikisi de disarmed ise emergency kilidini kaldırır.
- `RESET_EMERGENCY` ARM göndermez; İDA'yı HOLD'da doğrular ve Jetson STOP
  uygulama ACK'i gelmeden emergency kilidini açmaz.
- `START_IDA_MISSION`, yalnız son başarılı mission upload varsa kabul edilir.
- `START_IDA_MISSION`, önce GUIDED modunu doğrular; ardından takım stack'inin
  `SCR_USER1/2/3` alanlarına dokunmadan `SCR_USER6` mailbox'a tek kullanımlık
  START durumu yazar ve Jetson'un aynı mailbox'ı ACK durumuna çevirmesini bekler.
  Pixhawk AUTO mission yürütmesi kullanılmaz.
- `STOP_IDA_MISSION`, önce HOLD modunu doğrular; `SCR_USER6` mailbox'a STOP
  durumu yazar ve Jetson STOP ACK'i gelmeden görev kilidini kaldırmaz.
- `RESET_EMERGENCY`, iki araç bağlı/disarmed koşuluna ek olarak Jetson STOP
  uygulama ACK'ini doğrulamadan emergency kilidini kaldırmaz.
- Hedef kilitlendikten sonra normal `LOCK_TARGET` hedefi değiştirmez; güvenli/emergency durumda `FORCE_UPDATE_TARGET` gerekir.
- İDA otonom görevi başladıktan sonra (şartname §5.5.3.1) güvenli çıkış olan
  `STOP_IDA_MISSION` ve `DISARM_IDA` ile `EMERGENCY_STOP_ALL` dışındaki İDA
  komutları reddedilir. Kilit yalnız doğrulanmış STOP ACK, kontrollü emergency
  reset veya yeni başarılı mission upload ile kalkar. İHA komutları bilinçli
  olarak kilit dışıdır.
- `SET_RADIO_FREQUENCY` yalnız araç bağlı, disarmed, emergency değil ve İDA görevi başlamamışken kabul edilir.

## Bilinen Sınırlar

Bu sınırlar bilinçlidir ve AI/mühendis yanlış “tamamlandı” varsayımı yapmamalıdır:

- Donanım bağlı değilken Pixhawk gerçek davranışı kanıtlanamaz.
- Hedef gönderimi `SCR_USER4` yazımı + `PARAM_VALUE` geri-okuması ile `acked`, Jetson'un eşleşen `TGT_ACK` mesajıyla `state_verified` olur. Eşleşmeyen veya gelmeyen ACK görev doğrulaması değildir.
- Şartname §4.1 yer tarafına görüntü aktarımını yasakladığı için arayüzde kamera/video bileşeni yoktur ve eklenmemelidir.
- RFD900x frekans yazma akışı (`SET_RADIO_FREQUENCY`) kodda gerçektir fakat donanımsız doğrulanamaz; saha testi zorunludur.
- `ida.target_speed` yalnız otopilot `POSITION_TARGET_*` yayınladığında dolar (AUTO/GUIDED). MANUAL modda boş kalması normaldir.
- İHA otonom hedef tespiti ancak İHA bilgisayarı `NAMED_VALUE_INT("TARGET_COLOR")` yayınlarsa gerçekleşir; yayınlanmazsa hedef `MANUEL` kaynağıyla operatörden alınır. Backend tarafı UDP MAVLink ile uçtan uca doğrulandı, gerçek İHA donanımıyla doğrulanmadı.
- İHA uçuş bölgesi (geofence) yalnız bir YKİ uyarısıdır; araca geofence yüklemez ve uçuşu fiziksel olarak engellemez. Sınır operatör tarafından harita üzerinden girilir ve tarayıcı `localStorage`'ında saklanır.
- Harita tile'ları dış ağdaki Esri servisinden gelir; offline harita yoktur.
- `yazilmis_kodlar.txt` otomatik snapshot'tır, kaynak gerçek değildir.

## Test Komutları

```bash
python3 -m compileall -q backend
npm run test:backend
npm test
npm run lint
npm run build
```

Son bilinen doğrulama:

- Backend unit test: 14 test geçti
- Frontend unit test: 17 test geçti
- ESLint: geçti
- Vite build: geçti, yalnız büyük bundle uyarısı var

## Donanımsız Entegrasyon Testi

1. Backend'i COM portlar yokken başlat.
2. `curl http://localhost:5000/health` ile backend'in ayakta kaldığını doğrula.
3. Frontend'i token olmadan aç; telemetri gelir ama command `unauthorized` dönmelidir.
4. Backend `EZEL_WS_TOKEN`, frontend `VITE_WS_TOKEN` aynı ayarlanır.
5. Frontend console/log içinde auth başarılı mesajı görülür.
6. Araç bağlı değilken IDA/IHA komutları bağlantı yok diye reddedilmelidir.

## Saha Kabul Testi

Donanım bağlandığında güvenli sırayla ilerle:

1. İDA heartbeat ve `SYS_ID=1` doğrula.
2. İHA heartbeat ve `SYS_ID=2` doğrula.
3. `/health` içinde `ida_link=CONNECTED`, `iha_link=CONNECTED` doğrula.
4. Pervane/motor riski yokken ARM/DISARM test et.
5. Komut sonucunda `acked` veya `state_verified` bekle.
6. Kısa IDA mission upload test et ve mission ACK bekle.
7. Mission start öncesi `mission_uploaded=true` doğrula.
8. Emergency stop test et: İDA `HOLD`, İHA `RTL` davranışı gözlenmeli.
9. Emergency aktifken link kes/geri getir; reconnect reassert logunu kontrol et.
10. İki araç disarmed iken `RESET_EMERGENCY` kabul edilmeli; biri armed iken reddedilmeli.
11. Manuel hedef kilitle; önce `acked`, ardından eşleşen Jetson `TGT_ACK` ile `state_verified` görülmeden hedef teslimini tamamlanmış sayma.

## Yeni AI İçin Başlangıç Promptu

Bu projeyi incelemeden önce şu dosyaları sırayla oku:

1. `AI_HANDOFF.md`
2. `AGENTS.md`
3. `docs/project-map.md`
4. `docs/architecture.md`
5. `docs/risk-registry.md`
6. `docs/test-scenarios.md`
7. `backend/README.md`
8. `backend/main.py`
9. `backend/services/command_gate.py`
10. `backend/links/mavlink_vehicle.py`
11. `src/services/telemetryService.js`
12. `src/context/VehicleContext.jsx`

Çalışırken şunları yap:

- `node_modules`, `dist`, `backend/logs`, `yazilmis_kodlar.txt` dosyasını kaynak gerçek kabul etme.
- UI tasarımını gereksiz değiştirme.
- Her komut sonucunda `status` semantiğini koru.
- `sent_unconfirmed` sonucunu başarı gibi sunma.
- WebSocket komut auth'unu bypass etme.
- Emergency ve mission güvenlik kurallarını zayıflatma.
- Yeni değişiklikten sonra `npm run lint`, `npm test`, `npm run test:backend`, `npm run build` çalıştır.

## Öncelikli Sonraki İşler

1. `SCR_USER4 -> Jetson -> TGT_ACK` zincirini gerçek iki RFD hattı ve Pixhawk üzerinde doğrulamak.
2. Donanımlı SITL veya fake MAVLink entegrasyon testi eklemek.
3. Offline map veya saha harita fallback'i eklemek.
4. Kamera/video akışını gerçek input'a bağlamak.
5. RFD modem ayarlarının UI'dan gerçekten yazılması gerekiyorsa ayrı güvenlikli backend endpoint/komut tasarlamak.
