# EZEL GCS MAVLink Backend

Bu backend iki ayrı seri/RFD900x bağlantısından İDA ve İHA Pixhawk telemetrisi okur, frontend'e WebSocket üzerinden yayınlar ve tüm araç komutlarını merkezi güvenlik kapısından geçirir.

## Kurulum

Python 3.10+ kullanın:

```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 5000
```

Backend `ws://localhost:5000/ws` adresinde çalışır. COM portları açılamazsa backend kapanmaz; hata loglar ve yeniden bağlanmayı dener.

`telemetri_oku.py` eski, ACK'siz AUTO/ARM deneme sunucusudur ve doğrudan
çalıştırılması kod seviyesinde engellenmiştir. Port 5000 için tek kanonik giriş
yalnız `backend/main.py` üzerindeki FastAPI/uvicorn uygulamasıdır.

Laboratuvar debug yüzeyi varsayılan olarak kapalıdır. Yalnız sim/lab kullanımında
`EZEL_LAB_DEBUG_ENABLED=true`, frontend için `VITE_ENABLE_LAB_DEBUG=true` ve
browser tokenından farklı bir `EZEL_JETSON_DEBUG_TOKEN` ayarlanır. Jetson monitor
üreticisi `ws://localhost:5000/ws/jetson-debug` adresine
`X-EZEL-JETSON-DEBUG-TOKEN` WebSocket handshake başlığıyla bağlanır.
Browser `/ws` üzerinden snapshot enjekte edemez; backend test sonucu hesaplamaz,
yalnız kanonik `contracts/ida_vehicle_test.v1.json` oturumlarını monitor sonucu ile
eşleştirir. Ham kamera/video bu kontratta yoktur.
Terminal `/vehicle_test/result` producer tarafından backend'in
`vehicle_test_upstream_ack` uygulama onayı gelene kadar tutulur; bağlantı kaybında
aynı `run_id`/`seq` ve birebir payload yeniden gönderilir. Backend bu exact replay'i
idempotent kabul eder, aynı kimlikte farklı payload'ı reddeder.

Bu lab WebSocket'leri düz `ws://` taşır ve yalnız güvenilir, izole laboratuvar
LAN'ında kullanılmalıdır. Başka bir ağdan erişim gerekiyorsa backend'i doğrudan
internete açmayın; TLS sonlandıran bir reverse proxy (`wss://`) veya doğrulanmış
bir VPN kullanın. Browser ve Jetson producer tokenlarını ayrı, uzun ve rastgele
tutun; lab modu açılırken backend bunu zorunlu olarak doğrular.

`--host 0.0.0.0` backend'i yerel ağa açar. Araç komutları WebSocket token auth olmadan çalışmaz; buna rağmen port `5000` erişimini yarışma/YKİ ağıyla sınırlayın.

Donanımsız servis testleri:

```bash
python -m unittest discover -s tests -v
```

## COM Port Ve Güvenlik Ayarları

Saha bilgisayarına göre [config.py](config.py) içindeki değerleri düzenleyin:

```python
IDA_PORT = "COM1"
IHA_PORT = "COM2"
BAUDRATE = 57600
IDA_SYS_ID = 1
IHA_SYS_ID = 2
GCS_SYS_ID = 255
IDA_COMPANION_COMPONENT_ID = 191
```

`TGT_ACK` yalnız bu component kimliğinden kabul edilir. Jetson
`companion_component_id` launch parametresi ve YKİ
`EZEL_IDA_COMPANION_COMPONENT_ID` değeri aynı olmalıdır.

Windows'ta Aygıt Yöneticisi, Linux'ta `ls /dev/ttyUSB*` ile modem portlarını doğrulayın. İki modem için SYS_ID ayrımı tek başına yeterli değildir; doğru fiziksel modem-port eşleşmesini de etiketleyin.

Varsayılan emergency politikası:

```python
EMERGENCY_IDA_ACTION = "HOLD"
EMERGENCY_IHA_ACTION = "RTL"
```

İHA için havada doğrudan `DISARM` ciddi düşüş riski taşır. Yalnız saha prosedürü ve araç konfigürasyonu doğrulandıysa değiştirin. `EMERGENCY_STOP_ALL` emergency kilidini aktif tutar; bu sırada ARM, mission start ve mission upload reddedilir. Link kopukken emergency verildiyse, link geri geldiğinde backend emergency action'ı otomatik yeniden dener ve sonucu loglar.

`RESET_EMERGENCY` yalnız iki araç bağlantısı doğrulanmış ve iki araç da disarmed iken kilidi kaldırır. Reset komutu araçlara ARM veya mode komutu göndermez.

Komut auth ayarı:

```bash
export EZEL_WS_TOKEN="sahada-uzun-rastgele-token"
```

Frontend tarafında aynı token proje kökündeki `.env.local` içine yazılmalıdır:

```bash
VITE_WS_TOKEN=sahada-uzun-rastgele-token
```

`EZEL_WS_TOKEN` ayarlı değilse backend telemetri yayınlar fakat araç komutlarını `unauthorized` olarak reddeder. Sadece kapalı laboratuvar testinde `EZEL_WS_AUTH_REQUIRED=false` kullanılabilir.

Return-home / breakpoint davranışı:

```bash
export EZEL_AUTO_RETURN_ON_LINK_LOSS=true
export EZEL_RETURN_HOME_MODE=RTL
```

Backend ilk geçerli GPS konumunu araç için home/breakpoint olarak kaydeder. Link heartbeat timeout ile kopmuş kabul edilirse backend dönüş isteğini pending duruma alır ve link yeniden bağlandığında `RTL` modunu bir kez dener. MAVLink/RFD linki gerçekten kopukken GCS komut gönderemez; bu yüzden gerçek sinyal kaybında dönüş garantisi Pixhawk failsafe/RTL konfigürasyonuyla sahada ayrıca doğrulanmalıdır.

## Frontend Mock Modu

Frontend varsayılan olarak gerçek backend'e bağlanır. Mock telemetri kullanmak için proje kökünde `.env.local` oluşturun:

```bash
VITE_USE_MOCK=true
```

Farklı WebSocket adresi için:

```bash
VITE_WS_URL=ws://localhost:5000/ws
VITE_WS_TOKEN=sahada-uzun-rastgele-token
```

## Loglar

Her backend başlangıcında `backend/logs/` altında şu dosyalar oluşturulur:

- `telemetry_YYYYMMDD_HHMMSS.csv`
- `commands_YYYYMMDD_HHMMSS.jsonl`
- `system_YYYYMMDD_HHMMSS.log`

## İlk Saha Test Sırası

Testleri pervane/motor güvenliği sağlanmış kontrollü ortamda yapın:

1. İDA heartbeat: `/health` içinde `ida_link=CONNECTED`
2. İHA heartbeat: `/health` içinde `iha_link=CONNECTED`
3. WebSocket telemetri: frontend link ve araç verileri güncelleniyor
4. ARM/DISARM test: önce motorsuz veya yükseltilmiş güvenli platformda
5. Mission upload test: kısa ve görünür İDA rotası
6. Emergency stop test: İDA `HOLD`, İHA `RTL` davranışı araç tarafında doğrulanıyor
7. Emergency reset test: iki araç disarmed iken reset kabul ediliyor, biri armed iken reddediliyor
8. Link-loss return-home test: Pixhawk failsafe/RTL gerçek link kaybında başlangıç noktasına dönüşü sağlıyor

## Önemli Sınırlar

- Komut sonuçları `status` alanı taşır: `state_verified` en güçlü doğrulama, `acked` Pixhawk ACK, `sent_unconfirmed` ise yalnız hatta yazma anlamına gelir.
- ARM/DISARM ve mode değişiklikleri COMMAND_ACK bekler, mümkünse heartbeat state ile doğrulanır. ACK veya state gelmezse frontend açık uyarı gösterir.
- Mission upload MAVLink request/ACK akışını bekler ve zaman aşımını loglar.
- `START_IDA_MISSION`, yalnız son mission upload başarılıysa kabul edilir.
- Görev başlangıcı Pixhawk AUTO mission değildir. Backend GUIDED modunu
  doğruladıktan sonra `SCR_USER6` mailbox'a sıralı START durumu yazar; Jetson
  bridge otonomiye `/mission/start` iletir ve yalnız otonomi uyguladıktan sonra
  aynı mailbox'ı ACK durumuna çevirir. ACK yoksa başlangıç başarısızdır, araç HOLD'a alınır
  ve sıra bütünlüğü için pending token overwrite edilmez.
- Görev durdurma önce HOLD, sonra `SCR_USER6` STOP durumu ve eşleşen ACK ister.
  Bekleyen komut varken yeni START/STOP yazılmaz.
- `SCR_USER4/5/6` sırasıyla hedef renk, paketlenmiş P1/P2 adetleri ve mission
  command/ACK mailbox içindir; takım stack'inin `SCR_USER1/2/3` alanları değişmez.
- Manuel hedef kilidi başarılı olunca frontend hedef bilgisini İDA Pixhawk `SCR_USER4` parametresine yazar. Takım stack'inin `SCR_USER1/2/3` alanlarına dokunulmaz. `PARAM_VALUE` geri-okuması `acked`, Jetson'un hedefi okuyup gönderdiği eşleşen `TGT_ACK` ise `state_verified` sonucudur. İHA ve İDA birbirine doğrudan bağlanmaz; iki araç YKİ'de ayrı MAVLink/RFD bağlantılarıdır.
- Reconnect sonrası normal kritik komutlar otomatik tekrar gönderilmez; yalnız aktif emergency latch varsa emergency action yeniden denenir.
- Gerçek saha kullanımı öncesi araç parametreleri, failsafe, flight mode eşlemeleri ve RFD ağ ayarları ayrı ayrı doğrulanmalıdır.
