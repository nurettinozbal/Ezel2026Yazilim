# İDA Bench Test Ana Planı

Bu belge, İDA'nın denize indirilmeden önce masada/atölyede uygulanacak bütün
başlangıç ve tam sistem testlerinin ana sırasıdır. Amaç yalnız bileşenlerin ayrı
ayrı çalıştığını görmek değil; YKİ'den Jetson'a, sensörlerden füzyona, otonomi
kararından Pixhawk'a kadar zincirin güvenli ve kanıtlanabilir biçimde çalıştığını
göstermektir.

Bu plan **deniz testi değildir**. Testlerin hiçbiri tekneyi serbest bırakmaz.
Son aşamada tekne denize indirilecek haliyle tamamen monte edilmiş olabilir;
ancak sağlam bir test sehpasına sabitlenir ve motor/itici tehlike alanına hiçbir
insan yaklaşmaz.

## 1. En önemli kurallar

1. Aşamalar sırayla uygulanır. Bir aşama KIRMIZI ise sonraki aşamaya geçilmez.
2. Yazılım üzerinden DISARM veya STOP, fiziksel acil durdurmanın yerine geçmez.
3. Motor gücünü fiziksel olarak kesen, kolay erişilen bağımsız bir anahtar veya
   kontaktör bulunmadan motorlu test yapılmaz.
4. Motorların/iticilerin üreticisi kuru çalıştırmaya izin vermiyorsa kuru motor
   testi yapılmaz. Uygun korumalı su tankı veya üreticinin önerdiği test düzeneği
   kullanılır.
5. Motor tehlike alanında insan, kablo, kıyafet, alet veya gevşek parça bulunmaz.
6. Takımın çalışan Jetson stack'i açıkken bizim canonical gerçek araç launch'ımız
   başlatılmaz.
7. `sent_unconfirmed` başarı değildir. Kritik bir komut için en az Pixhawk ACK'i,
   mümkünse daha sonra gelen gerçek durum doğrulaması gerekir.
8. `ida_vehicle_test` yalnız pasif ölçüm aracıdır. ARM, mod veya motor komutu
   gönderemez ve PASS sonucu motor kullanım izni vermez.
9. Her aşamada ekran videosu, rosbag, terminal çıktısı ve test formu saklanır.
10. Şüpheli ses, koku, ısınma, titreşim, kablo hareketi, veri kesilmesi veya
    beklenmeyen motor hareketinde fiziksel güç hemen kesilir.

## 2. Test durumlarının anlamı

- **YEŞİL:** Beklenen sonuç eksiksiz görüldü, kanıt kaydedildi, sonraki aşamaya
  geçilebilir.
- **SARI:** Sonuç belirsiz veya eksik. Test düzeltilip aynı aşama tekrarlanır.
- **KIRMIZI:** Güvenlik ihlali, yanlış davranış veya veri kaybı var. Test durur;
  motor gücü varsa kesilir ve kök neden çözülmeden devam edilmez.

Hiçbir aşama “bir kez çalıştı” denilerek geçilmiş sayılmaz. Tekrarlanabilir ve
kayıtla doğrulanabilir olmalıdır.

## 3. Görev dağılımı

Motor gücü olmayan aşamalarda en az iki, motorlu aşamalarda en az üç kişi bulunur:

- **Test yöneticisi:** Kontrol listesini okur, aşamanın başlatılmasına izin verir.
- **Operatör:** YKİ ve Jetson komutlarını uygular. Fiziksel güç kesiciye dokunmaz.
- **Güvenlik gözlemcisi:** Yalnız güvenliğe bakar; fiziksel acil durdurmayı tutar.
- **Kayıt sorumlusu:** Video, rosbag, saat ve gözlemleri kaydeder. Bu rol diğer
  rollerden biriyle birleştirilebilir; güvenlik gözlemcisiyle birleştirilmez.

Motorlu test sırasında operatör ve güvenlik gözlemcisi farklı kişiler olmalıdır.

## 4. Test klasörü ve kayıt standardı

Her test günü için ayrı klasör oluşturulur:

```text
bench_YYYYMMDD/
  00_env/
  01_power_off/
  02_pixhawk/
  03_yki/
  04_camera/
  05_lidar/
  06_fusion/
  07_autonomy_shadow/
  08_fail_safe/
  09_takeover_no_motor/
  10_arm_no_propulsion/
  11_motor_pulse/
  12_full_bench/
```

Her aşamanın `NOTLAR.md` dosyasında şunlar bulunur:

- tarih ve başlangıç/bitiş saati,
- testi yapan kişiler,
- Jetson/Pixhawk/YKİ cihaz kimlikleri,
- commit veya kaynak paket hash'i,
- kullanılan model dosyalarının SHA256 değerleri,
- JetPack, ROS, ArduPilot ve Python sürümleri,
- bağlantı şeması ve seri portlar,
- verilen komutlar,
- beklenen ve gerçekleşen sonuç,
- YEŞİL/SARI/KIRMIZI kararı,
- rosbag, video ve log dosyalarının yolu.

## 5. Aşama 0 — Test öncesi toplantı ve değişiklik dondurma

### Amaç

Herkesin aynı kodu, aynı kablo planını ve aynı güvenlik sınırlarını kullandığını
doğrulamak.

### Kontrol listesi

- [ ] Takımın çalışan kodunun yedeği veya commit/hash kaydı alındı.
- [ ] Bizim alternatif workspace'imiz ayrı klasörde.
- [ ] Test günü kod değiştirmeye yetkili tek kişi belirlendi.
- [ ] İDA ve İHA RFD modemleri fiziksel olarak etiketli.
- [ ] Pixhawk SYS_ID, Jetson companion component ID ve YKİ portları yazılı.
- [ ] Fiziksel güç kesicinin yeri ve sorumlusu belli.
- [ ] Teknenin test sehpasına nasıl sabitleneceği belirlendi.
- [ ] Motor üreticisinin kuru çalışma izni kontrol edildi.
- [ ] Test boyunca kimsenin girmeyeceği motor tehlike alanı işaretlendi.

### SCR_USER zorunlu kapısı

Takım stack'i tarafında doğrulanan tahsis:

- `SCR_USER1`: takım fazı (`0/5/10/15`),
- `SCR_USER2`: takım hedef rengi,
- `SCR_USER3`: takım lidar kaçış izni.

Canonical alternatif stack bu alanlara dokunmaz:

- `SCR_USER4`: canonical hedef renk,
- `SCR_USER5`: canonical P1/P2 waypoint sayılarının tek tam sayı paketi
  (`P1*1001+P2`),
- `SCR_USER6`: canonical mission START/STOP ve Jetson ACK mailbox'ı.

Takımın mevcut otonomi kodu `SCR_USER` kullanıyorsa hangi numaraları hangi anlamla
kullandığı yazılı olarak çıkarılmadan YKİ hedef veya görev testi yapılmaz:

```bash
grep -RniE 'SCR_USER[1-6]' /APM/scripts 2>/dev/null
grep -RniE 'SCR_USER[1-6]' <takim-kod-klasoru>
```

Aynı parametreyi iki farklı bileşen yazıyorsa test KIRMIZI'dır. Boş parametreler
ekipçe tahsis edilir; yeterli boş alan yoksa Pixhawk Lua tarafında İDA'ya özel
isimli parametre tasarlanır. Rastgele bir `SCR_USER` numarası seçilmez.

### Geçiş kriteri

Tüm kutular işaretli, parametre tahsis tablosu imzalı ve test düzeninin fotoğrafı
kaydedilmiş olmalıdır.

## 6. Aşama 1 — Enerji vermeden kod ve unit test kontrolü

### Fiziksel durum

- Tahrik bataryası ayrık.
- Motor güç hattı ayrık.
- Pixhawk ve Jetson USB/bench beslemesiyle çalışabilir.

### Jetson ortamı

```bash
cd ~/ida_alt_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Bağımlılıkları otomatik olarak takımın sistem Python'una kurmayın. Önce kontrol:

```bash
python3 -c 'import mavsdk, pymavlink'
python3 -c 'import ultralytics'
command -v mavlink-routerd
```

### ROS testleri

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 tools/check_passive_compatibility.py
```

Son komutta en az şu iki değer görülmelidir:

```text
compatible: true
actuation_enabled: false
```

### YKİ testleri

```bash
cd ~/ezel-yazilim_yeni/ezel-yazilim/arayuz
npm test
npm run lint
npm run build
cd backend
python3 -m unittest discover -s tests -v
```

### KIRMIZI durumlar

- Herhangi bir `FAILED` veya `ERROR`,
- pasif kontrol aracında actuation publisher/import görülmesi,
- dependency'nin yalnız sistem geneli kurulumla çözülebilmesi,
- model hash'inin ekipte kayıtlı değerle eşleşmemesi.

## 7. Aşama 2 — Enerjisiz fiziksel montaj ve kablo kontrolü

### Amaç

Yanlış polarite, gevşek soket, hatalı seri cihaz veya fiziksel çakışmayı enerji
vermeden bulmak.

### Kontrol listesi

- [ ] Batarya polaritesi multimetreyle doğrulandı.
- [ ] Sigorta ve fiziksel güç kesici doğru hatta.
- [ ] Pixhawk, Jetson, kamera ve lidar şaseleri sağlam.
- [ ] `/dev/pixhawk` ve `/dev/lidar` udev bağlantıları doğru fiziksel cihazı gösteriyor.
- [ ] Motor kabloları hareketli parçaya değmiyor.
- [ ] Kamera ve lidar teknenin ileri eksenine göre sabit.
- [ ] Pervane/itici koruması takılı veya üretici test düzeneği hazır.
- [ ] Tekne sehpadan kayamaz, dönemez veya devrilemez.

Kontrol:

```bash
ls -l /dev/pixhawk /dev/lidar
udevadm info /dev/pixhawk
udevadm info /dev/lidar
```

## 8. Aşama 3 — Jetson ve ROS graph kontrolü

### Fiziksel durum

- Motor gücü hâlâ ayrık.
- Takımın mevcut stack'i kullanılacaksa yalnız observe-only test yapılır.

### Mevcut takım stack'ini bozmadan observe-only test

```bash
cd ~/ida_alt_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/check_passive_compatibility.py

export EZEL_JETSON_DEBUG_TOKEN='<browser-tokenindan-farkli-uzun-token>'
ros2 launch ida_bringup vehicle_test_lab.launch.py \
  websocket_url:=ws://<YKI-IP>:5000/ws/jetson-debug \
  simulation_sources:=false \
  evidence_root:=/home/ezel/ida_alt_evidence \
  log_root:=<takimin-mevcut-log-koku>
```

Başlamadan önce ve başladıktan sonra:

```bash
ros2 node list | sort
ros2 topic info -v /perception/buoys
ros2 topic info -v /perception/obstacles
ros2 topic info -v /control/cmd_vel_body
ros2 topic info -v /autonomy/cmd_vel_body
```

Yeni node'lar yalnız `ida_vehicle_test_monitor` ve
`ida_vehicle_test_producer` olmalıdır. Production topic'lerinde yeni publisher
oluşursa KIRMIZI.

## 9. Aşama 4 — Pixhawk bağlantısı, motor gücü kapalı

### Amaç

Tek seri-port sahipliğini ve gerçek telemetri freshness'ini doğrulamak.

Takım stack'i bu noktada tamamen durdurulur. Portun boş olduğu kanıtlanır:

```bash
sudo lsof /dev/pixhawk
```

Çıktı varsa canonical takeover yapılmaz.

İlk canonical bağlantı motor ve araç kurulumu kapalı başlatılır:

```bash
export IDA_CANONICAL_TAKEOVER=true
export IDA_VEHICLE_SETUP_ENABLED=false
export IDA_GUIDED_MODE_ENABLED=false
export IDA_MOTOR_COMMAND_ENABLED=false
./scripts/start_real.sh
```

Kontrol:

```bash
sudo lsof /dev/pixhawk
ss -lunp | grep -E '14540|14541'
ros2 topic echo --once /control/mavsdk_status --full-length
ros2 topic hz /telemetry/state
```

Beklenen:

- `/dev/pixhawk` yalnız `mavlink-routerd` tarafından açık,
- MAVSDK ve pymavlink ayrı UDP endpointlerinde,
- status `dry_run=false`, `connected=true`, `telemetry_ready=true`,
- gerçek acquisition stamp'leri ilerliyor,
- hiçbir servo parametresi yazılmıyor,
- GUIDED moda geçilmiyor,
- `/control/cmd_vel_body` verisi motora çıkmıyor.

Pixhawk bağlantısı kesildiğinde status kısa sürede unhealthy olmalıdır. Bağlantı
geri geldiğinde motor izni kendiliğinden açılmamalıdır.

## 10. Aşama 5 — YKİ backend ve arayüz bağlantısı

### Güvenli başlangıç

Browser ve Jetson tokenları farklı olmalıdır:

```bash
cd ~/ezel-yazilim_yeni/ezel-yazilim/arayuz
export EZEL_WS_AUTH_REQUIRED=true
export EZEL_WS_TOKEN='<uzun-browser-tokeni>'
export VITE_WS_TOKEN='<ayni-uzun-browser-tokeni>'
export EZEL_LAB_DEBUG_ENABLED=true
export VITE_ENABLE_LAB_DEBUG=true
export EZEL_JETSON_DEBUG_TOKEN='<farkli-uzun-jetson-tokeni>'
./start.sh
```

Backend doğrudan internete açılmaz. Bench LAN güvenilir ve izole değilse `ws://`
yerine TLS/VPN kullanılmalıdır.

Kontrol:

- [ ] İDA ve İHA fiziksel RFD portları doğru.
- [ ] İDA ve İHA SYS_ID'leri farklı ve doğru.
- [ ] İDA telemetrisi yalnız İDA panelinde.
- [ ] İHA telemetrisi yalnız İHA panelinde.
- [ ] Bağlantı kesilince arayüz stale/disconnected gösteriyor.
- [ ] Eski otonomi değeri bağlantı kesilince FRESH görünmüyor.
- [ ] Mühendislik/Test sayfasında pasif test kataloğu görülüyor.

YKİ'den henüz ARM, görev başlatma veya motor komutu gönderilmez.

## 11. Aşama 6 — Hedef renk ve görev metadata testi

Bu aşama yalnız SCR_USER tahsisi Aşama 0'da kesinleştirildiyse yapılır.

### Hedef renk

Her renk ayrı ayrı test edilir:

1. YKİ'de kırmızı/yeşil/siyah hedefi kilitleyin.
2. `SEND_TARGET_TO_IDA` gönderin.
3. Önce `acked`, sonra `state_verified` bekleyin.
4. Jetson'da doğrulayın:

```bash
ros2 topic echo --once /mission/target_color --full-length
```

Yanlış renk, yalnız `sent_unconfirmed`, eşleşmeyen `TGT_ACK` veya timeout KIRMIZI.

### Görev metadata

Hareket ettirmeyecek küçük bir bench görevi hazırlanır; yalnız upload edilir,
başlatılmaz. Her waypoint'in P1/P2/P3 seçimi açıkça yapılır.

Beklenen:

- görev upload ACK'i alınır,
- P1 ve P2 adetleri tahsis edilmiş parametrelerde geri okunur,
- `/mission/waypoints` doğru sırayı ve parkur numaralarını taşır,
- metadata eksik/yanlışsa Jetson görevi yayınlamaz,
- **START MISSION düğmesine basılmaz**.

### YKİ–Pixhawk–Jetson görev kontrol köprüsü (motor beslemesi kapalı)

Metadata kabulünden sonra, motor/ESC güç hattı hâlâ fiziksel olarak ayrıkken:

1. `SCR_USER6` mailbox'ın ACK/idle durumda olduğunu doğrulayın (ilk kurulumda
   `8000000`).
2. İDA'yı motorsuz bench prosedürüne göre ARM edin; sonuç yalnız
   `state_verified` ise ilerleyin.
3. Jetson'da `/mission/start` ve `/mission/control_ack` topic'lerini kayda alın.
4. YKİ'de **Görev Başlat** seçin. Pixhawk GUIDED doğrulanmalı, SCR_USER6
   START_CMD durumundan START_ACK durumuna geçmeli ve YKİ
   `mission_started=true` göstermelidir.
5. Her zaman erişilebilir ayrı **Görev Durdur** düğmesini seçin. Önce HOLD, sonra
   STOP_CMD→STOP_ACK ve Jetson `MISSION_READY` doğrulanmalıdır.
6. Köprü kapalıyken START denemesi `timeout` olmalı; araç HOLD'a geçmeli ve
   görev başlamış sayılmamalıdır. Köprü açılınca pending komut ACK olur fakat
   bridge kendiliğinden GUIDED'a geçmez. HOLD korunurken YKİ'den STOP gönderilir.
7. Pending mailbox varken yeni upload, yeni START ve doğrudan GUIDED reddedilir.

Bu alt aşamada `motor_command_enabled=false`; hiçbir motor çıktısı kabul kanıtı
değildir. Kanıt dosyaları YKİ command_result/audit, SCR_USER6 değer dizisi ve iki
ROS topic kaydıdır.

## 12. Aşama 7 — Kamera testleri

### Fiziksel durum

- Motor gücü ayrık.
- Araç sabit.
- Mat turuncu ve sarı boyalı 5 L PET şişeler kullanılır.
- P3 için kırmızı, yeşil ve siyah hedefler ayrı test edilir.

Kontrol:

```bash
ros2 topic hz /camera/image_raw/compressed
ros2 topic echo --once /camera/camera_info
ros2 topic echo --once /perception/camera/p1p2/raw --full-length
ros2 topic echo --once /perception/camera/p3/raw --full-length
```

Her renk için şu pozlar uygulanır:

- merkezde 2 m, 5 m ve mümkünse 10 m,
- yaklaşık 20 derece sağ,
- yaklaşık 20 derece sol,
- görüntü kenarında,
- boş sahne,
- kısmen örtülü hedef,
- iki farklı renk yan yana.

Beklenen:

- acquisition stamp ilerliyor,
- sağ/sol bearing işareti doğru,
- bbox işaretlenen gerçek nesnenin üzerinde,
- yanlış yüksek güvenli renk yok,
- boş sahne fresh-empty olabilir fakat kamera yokluğu fresh-empty görünmez,
- model/inference hatası `stale` veya unhealthy olur.

YKİ Pasif Testler sekmesinden sırayla `camera_p1p2` ve `camera_p3` fixture'ları
çalıştırılır. Poz, renk, beklenen mesafe ve açı formda gerçek düzene göre girilir.

## 13. Aşama 8 — Lidar testleri

Kontrol:

```bash
ros2 topic hz /scan
ros2 topic echo --once /scan
ros2 topic echo --once /perception/lidar/raw_obstacles --full-length
```

Şişe merkez/sağ/sol ve 2/5/10 m konumlarına sırayla yerleştirilir. Mesafeler
mezurayla ölçülür.

Beklenen:

- şişe tutarlı bir cluster üretir,
- sağ/sol fiziksel yönle aynı,
- boş sahnede hayali yakın obstacle yok,
- tekrar yayınlanan aynı acquisition stamp yeni örnek sayılmaz,
- sensör kesildiğinde veri taze görünmez.

YKİ'de `lidar` fixture'ı çalıştırılır. Kabul hedefi başlangıçta en az %95 tarama
varlığı, medyan mesafe hatasının `max(0.15 m, %5)` altında ve bearing p95
hatasının 3 derecenin altında olmasıdır.

## 14. Aşama 9 — Kamera–lidar füzyon testleri

Önce shadow mod kullanılır; motor gücü ayrık kalır.

Sırayla aşağıdaki vakalar uygulanır:

1. Tek turuncu şişe.
2. Tek sarı şişe.
3. İki şişe, farklı renk ve açılar.
4. İki şişenin yerlerini ters çevirme.
5. Açısal olarak üst üste/çok yakın belirsizlik.
6. Kamera kapalı, lidar açık.
7. Lidar kapalı, kamera açık.
8. Kamera ve lidar ikisi de açık fakat hedef yok.
9. Kamera mesajını geciktirme veya eski stamp replay.

Kontrol:

```bash
ros2 topic echo /perception/fusion/status --full-length
ros2 topic echo /perception/fusion/shadow/buoys --full-length
ros2 topic echo /perception/fusion/shadow/obstacles --full-length
```

Beklenen:

- aynı lidar nesnesi iki renge atanmaz,
- belirsizlikte keyfi renk seçilmez,
- kamera yoksa lidar hard/unknown obstacle olarak kalır,
- lidar yoksa kamera tek başına metrik obstacle veya engage hedefi üretmez,
- tipik kamera–lidar zaman farkı 50 ms altında,
- 100 ms üzeri eşleşme reddedilir,
- canonical topiclerde tek publisher vardır.

YKİ'de `fusion_shadow` için `positive`, `negative`, `ambiguity`, `lidar_only` ve
`camera_only` vakaları ayrı ayrı çalıştırılır.

## 15. Aşama 10 — Loglama testi

```bash
ros2 topic echo /logging/status --full-length
```

YKİ'de `logging` pasif testi çalıştırılır. Test yalnız “active=true” değerini
değil, zorunlu kayıt dosyalarının gerçekten büyüdüğünü doğrulamalıdır.

Her run için tek ve benzersiz `run_...` klasörü oluşmalıdır. Eski kayıt üzerine
yazma, `../` ile log kökünden çıkma veya iki launch'ın aynı klasörü kullanması
KIRMIZI'dır.

## 16. Aşama 11 — Otonomi shadow kararı, araç sabit

### Amaç

Şişelerle oluşturulan sahnede sistemin ne karar verdiğini motorlara çıkarmadan
görmek.

### Fiziksel/yazılım durumu

- Motor gücü ayrık veya `motor_command_enabled=false`.
- Araç DISARMED.
- Füzyon shadow/canonical seçimi test formunda açıkça belirtilmiş.
- Tekne önünde iki turuncu şişeyle koridor kurulmuş.

İzlenecek topic'ler:

```bash
ros2 topic echo /autonomy/state --full-length
ros2 topic echo /autonomy/debug --full-length
ros2 topic echo /planning/costmap --full-length
ros2 topic echo /autonomy/cmd_vel_body --full-length
ros2 topic echo /control/cmd_vel_body --full-length
```

Şu sahneler ayrı ayrı uygulanır:

- boş ve açık koridor,
- merkezde unknown obstacle,
- sağa yakın obstacle,
- sola yakın obstacle,
- dar koridor,
- tek duba kaybı,
- kamera kapalı fakat lidar açık,
- lidar kesintisi,
- eski/stale sensör verisi,
- belirsiz kamera–lidar eşleşmesi.

Beklenen karar anlaşılır şekilde kaydedilir: ileri hız, dönüş hızı, seçilen
aksiyon, failsafe nedeni ve costmap obstacle'ları. Motor çıkışı kapalı olduğu için
yanlış karar fiziksel hareket oluşturmaz.

YKİ'de `autonomy_shadow` testi çalıştırılır. Bu test kararların güvenli olduğunu
tek başına kanıtlamaz; yalnız gereken topic'lerin taze ve tutarlı olduğunu
kanıtlar. Her edge case insan tarafından ayrıca incelenir.

## 17. Aşama 12 — Kesinti ve fail-safe testleri

Motor gücü ayrıkken aşağıdaki kesintiler tek tek uygulanır:

- kamera kablosu çıkarma,
- lidar kablosu çıkarma,
- Pixhawk USB/MAVLink bağlantısını kesme,
- YKİ–Jetson WebSocket bağlantısını kesme,
- YKİ RFD bağlantısını kesme,
- aynı acquisition stamp'i tekrar oynatma,
- ROS clock geri alma yalnız izole replay ortamında,
- füzyon publisher'ını durdurma,
- duplicate canonical publisher oluşturmak yerine graph test-double kullanma.

Beklenen ortak davranış:

- veri stale/unhealthy olur,
- eski veri taze diye yeniden yayımlanmaz,
- motor izni veya ARM kendiliğinden açılmaz,
- reconnect eski komutu tekrar çalıştırmaz,
- kamera yokluğunda lidar obstacle kaybolmaz,
- Pixhawk disconnect sonrası bridge health false olur,
- emergency/STOP yolu normal kilitlerden öncelikli kalır.

## 18. Aşama 13 — Tam canonical stack, motor çıkışı kapalı

Bu aşama, denize indirilecek bütün sensörler ve elektronikler araç üzerinde
takılıyken yapılır. Motorlar fiziksel olarak yerinde olabilir; tahrik gücü kapalı
kalır.

```bash
export IDA_CANONICAL_TAKEOVER=true
export IDA_VEHICLE_SETUP_ENABLED=false
export IDA_GUIDED_MODE_ENABLED=false
export IDA_MOTOR_COMMAND_ENABLED=false
export IDA_MODEL_P1P2=/home/ezel/models/parkur12_best.pt
export IDA_MODEL_P3=/home/ezel/models/parkur3_best.pt
export IDA_FUSION_MODEL_LOADED=true
export IDA_CAMERA_CALIBRATED=true
export IDA_LIDAR_CALIBRATED=true
export IDA_EXTRINSICS_CALIBRATED=true
./scripts/start_real.sh
```

Readiness bayrakları yalnız önceki aşamalar gerçekten geçtiyse true yapılır.

Kontrol listesi:

- [ ] Bütün gerekli node'lar yalnız bir kez çalışıyor.
- [ ] Canonical buoy ve obstacle publisher sayısı tam 1.
- [ ] Pixhawk port sahibi yalnız mavlink-router.
- [ ] Kamera ve lidar taze.
- [ ] Füzyon ready/accepted durumu sahneyle uyumlu.
- [ ] Hedef renk `state_verified`.
- [ ] Görev P1/P2/P3 sınıflandırması doğru.
- [ ] Otonomi komutu beklenen yönde.
- [ ] Limiter çıkışı sınırlar içinde.
- [ ] Motor komutu Pixhawk'a iletilmiyor.
- [ ] Loglar ve YKİ debug görünümü çalışıyor.

## 19. Aşama 14 — Elektriksel ARM testi, tahrik etkisiz

Bu aşamada amaç motor döndürmek değil, YKİ ARM/DISARM zincirinin doğru olduğunu
kanıtlamaktır.

Tercih sırası:

1. ESC/motor sinyal hattı ölçülürken tahrik güç hattı ayrık,
2. mümkünse pervane/itici elemanı sökülmüş veya üretici test aparatı kullanılmış,
3. motor mekanik olarak tehlike yaratamıyorsa güç kontrollü biçimde bağlanır.

Kontrol listesi:

- [ ] Fiziksel güç kesici test edildi.
- [ ] YKİ emergency komutu test edildi; fiziksel kesicinin yerine sayılmadı.
- [ ] Araç DISARMED başlıyor.
- [ ] Reboot/reconnect sonrası kendiliğinden ARM olmuyor.
- [ ] YKİ ARM komutu yalnız doğru İDA linkine gidiyor.
- [ ] ARM sonucu fresh heartbeat ile `state_verified` oluyor.
- [ ] DISARM sonucu fresh heartbeat ile doğrulanıyor.
- [ ] `sent_unconfirmed` durumunda test başarısız sayılıyor.
- [ ] İHA ARM durumu İDA panelini değiştirmiyor.

Bu aşamada `motor_command_enabled=false` kalır ve görev başlatılmaz.

## 20. Aşama 15 — Süre sınırlı motor komutu kabul kapısı

### Geçici bench RC override yolu

Bridge içinde yalnız bench doğrulaması için opt-in `bench_rc_override` backend'i
vardır. Bu yol normal otonomi/GUIDED hız kontrolünün yerine geçmez ve production
varsayılanı değildir. Yalnız araç sağlam biçimde sabitlenmişken, motor tehlike
alanı boşken, fiziksel güç kesici güvenlik gözlemcisinin elindeyken ve görev
başlatılmadan kullanılabilir.

Güvenlik sınırları:

- varsayılan backend `guided_velocity` olarak kalır,
- bench backend açıkça seçilmeden RC override gönderilmez,
- `bench_rc_safety_ack=PROPELLER_AREA_CLEAR` exact değeri zorunludur,
- `guided_mode_enabled` ve `vehicle_setup_enabled` false olmak zorundadır,
- yalnız ileri ve düz komut kabul edilir; reverse, yaw veya non-finite değer
  nötr `1500/1500` üretir,
- ileri komut en fazla `0.30 m/s` girişine ve `+100 PWM` farkına clamp edilir,
- komut akışı 0.6 saniye kesilirse çıkış otomatik nötre döner,
- node kapanırken ayrıca nötr RC override gönderilir,
- dokunulmayan RC kanalları `65535` ile serbest bırakılır.

Bridge doğrudan yalnız şu parametrelerle başlatılır:

```bash
ros2 run ida_control mavsdk_bridge_node --ros-args \
  -p dry_run:=false \
  -p system_address:=udpin://0.0.0.0:14540 \
  -p pymavlink_address:=udpin:0.0.0.0:14541 \
  -p vehicle_setup_enabled:=false \
  -p guided_mode_enabled:=false \
  -p motor_command_enabled:=true \
  -p actuation_backend:=bench_rc_override \
  -p bench_rc_safety_ack:=PROPELLER_AREA_CLEAR \
  -p bench_rc_steering_channel:=1 \
  -p bench_rc_throttle_channel:=2 \
  -p bench_rc_neutral_pwm:=1500 \
  -p bench_rc_max_delta_pwm:=100 \
  -p bench_rc_max_forward_mps:=0.30 \
  -p bench_rc_source_system_id:=255 \
  -p mission_raw_enabled:=false
```

Bu komut yalnız bridge'i hazırlar; ARM etmez ve tek başına motor hareketi
oluşturmaz. ARM ayrı ve doğrulanmış komutla yapılır. İlk pulse `linear.x=0.15`,
`angular.z=0.0` olmalıdır; bu yaklaşık `1550 PWM` üretir. En yüksek izinli bench
girişi `linear.x=0.30` yaklaşık `1600 PWM` üretir. Her pulse tek seferlik kısa
publish olmalı; sürekli publisher veya mission/autonomy launch kullanılmamalıdır.

Bench bitiş sırası zorunludur: sıfır komut, RC nötr doğrulaması, DISARM durum
doğrulaması, tahrik gücünü fiziksel kesme, bridge/router kapatma ve seri portun
boş olduğunu kontrol etme.

### Kalıcı güvenlik katmanı için zorunlu blokaj

`vehicle_test_lab.launch.py` bilinçli olarak motor komutu gönderemez. Normal
`START_IDA_MISSION` ise araç sabitken otonominin sürekli hız komutu üretmesine
neden olabilir. Bu nedenle normal görev başlatma, bench motor pulse testi olarak
**kullanılmaz**.

Bu aşama çalıştırılmadan önce ayrı motor bench kontrolü şu özelliklerle
uygulanmış ve unit/HIL testlerinden geçmiş olmalıdır:

- YKİ'de ayrı `motor_test_operator` ve `safety_officer` onayı,
- tek kullanımlık, süreli ve payload'a bağlı permit,
- fiziksel keyed `TEST_ENABLE`,
- basılı tutulması gereken yerel dead-man,
- tek controller lease ve monotonic timeout,
- başlangıç zarfı en fazla 0.1 m/s ileri, lateral 0 ve çok düşük yaw,
- her pulse en fazla 0.5–1.0 saniye,
- otomatik sıfır komutu, HOLD ve ardından DISARM doğrulaması,
- bağlantı, process veya heartbeat kaybında anında sıfır/HOLD,
- replay/double-click/iki kullanıcı yarışında komutun yalnız bir kez çalışması,
- PASS/FAIL/CANCEL ve fiziksel interlock durumlarının değiştirilemez audit kaydı.

Bu güvenlik yüzeyi hazır olmadan `motor_command_enabled=true` ile bench testi
çalıştırılmaz. Yalnız launch parametresinin var olması güvenlik izni değildir.

### Hazır olduğunda test sırası

1. Motor güç hattı kapalıyken pulse isteği gönderilir; sinyal yolu ölçülür.
2. Fiziksel TEST_ENABLE kapalıyken aynı istek reddedilmelidir.
3. Dead-man bırakıldığında komut anında sıfırlanmalıdır.
4. WebSocket kesildiğinde çıkış sıfırlanmalıdır.
5. Jetson bridge durdurulduğunda çıkış sıfırlanmalıdır.
6. Tekne tam sabit, tehlike alanı boş ve üretici koşulları uygunsa güç verilir.
7. Bir motor/itici çok kısa ve en düşük zarfla denenir.
8. Diğer motor/itici ayrı denenir.
9. İki motor eşit ileri pulse ile denenir.
10. Çok düşük diferansiyel/yaw pulse denenir.
11. Her pulstan sonra sıfır, HOLD ve DISARM doğrulanır.
12. Fiziksel güç kesici gerçek hareket sırasında bir kez test edilir.

Beklenmeyen yön, gecikmeli durma, kendiliğinden tekrar, yüksek akım veya titreşim
KIRMIZI'dır.

## 21. Aşama 16 — En kapsamlı, denize hazır tam bench koşusu

Bu son testte araç denize indirilecek fiziksel haliyle hazırdır:

- bütün elektronikler ve sensörler yerinde,
- motorlar/iticiler nihai montajında,
- batarya, sigorta ve güç kesici nihai bağlantısında,
- kamera/lidar kalibrasyonu kilitli,
- YKİ ve iki ayrı RFD bağlantısı hazır,
- tekne sağlam sehpa ve gerekiyorsa üretici uyumlu korumalı su yükünde,
- tehlike alanı tamamen boş.

### Başlatma

Motor güvenlik katmanı Aşama 15'te kabul edilmişse:

```bash
export IDA_CANONICAL_TAKEOVER=true
export IDA_VEHICLE_SETUP_ENABLED=true
export IDA_MOTOR_COMMAND_ENABLED=true
export IDA_PHYSICAL_SAFETY_ACK=PROPELLER_AREA_CLEAR
export IDA_MODEL_P1P2=/home/ezel/models/parkur12_best.pt
export IDA_MODEL_P3=/home/ezel/models/parkur3_best.pt
export IDA_FUSION_MODEL_LOADED=true
export IDA_CAMERA_CALIBRATED=true
export IDA_LIDAR_CALIBRATED=true
export IDA_EXTRINSICS_CALIBRATED=true
./scripts/start_real.sh
```

Bu komut yalnız stack'i ve motor yolunu açar; tek başına ARM veya motor pulse
izni değildir. ARM ve süre sınırlı pulse yalnız Aşama 15'teki izin zincirinden
geçer.

### Tam akış

1. Araç DISARMED ve motor çıkışı sıfır başlar.
2. YKİ, Jetson, Pixhawk, kamera, lidar ve füzyon sağlıkları kontrol edilir.
3. Turuncu/sarı PET şişelerle sabit koridor oluşturulur.
4. Kamera, lidar ve fused nesneler YKİ mühendislik ekranında karşılaştırılır.
5. Otonominin seçtiği hız/yaw kararı motor kapalıyken kaydedilir.
6. Karar insan tarafından güvenli bulunursa kısa motor permit'i hazırlanır.
7. Güvenlik gözlemcisi TEST_ENABLE ve dead-man durumunu doğrular.
8. Araç ARM edilir ve fresh heartbeat ile doğrulanır.
9. Yalnız tek, süre sınırlı düşük pulse uygulanır.
10. Çıkış otomatik sıfıra gelir, HOLD ve DISARM doğrulanır.
11. Kamera/lidar kesintisiyle aynı istek tekrar denenmez; sistemin permit'i
    reddettiği gözlenir.
12. Fiziksel güç kesici doğrulanır.
13. Sistem tekrar açıldığında hiçbir permit veya ARM durumu geri gelmez.

### Final YEŞİL kriterleri

- başlangıçta ve reconnectte hareket yok,
- doğru araç ve doğru motor yönü,
- pulse zarfı ve süresi aşılamıyor,
- bırakılan dead-man veya kesilen link motoru durduruyor,
- emergency fiziksel güç kesme çalışıyor,
- kamera–lidar eşleşmesi yanlış renk üretmiyor,
- lidar-only obstacle güvenli kalıyor,
- otonomi kararı, limiter ve gerçek çıkış zaman olarak ilişkilendirilebiliyor,
- her kritik sonuç fresh durumla doğrulanıyor,
- rosbag, video, YKİ audit ve vehicle-test evidence eksiksiz.

Bu aşama YEŞİL olsa bile doğrudan otonom deniz görevine geçilmez. İlk su testi
ayrı saha planında düşük hız, tether/escort ve geniş güvenlik alanıyla yapılır.

## 22. Ortak rosbag komutu

Kamera topic adı gerçek sistemde farklıysa düzeltilir:

```bash
ros2 bag record -o bench_YYYYMMDD_HHMM_stageXX \
  /scan \
  /camera/image_raw/compressed \
  /camera/camera_info \
  /tf /tf_static \
  /telemetry/state \
  /mission/waypoints \
  /mission/target_color \
  /perception/camera/p1p2/raw \
  /perception/camera/p3/raw \
  /perception/lidar/raw_obstacles \
  /perception/fusion/status \
  /perception/buoys \
  /perception/obstacles \
  /autonomy/state \
  /autonomy/debug \
  /autonomy/score \
  /planning/costmap \
  /autonomy/cmd_vel_body \
  /control/cmd_vel_body \
  /control/mavsdk_status \
  /logging/status \
  /vehicle_test/status \
  /vehicle_test/events \
  /vehicle_test/result
```

Kayıt `Ctrl+C` ile düzgün kapatılır. Rosbag klasörü ziplenebilir. Ekran videosu
rosbag'in yerine geçmez; ikisi birlikte saklanır.

## 23. Her aşamada kullanılacak hızlı sağlık komutları

```bash
ros2 node list
ros2 topic list -t
ros2 topic hz /telemetry/state
ros2 topic hz /scan
ros2 topic info -v /perception/buoys
ros2 topic info -v /perception/obstacles
ros2 topic echo --once /control/mavsdk_status --full-length
ros2 topic echo --once /perception/fusion/status --full-length
ros2 topic echo --once /autonomy/state --full-length
ros2 topic echo --once /autonomy/debug --full-length
```

## 24. KIRMIZI durumda yapılacaklar

Motorlu testte sıra:

1. Fiziksel tahrik gücünü kesin.
2. İnsanları tehlike alanından uzak tutun.
3. Yazılımdan DISARM/HOLD isteyin; fiziksel kesmenin yerine saymayın.
4. Batarya ve ESC sıcaklığını güvenli mesafeden kontrol edin.
5. Terminal, YKİ audit, video ve rosbag'i kapatıp arşivleyin.
6. Kök neden ve düzeltme için yeni test kaydı açın.
7. Aynı aşamayı baştan geçmeden sonraki aşamaya geçmeyin.

## 25. Denize hazır kararı için imza özeti

| Aşama | Sonuç | Sorumlu | Kanıt yolu | Tarih |
|---|---|---|---|---|
| Unit/build |  |  |  |  |
| Fiziksel montaj |  |  |  |  |
| Jetson/ROS graph |  |  |  |  |
| Pixhawk/MAVLink |  |  |  |  |
| YKİ/RFD |  |  |  |  |
| SCR_USER tahsisi |  |  |  |  |
| Kamera P1/P2/P3 |  |  |  |  |
| Lidar |  |  |  |  |
| Füzyon edge case'leri |  |  |  |  |
| Loglama |  |  |  |  |
| Otonomi shadow |  |  |  |  |
| Fail-safe/kesinti |  |  |  |  |
| Full stack, motor kapalı |  |  |  |  |
| ARM, tahrik etkisiz |  |  |  |  |
| Süre sınırlı motor pulse |  |  |  |  |
| Tam bench koşusu |  |  |  |  |

Son satırlar tamamlanmadan “denize hazır” kararı verilmez.
