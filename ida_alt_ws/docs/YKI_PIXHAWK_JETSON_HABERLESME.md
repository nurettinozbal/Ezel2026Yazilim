# YKİ–Pixhawk–Jetson haberleşmesi

## Kesin mimari

İHA ile İDA arasında doğrudan haberleşme yoktur. İki araç YKİ’ye kendi ayrı RFD
bağlantısıyla ulaşır.

```text
İHA -> İHA RFD -> YKİ: TARGET_COL (1 kırmızı, 2 yeşil, 4 siyah)
YKİ -> İDA RFD -> İDA Pixhawk: PARAM_SET SCR_USER4
İDA Pixhawk -> Jetson MAVSDK: SCR_USER4 okuma
Jetson -> mavlink-router -> Pixhawk -> İDA RFD -> YKİ: TGT_ACK
```

`PARAM_VALUE` geri-okuması yalnız Pixhawk’ın parametreyi kabul ettiğini gösterir
ve arayüzde `acked` olur. Jetson aynı hedefi okuyup `/mission/target_color`
mesajını yayınlamadan teslim tamamlanmış sayılmaz. Eşleşen `TGT_ACK` gelince
durum `state_verified` olur.

Görev noktalarının tamamı Pixhawk'a standart `NAV_WAYPOINT` olarak yüklenir;
parkur işareti için çalıştırılabilir MAVLink komutları kötüye kullanılmaz. YKİ
arayüzünde her waypoint P1/P2/P3 olarak seçilir. YKİ doğrulanmış P1 ve P2
adetlerini `P1*1001+P2` formülüyle tek tam sayıya paketleyip `SCR_USER5`e yazar.
Jetson bu metadata olmadan `/mission/waypoints` yayınlamaz. Böylece gateway kaldırıldığında bütün
noktaların yanlışlıkla P1 sayılması fail-closed biçimde önlenir.

Görev başlatma Pixhawk'ın AUTO mission yürütmesi değildir. YKİ, upload ve ARM
kapılarını geçtikten sonra Pixhawk'ı GUIDED moda alır ve aşağıdaki kalıcı el
sıkışmasını kullanır:

```text
YKİ -> İDA Pixhawk SCR_USER6: START_CMD(N) veya STOP_CMD(N)
Jetson bridge -> /mission/start: aynı command token
Otonomi -> /mission/control_ack: token + applied=true
Jetson bridge -> İDA Pixhawk SCR_USER6: START_ACK(N) veya STOP_ACK(N)
İDA Pixhawk -> YKİ: PARAM_VALUE SCR_USER6
```

`SCR_USER6` tek kalıcı mailbox'tır; 8.000.000 tabanlı ayrık command/ACK durumları
kullanır ve bütün değerler float32 içinde tam temsil edilir. Mailbox ACK
durumunda değilken YKİ yeni komut oluşturmaz. START ACK gelmezse YKİ HOLD'a
geçer ve pending START durumunu değiştirmez; sequence atlamak yasaktır.
Köprü aynı tokenı ACK'ledikten sonra operatör normal STOP gönderebilir. STOP önce
HOLD'u, sonra Jetson uygulama ACK'ini doğrular. `RESET_EMERGENCY` de STOP ACK
gelmeden kilidi açmaz.

## Tek seri-port sahibi

Jetson’daki Pixhawk TTY'sini (varsayılan `/dev/ttyACM0`) yalnız repo içindeki
`tty_mavlink_router` açar:

- loopback 14540: MAVSDK telemetri, parametre ve mission okuma,
- loopback 14541: pymavlink motor komutu ve düşük hızlı YKİ durum alanları.

Bu portlar yalnız `127.0.0.1` üzerindedir; fiziksel bağlantı USB TTY'dir ve araç
ağına MAVLink UDP yayını yapılmaz. Harici `mavlink-routerd` paketi gerekmez.

`ida_ws_gateway` kullanılmaz, colcon tarafından derlenmez ve launch edilmez.
MAVSDK veya pymavlink doğrudan `/dev/pixhawk` açmamalıdır.

Kontrol:

```bash
sudo lsof /dev/ttyACM0
ss -lunp | grep -E '14540|14541'
```

İlk komutta tam takeover sırasında yalnız `tty_mavlink_router` görünmelidir.

## Çalışan takım kodunu bozmadan test

Takım stack’i çalışırken `real_vehicle.launch.py` açılmaz. Yalnız pasif test
launch’ı veya ayrı ROS domain’indeki replay kullanılır. `real_vehicle.launch.py`
varsayılan `canonical_takeover_enabled:=false` olduğu için yanlışlıkla
çalıştırılırsa canonical ROS düğümü veya seri-port sahibi başlatmaz.

Tam sahiplik devri ancak takım süreçleri durdurulduktan ve `/dev/pixhawk` boş
görüldükten sonra yapılır. İlk bağlantı testi motor çıkışı olmadan:

```bash
ros2 launch ida_bringup real_vehicle.launch.py \
  canonical_takeover_enabled:=true \
  mavlink_router_enabled:=true \
  dry_run:=false \
  vehicle_setup_enabled:=false \
  motor_command_enabled:=false \
  fusion_model_loaded:=false \
  camera_calibrated:=false \
  lidar_calibrated:=false \
  extrinsics_calibrated:=false
```

Bu mod telemetri ve hedef rengi zincirini sınar; servo parametresi, GUIDED geçişi
ve motor hızı üretmez.

Tam takeover öncesi çalışma ortamı da ayrıca doğrulanır:

```bash
python3 -c 'import mavsdk, pymavlink'
command -v tty_mavlink_router
ls -l /dev/ttyACM0 /dev/ttyUSB0
```

Model yolu verilecekse aynı ROS ortamında `python3 -c 'import ultralytics'` ve
model dosyalarının varlığı doğrulanmalıdır. `scripts/start_real.sh` bu kontrolleri
kendisi de yapar ve eksik bağımlılıkta hiçbir launch başlatmadan çıkar. Takımın
çalışan sistem Python'una otomatik veya sistem geneli paket kurulmaz; eksik paket
varsa önce ayrı, sürümü kilitli çalışma ortamında hazırlanıp pasif test edilir.

## Hedef rengi kabul testi

1. YKİ’de kırmızı, yeşil veya siyah hedefi kilitleyin.
2. `SEND_TARGET_TO_IDA` sonucunun önce `acked` olduğunu görün.
3. Jetson’da aşağıdakini izleyin:

```bash
ros2 topic echo /mission/target_color --full-length
ros2 topic echo /control/mavsdk_status --full-length
```

4. Arayüzün `state_verified` göstermesini bekleyin.
5. Gönderilen renkten farklı `TGT_ACK`, timeout veya disconnected durumda test
   başarısızdır; göreve başlanmaz.

Beş hedef rengi ayrı ayrı denenmelidir: 1=kırmızı, 2=yeşil, 3=turuncu,
4=siyah, 5=sarı. Bilinmeyen/renksiz lidar engeli hedef kabul edilmez.

## Görev kontrol köprüsü için motorsuz bench kabulü

Bu testte ESC/motor beslemesi kapalı ve `motor_command_enabled:=false` kalır.

1. YKİ'den kısa bir P1/P2 mission yükleyin; upload, paketli `SCR_USER5` metadata
   ve `SCR_USER6` mailbox readback
   sonucunun başarılı olduğunu görün.
2. Jetson'da `/mission/waypoints`, `/mission/start` ve
   `/mission/control_ack` topic'lerini kaydedin.
3. Pixhawk'ta `SCR_USER6` ACK/idle durumunda olmalıdır (ilk değer 8000000).
4. İDA ARM koşulu güvenli, motorsuz bench prosedürüne göre sağlandıktan sonra
   YKİ'de **Görev Başlat** seçin. GUIDED doğrulanmalı, `SCR_USER6` önce
   START_CMD, otonomi ACK'inden sonra START_ACK olmalıdır.
5. YKİ `mission_started=true` gösterirken her zaman erişilebilir ayrı
   **Görev Durdur** düğmesini seçin. Önce
   önce HOLD doğrulanmalı, sonra `SCR_USER6` STOP_CMD ve STOP_ACK durumlarından geçmeli;
   otonomi `MISSION_READY` durumuna dönmelidir.
6. Köprü kapalıyken START deneyinde ACK zaman aşımı görülmeli, görev başlamış
   sayılmamalı ve araç HOLD'da kalmalıdır. Pending START tokenı değiştirilmez.
   Köprü geri gelince aynı START tokenını ACK'ler; araç HOLD'da kalırken operatör
   YKİ'den normal STOP gönderip negatif sonraki tokenı doğrulamalıdır.
7. Aynı tokenı tekrar yazma ve sıra atlayan token testleri yalnız motorsuz
   bench'te yapılır; `/mission/start` üzerinde yeni yayın oluşmamalıdır.

Başarılı PARAM_SET/readback tek başına başarı değildir. Kabul kanıtı YKİ
`state_verified`, SCR_USER6 ACK durumu ve ROS uygulama ACK'inin üçüdür.

## Telemetri çakışmasını önleyen kurallar

- YKİ backend’i `REQUEST_DATA_STREAM` göndermez; link bant genişliği Pixhawk’ın
  mevcut stream-rate ayarlarıyla yönetilir.
- Jetson aynı fiziksel portu iki kez açmaz.
- YKİ durum mesajları yalnız değişimde ve en fazla iki saniyelik heartbeat ile
  tekrarlanır.
- YKİ İDA bağlantısı yalnız beklenen SYS_ID’den gelen alanları işler.
- Canonical `/perception/buoys` ve `/perception/obstacles` topic’lerinin tek
  yayıncısı `ida_sensor_fusion` olmalıdır.
- `acked`, Jetson’un hedefi kullandığı anlamına gelmez; yalnız
  `state_verified` uçtan uca kanıttır.

## Motor yoluna geçiş

`vehicle_setup_enabled` yalnız açıkça seçilen servo/frame parametrelerini yazar;
`guided_mode_enabled` GUIDED velocity paket döngüsüne izin verir, fakat araç
modunu değiştirmez. GUIDED geçişinin tek otoritesi ACK doğrulayan YKİ START
komutudur; bridge açılış/reconnect sırasında HOLD'u bozamaz.
`motor_command_enabled` sınırlanmış hız komutunu Pixhawk’a çıkarabilir. Bu iki
bayrak pervaneler güvenli, fiziksel acil durdurma hazır ve ayrı ARM/motor test
planı tamamlanmış olmadan açılmaz. Telemetri/hedef testi bu bayraklara ihtiyaç
duymaz.
