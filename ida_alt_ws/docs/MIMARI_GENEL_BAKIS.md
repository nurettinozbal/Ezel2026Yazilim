# Mimari genel bakış

## Sistem ne yapıyor?

Sistem kıyıdan verilen görev noktalarını alır, aracın konumunu ve çevresindeki
nesneleri izler, güvenli bir hız/dönüş komutu üretir ve bu komutu Pixhawk’a gönderir.

Basit veri akışı şöyledir:

```text
Kara arayüzü
    │ görev noktaları ve hedef rengi
    ▼
Jetson / ROS 2
    ├─ kamera: renk ve hedef sınıfı
    ├─ lidar: mesafe ve engel şekli
    ├─ otonomi: hangi parkurdayız?
    ├─ planlama: hangi yöne ve ne hızla gidelim?
    └─ güvenlik: komutu sınırla, bayat veride dur
    │
    ▼
Pixhawk / ArduRover
    │ motor komutları ve telemetri
    ▼
Tekne
```

## Ana parçalar

| Parça | Basit görevi |
|---|---|
| `ida_autonomy` | Görevin hangi aşamada olduğunu yönetir. |
| `ida_planning` | Waypoint takibi, yerel engelden kaçış ve güvenli rota seçimini yapar. |
| `ida_control` | Hız komutunu sınırlar ve Pixhawk’a iletir. |
| `ida_perception` | Kamera ve lidar verilerini ortak algı mesajlarına çevirir. |
| `ida_sensor_fusion` | Kameranın rengini lidarın ölçtüğü fiziksel nesneyle güvenli biçimde eşleştirir. |
| `ida_logging` | Telemetri, video ve yerel harita kayıtlarını tutar. |
| `mavlink-routerd` | Jetson’daki Pixhawk UART’ını tek başına açar ve iki yerel UDP uca böler. |
| `ida_control` YKİ köprüsü | Aynı routed MAVLink ucundan düşük hızlı durum alanlarını YKİ’ye yollar. |
| `ida_bringup` | Gerçek araç ve simülasyon bileşenlerini birlikte başlatır. |
| `ida_perception_sim` | Yalnız simülasyonda yapay algı verisi üretir. |

## Harita kullanılıyor mu?

Önceden hazırlanmış duba veya engel haritası kullanılmaz. Sisteme yalnız görev
noktaları verilir. Engeller, araç hareket ederken lidar ve kameradan görülür ve
kısa süreli yerel bir çevre görünümüne işlenir. Bu görünüm aracın yakın çevresi
içindir; yarış parkurunun kalıcı haritası değildir.

Gazebo world dosyası ve simülasyon nesne kimlikleri yalnız test ortamındadır.
Gerçek araç launch’ında simülasyon gerçeği kapalıdır.

## Simülasyon ve gerçek araç farkı

- Simülasyon, fiziksel aracı ve sensörleri kontrollü biçimde taklit eder.
- Gerçek araçta konum Pixhawk’tan, engel mesafesi S2 lidardan, renk bilgisi
  kameradan gelmelidir.
- Simülasyonda kullanılan `gate_truth` yalnız kapı sayma testini kararlı yapmak
  içindir; gerçek araçta açılmaz.
- Aynı karar ve güvenlik katmanı her iki ortamda da kullanılır.

## Kamera-lidar füzyonu

Simülasyonda kamera ve lidar artık ayrı ham mesajlar üretir. Füzyon aynı anda ve
aynı yönde görülen kayıtları bire bir eşleştirir. Eşleşme belirsizse renk atamaz;
lidarın gördüğü nesne yine “bilinmeyen sert engel” olarak kalır. Aynı canonical
topic'i aynı anda yalnız bir düğüm yayınlayabilir.

Bu akış simülasyonda doğrulanmıştır. Gerçek araç launch'ında ise kalibrasyon
tamamlanana kadar varsayılan olarak karar verici değildir. Önce gözlem/shadow
modu, sonra saha kabul testleri kullanılmalıdır.

## YKİ, Pixhawk ve Jetson haberleşmesi

`ida_ws_gateway` canlı mimariden çıkarılmıştır ve colcon tarafından derlenmez.
Pixhawk’ın Jetson USB/UART portunu yalnız `mavlink-routerd` açar. MAVSDK
telemetri/parametre için 14540, pymavlink komut ve YKİ durum mesajları için 14541
ucunu kullanır. Böylece aynı seri portu açmaya çalışan iki ROS düğümü yoktur.

İHA ile İDA doğrudan konuşmaz. İHA hedef rengini kendi RFD hattından YKİ’ye
`TARGET_COL` mesajıyla yollar. YKİ hedefi kilitler ve İDA Pixhawk’ındaki
takım stack'inden ayrı `SCR_USER4` parametresine yazar. Jetson parametreyi okuyup hedefi ROS’a
yayınladıktan sonra `TGT_ACK` gönderir. Arayüz ancak bu son doğrulamada
`state_verified` gösterir.

Görev rotası Pixhawk'a standart NAV_WAYPOINT mission olarak yüklenir, fakat
Pixhawk AUTO rotayı sürmez. Jetson `mission_raw` ile rotayı indirir; YKİ GUIDED
modu doğruladıktan sonra `SCR_USER6` mailbox'a START/STOP komutu yazar. Bridge bu
komutu `/mission/start` üzerinden otonomiye taşır ve ancak otonomi uyguladığını
`/mission/control_ack` ile bildirdiğinde aynı mailbox'ı ACK durumuna çevirir. Böylece YKİ,
Pixhawk ve Jetson uçtan uca aynı görev durumunda olmadan görev başlamış sayılmaz.
Takım kodunun `SCR_USER1/2/3` tahsisi korunur.
