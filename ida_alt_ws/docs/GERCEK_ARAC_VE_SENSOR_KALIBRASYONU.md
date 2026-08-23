# Gerçek araç ve sensör kalibrasyonu

Bu işlem tek seferlik bir “ayar” değildir. Kamera veya lidarın yeri değişirse ilgili
adımlar yeniden yapılmalıdır.

## 1. Mekanik hazırlık

- Kamera ve lidar oynamayacak şekilde sabitlenir.
- Her sensörün teknenin merkezine göre ileri/geri ve sağ/sol mesafesi ölçülür.
- Sensörlerin baktığı yön teknenin düz ileri yönüyle karşılaştırılır.
- Kablo, USB ve güç bağlantıları titreşimde çıkmayacak şekilde sabitlenir.
- Pervaneler güvenli hale getirilmeden motorlu test yapılmaz.

Ölçümler tarih ve fotoğrafla kayıt altına alınmalıdır.

## 2. Cihaz adlarını sabitleme

Jetson yeniden başladığında `/dev/ttyUSB0` başka bir cihaza geçebilir. Pixhawk,
lidar ve diğer seri cihazlar için kalıcı udev adı kullanılmalıdır; örneğin
`/dev/pixhawk` ve `/dev/lidar`.

Kontrol:

```bash
ls -l /dev/pixhawk /dev/lidar
```

## 3. S2 lidar kontrolü

Gerçek launch, `sllidar_ros2` S2 sürücüsünü başlatır ve `/scan` yayınını önce
`/perception/lidar/raw_obstacles` ham füzyon mesajına dönüştürür. Canonical
`/perception/obstacles` topic'inin tek yayıncısı `ida_sensor_fusion` olur.

Önce yalnız lidar açılır ve şu kontroller yapılır:

```bash
ros2 topic hz /scan
ros2 topic echo --once /scan
ros2 topic echo --once /perception/lidar/raw_obstacles --full-length
ros2 topic echo --once /perception/fusion/status --full-length
```

Bir kişi veya duba sırayla tam öne, sağa ve sola konur. Arayüzdeki sağ/sol işareti
fiziksel yönle aynı olmalıdır. ROS LaserScan açı yönü ile yazılımın “sağ pozitif”
kabulü farklı olabileceği için bu adım atlanmamalıdır. İşaret tersse planlama testi
yapılmadan açı yönü/offset ayarı düzeltilmelidir.

## 4. Kamera bağlantısı

Mevcut YOLO düğümü `CompressedImage` bekler. Gerçek kamera sürücüsü launch dışında
başlatılmalı ve sıkıştırılmış bir görüntü topic’i yayınlamalıdır. Ardından
`src/ida_bringup/config/perception.yaml` içindeki iki `camera_topic` alanı gerçek
topic adıyla eşleştirilmelidir.

Kontrol:

```bash
ros2 topic list -t
ros2 topic hz /camera/image_raw/compressed
ros2 topic echo --once /camera/camera_info
```

Kamera topic’i boş bırakılırsa YOLO düğümü çalışır görünür fakat görüntü alamaz.

## 5. Kamera iç kalibrasyonu

Kamera çözünürlüğü sabitlenir. Dama tahtasıyla farklı uzaklık ve açılardan yeterli
görüntü alınır. Elde edilen odak, görüntü merkezi ve bozulma değerleri
`CameraInfo` olarak yayınlanır.

Kalibrasyon sonrasında görüntünün merkezindeki bir hedefin açısı yaklaşık sıfır,
sağdaki hedefin işareti sağ, soldakinin işareti sol olmalıdır.

## 6. Kamera–lidar birlikte kalibrasyonu

Tek bir turuncu duba şu konumlarda sırayla kaydedilir:

- 2 m, 5 m ve 10 m tam merkez,
- 2–3 m sağ,
- 2–3 m sol.

Her konumda araç sabit tutulur ve en az 10 saniye veri alınır. Kamera kutusunun
yatay açısı ile lidar kümesinin açısı aynı nesneyi göstermelidir. Ayrıca kamera ve
lidar mesaj zamanları aynı ROS saatini kullanmalıdır.

Önerilen hedefler:

- tipik zaman farkı 50 ms veya daha az,
- 100 ms üzerindeki eşleşmeleri kullanmama,
- aynı lidar kümesini iki kamera tespitine vermeme,
- belirsiz eşleşmede renk atamama,
- eşleşmeyen lidar nesnesini yine sert/bilinmeyen engel sayma.

## 7. Füzyonu devreye alma sırası

1. Önce füzyon yalnız kayıt ve karşılaştırma modunda çalışır.
2. Mevcut güvenli lidar engel akışı karar vermeye devam eder.
3. Kayıtlarda sağ/sol, mesafe, zaman ve renk eşleşmesi doğrulanır.
4. Kamera kesildiğinde veya görüntü bayatladığında davranış test edilir.
5. Ancak kabul kriterleri geçince kamera rengi otonomi kararına bağlanır.

`ida_sensor_fusion` simülasyonda canonical akışta doğrulanmıştır. Bu sonuç gerçek
araç kalibrasyonunun yerine geçmez. Gerçek launch'ta ancak aşağıdaki durumların
tamamı görüldükten sonra karar verici yapılmalıdır:

- status mesajında `ready=true`, `accepted=true`,
- kamera ve lidar verisi taze,
- sağ/sol işareti fiziksel yerleşimle aynı,
- yanlış veya belirsiz renklendirme yok,
- kamera kesilince renk kayboluyor fakat lidar engeli kalıyor,
- canonical `/perception/buoys` ve `/perception/obstacles` için yayıncı sayısı 1.

## 8. Canlı stack sahiplik kapıları

`real_vehicle.launch.py` güvenli varsayılanla hiçbir canonical düğüm başlatmaz.
Takımın çalışan kodu açıkken yalnız `vehicle_test_lab.launch.py` veya izole replay
kullanılır. Tam geçişte aşağıdaki dört izin birbirinden bağımsızdır:

- `canonical_takeover_enabled`: canonical ROS topic'lerinin sahibi olma,
- `mavlink_router_enabled`: Pixhawk Jetson seri portunun sahibi olma,
- `vehicle_setup_enabled`: yalnız açıkça seçilen servo/frame parametre yazımı,
- `guided_mode_enabled`: GUIDED velocity paket yoluna izin (modu değiştirmez;
  GUIDED geçişinin tek otoritesi YKİ START handshake'idir),
- `motor_command_enabled`: sınırlanmış motor hız komutunu MAVLink'e çıkarma.

İlk gerçek bağlantı testi yalnız ilk iki izinle, pervaneler güvenli durumdayken
yapılır; son iki izin ARM/motor test planındaki fiziksel kontrollerden önce açılmaz.

## 9. Kalibrasyon kaydı

Her kalibrasyon için şu bilgiler saklanır:

- tarih, araç ve sensör yerleşimi fotoğrafı,
- kamera çözünürlüğü ve model dosyası,
- lidar portu, modu ve açı offset’i,
- kullanılan ROS topic adları,
- JetPack, Ultralytics, Torch ve TensorRT sürümleri,
- model SHA256 değeri ve `.pt`/`.engine` türü,
- rosbag klasörü,
- testi yapan kişi ve sonuç.
