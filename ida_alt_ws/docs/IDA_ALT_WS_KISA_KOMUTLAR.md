# IDA alternatif stack kısa komutları

Jetson terminalinde uzun `source` ve `ros2 launch` dizileri yerine tek komut
kullanılır. Bu komut takımın mevcut `idaws` servisini değiştirmez.

```bash
ida_alt_ws start
ida_alt_ws status
ida_alt_ws logs
ida_alt_ws stop
```

`start` canlı canonical stack'i açar. Pixhawk, lidar, kamera, general YOLO,
sensör füzyonu, otonomi ve kayıt birlikte başlar. Komut kendi başına ARM,
GUIDED, görev yükleme veya hareket göndermez. Bunlar YKİ'den yapılır.

Araçlar:

```bash
ida_alt_ws cam
ida_alt_ws map
ida_alt_ws fusion
ida_alt_ws gps status
ida_alt_ws unit
```

- `cam`: Stack kapalıyken Arducam ayar penceresini açar. Beş sınıflı general
  model kullanılır. `1` turuncu/sarı, `3` kırmızı/yeşil/siyah görünümüdür.
- `map`: Çalışan canonical fusion çıktısını 8091 portunda gösterir.
- `fusion`: Canonical fusion sağlık özetini bir kez yazdırır.
- `gps`: Kapalı alan için geri alınabilir fake GPS yönetimidir. Sıra:
  `prepare`, `start LAT LON [ALT_M]`, `status`, `stop`, `restore`.
  Araç zaten GPS_INPUT modundaysa doğrulanmış eski yedek açıkça
  `gps adopt BACKUP_JSON` ile bağlanabilir.
  `prepare/restore` yalnız araç disarm iken çalışır ve Pixhawk'ı yeniden başlatır.
  Fake GPS bootta otomatik başlamaz; ARM, mod veya motor komutu göndermez.
- `unit`: Kısa regresyon paketidir; normal saha başlangıcında tekrar edilmez.

YKİ normalde arayüz bilgisayarında çalıştırılır. Jetson'a YKİ bağımlılıkları
ayrıca kurulmuşsa `ida_alt_ws yki` kullanılabilir; eksikse komut indirme
başlatmadan hemen açıklayıcı hata verir.

Çakışmayı önlemek için canonical stack açıkken `cam` başlamaz. `start` da
`idaws.service` aktifse başlamayı reddeder.
