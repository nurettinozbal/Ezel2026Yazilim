# Jetson Aday Stack Durumu — 15 Ağustos 2026

Bu belge, takımın çalışan stack'ine dokunmadan hazırlanan canonical alternatifin
durumunu kaydeder. Bu hazırlık sırasında ROS node'u/launch çalıştırılmadı; ARM,
mod, parametre veya motor komutu gönderilmedi.

## Güvenli ayrım

- Takım stack'i: `/home/ezelproject/idaws_ws`
- Daha önceki workspace: `/home/ezelproject/ida_ws`
- Bizim ayrı adayımız: `/home/ezelproject/ida_alt_ws`
- `idaws.service`: `disabled` ve `inactive`
- Aday için systemd servisi oluşturulmadı.
- Pixhawk portu: `/dev/idaws_pixhawk -> /dev/ttyACM0`
- LiDAR portu: `/dev/idaws_lidar -> /dev/ttyUSB0`

Takım stack'i `SCR_USER1/2/3`; canonical aday ise `SCR_USER4` hedef renk,
`SCR_USER5` paketlenmiş P1/P2 sayıları ve `SCR_USER6` mission command/ACK
mailbox kullanır. ArduPilot standart olarak yalnız SCR_USER1..6 sağladığı için
7/8 kullanılmaz. 15 Ağustos'taki salt-okunur kontrolde altı değer sırasıyla
`0,2,1,0,0,0` idi; mailbox ilk canonical mission upload sırasında güvenli idle
değerine alınır.

## Motor ve araç güvenlik varsayılanları

- Gerçek motor kanalları: sol `SERVO9`, sağ `SERVO11`.
- `canonical_takeover=false`
- `vehicle_setup=false`
- `guided_mode=false`
- `motor_command=false`

Bu dört kapı nedeniyle örnek ortam dosyasını source etmek tek başına aracı ele
almaz veya hareket yolu açmaz. Ortam örneği:
`scripts/bench_candidate.env.example`.

## Doğrulanan modeller

- P1/P2: `/home/ezelproject/teknofest_2025_kodlar/parkur_2.pt`
  - SHA-256: `ea063e2b96f5f1e8c8d5049e0cbdf79b1edc58cd021e15ec0346b04e79b56961`
- P3: `/home/ezelproject/teknofest_2025_kodlar/parkur_3.pt`
  - SHA-256: `8fd27648d41f9ed365706f53883d7c517c0ab33184f8cf5d37303500f6a58313`

`/home/ezelproject/idaws_ws/recordings/Downloads/parkur3_best.pt` yalnız
491520 bayttır ve adayda kullanılmamalıdır.

## Tamamlanan pasif doğrulamalar

- 11 ROS paketi `colcon build --symlink-install` ile başarıyla derlendi.
- Kontrol paketinin yerel 52 testi geçti.
- Jetson'da planlama 159, füzyon 35, algı 54, araç test 55 dahil görülen
  paket testleri geçti. YKİ kontrat dosyaları ilk arşivde bulunmadığı için iki
  araç-test kontrolü önce dosya-yok hatası verdi; dosyalar aday workspace'e
  eklendikten sonra paket 55/55 geçti.
- `real_vehicle.launch.py --show-args` yalnız ayrıştırıldı; launch edilmedi.
- Testlerin ardından `idaws.service` yine kapalı, seri port boştu.

## Canlı launch öncesi kalan bağımlılıklar

Jetson'da aşağıdakiler eksik bulundu ve takım sistemini bozmamak için kurulmadı:

- `mavlink-routerd`
- Python `websockets` paketi (YKI test producer için)

Bunlar exact Jetson/JetPack ortamında kontrollü ve geri alınabilir biçimde
kurulup yeniden unit test edilmeden canonical canlı launch yapılmamalıdır.

## Sonraki güvenli sıra

1. Motor/ESC gücü kapalıyken eksik bağımlılıkları izole biçimde kur.
2. `idaws.service` kapalı ve `/dev/ttyACM0` boş kontrolünü tekrarla.
3. Önce yalnız pasif vehicle-test/algı gözlem yolunu çalıştır ve logla.
4. Sensör freshness, tek canonical publisher ve YKİ bağlantısını doğrula.
5. Ancak fiziksel güvenlik kontrolü ve açık kullanıcı onayıyla GUIDED/motor
   kapılarını ayrı ayrı değerlendir.

Takım stack'ini geri almak için bu aday klasörü source edilmeden bırakılır;
`idaws.service` ise yalnız açık ekip kararıyla yeniden enable/start edilir.

## Aynı gün bench sonucundan kalıcılaştırılanlar

- S2 ekseni fiziksel sağ/sol şişe testiyle doğrulandı; kurulu araç profili
  `angle_offset_deg=180`, `mirror_scan=true`, ileri ofset `0.57 m` kullanır.
- İlk canlı GUIDED denemesinde sanal GPS süresi dolunca Pixhawk `EKF variance`
  failsafe ile GUIDED'dan HOLD'a geçti. Sonraki denemede sürekli GPS ve EKF sağlık
  kanıtıyla GUIDED kararlı kaldı. Bu nedenle bridge artık taze global/local
  position health ve gerçek GUIDED modu olmadan yalnız nötr komut üretir.
- Aktifken HOLD, EKF, konum veya link kaybı bir arm döngüsü boyunca kilitlenir;
  otomatik GUIDED geri alma yoktur, DISARM görülmeden tekrar hareket açılamaz.
- Sabit bench'te recovery dönüşünün servo çıkışını gereğinden fazla büyüttüğü
  gözlendi. Bench wrapper artık bridge seviyesinde ileri `0.25 m/s`, geri
  `0.0 m/s`, yaw `0.10 rad/s` zarfı uygular. Normal saha launch varsayılanları
  ayrı kalır.
- Bu değişiklikler yalnız yerel aday workspace'tedir. Jetson/Pixhawk'a otomatik
  deploy edilmedi; takım arkadaşının çalışan stack'i veya parametreleri
  değiştirilmedi.
