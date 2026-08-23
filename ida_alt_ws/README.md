# İDA otonomi yazılımı

Bu klasör, TEKNOFEST İnsansız Deniz Aracı için ROS 2 tabanlı otonomi yazılımını,
simülasyon araçlarını ve kara arayüzünü içerir.

## Güncel durum

- Onaylı çalışma kopyası sunucuda `/home/ezel/ida_ws` altındadır.
- Temel otonomi Parkur 1, Parkur 2 ve Parkur 3 görevlerini simülasyonda
  tamamlamıştır. Yeni kamera-lidar füzyonu Parkur 1/2 tam koşusunda ve Parkur 3
  odaklı koşusunda ayrıca doğrulanmıştır.
- Parkur geometrisi, uzak waypoint atlama, ters sürüş, kapı sayımı ve koridor durumu
  için bulunan kritik hatalar düzeltilmiştir.
- Gerçek araçta kamera bağlantısı ve kamera–lidar kalibrasyonu tamamlanmadan tam
  otonom sürüşe geçilmemelidir.
- Önceden hazırlanmış engel haritası kullanılmaz. Araç yalnız görev noktaları ve
  o anda sensörlerden gelen çevre bilgisiyle karar verir.

## Önce hangi belge okunmalı?

1. [Mimari genel bakış](docs/MIMARI_GENEL_BAKIS.md)
2. [Otonomi görev akışı](docs/OTONOMI_STACK_REHBERI.md)
3. [Bench test ana planı](docs/BENCH_TEST_ANA_PLANI.md)
4. [Gerçek araç ve sensör kalibrasyonu](docs/GERCEK_ARAC_VE_SENSOR_KALIBRASYONU.md)
5. [Saha test planı](docs/SAHA_TEST_PLANI.md)
6. [Çalıştırma ve kayıt alma](docs/CALISTIRMA_VE_KAYIT.md)
7. [Kamera modelleri ve TensorRT](docs/KAMERA_MODELLERI_VE_TENSORRT.md)
8. [Jetson–YKİ pasif test konsolu](docs/JETSON_YKI_PASIF_TEST_KONSOLU.md)
9. [Kalan iş paketleri](docs/KALAN_IS_PAKETLERI_2026-08-14.md)
10. [Jetson'da mevcut sistemi bozmadan alternatif test](docs/JETSON_ALTERNATIF_TEST_PLANI.md)
11. [YKİ–Pixhawk–Jetson haberleşmesi](docs/YKI_PIXHAWK_JETSON_HABERLESME.md)

Daha ayrıntılı mühendislik çalışmaları için
[optimizasyon planına](OPTIMIZASYON_PLANI_2026-08-14.md) bakılabilir.

## Ana klasörler

- `src/`: ROS 2 otonomi paketleri
- `scripts/`: Jetson üzerinde kullanılan yardımcı başlatma scriptleri
- `tools/`: parkur üretme ve doğrulama araçları
- `debug_logs/`: simülasyon ve rosbag kayıtları
- `ezel-yazilim/`: kara arayüzü

## Güvenlik notu

Parola, özel anahtar, cihaz seri numarası veya erişim bilgisi Markdown dosyalarına
yazılmamalıdır. Gerçek motor çıkışını açmadan önce kuru çalışma ve bağlı olmayan
pervane testi yapılmalıdır.
