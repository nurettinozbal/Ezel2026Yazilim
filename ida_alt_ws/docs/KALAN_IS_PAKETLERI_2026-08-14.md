# Kalan iş paketleri — 14 Ağustos 2026

Bu belge internet/sunucu erişimi kesildikten sonra kaldığımız yeri kaybetmemek
için hazırlandı. Aşağıdaki paketler birbirinden bağımsız ele alınmalıdır.

## Paket 1 — Parkur 2 rota optimizasyonu

Durum: açık ve öncelikli.

Canonical fusion rosbag kaydı, aracın Parkur 2'de 5.5 m çıkış eşiğinin
üzerinde 162.2 saniye kaldığını ve en fazla 13.355 m saptığını gösterdi.
Fusion o anda sağlıklıydı; son waypoint aracın arkasında/yanda kalmış,
DWA ileri hızla geniş bir yeniden-yakalama yayı çizmişti.

Yapılacaklar:

1. Bu rosbag için otomatik `max distance_from_course_m <= 5.5` regresyonu ekle.
2. P2'de waypoint geride kaldığında ileri hızı sürdüren geniş yayı engelle.
3. Segment üzerine sınırlı yeniden-bağlanma hedefi veya kontrollü düşük-hız
   politikası tasarla.
4. Duba teması, 8/8 kapı ve 4/4 sarı başarısını koru.
5. Aynı seed ve en az üç farklı seed ile tam P2 koşusu yap.

Kanıt: `debug_logs/server_fusion_20260814/P2_ROTA_SAPMASI.md`.

## Paket 2 — Sunucu regresyonu ve kalıcıya alma

Durum: sunucu erişimi geri gelince yapılacak.

Yerelde logging node'larının aynı `run_name` klasörünü paylaşması sağlandı.
Önceden farklı saniyelerde farklı `run_*` klasörleri açıldığı için status node
üç zorunlu dosyayı birlikte göremiyordu.

Sunucuda yapılacaklar:

1. Yerel kaynakları geçici overlay'e kopyala ve `colcon build --symlink-install`.
2. `ida_logging`, `ida_vehicle_test`, `ida_control`, bringup launch ve backend
   regresyonlarını çalıştır.
3. Sim launch'ta telemetry/map/video/status node'larının aynı mutlak run
   klasörünü kullandığını doğrula.
4. Sentetik sim processed video üretmiyorsa logging testinin `video evidence
   missing` nedeniyle FAIL vermesini kabul et; sahte video/PASS üretme.
5. Gerçek kamera akışında üç dosyanın da büyüdüğünü gör ve logging PASS al.
6. Tüm testler geçince `/home/ezel/ida_ws` için tarihli yedek al, sonra
   doğrulanmış kaynakları kalıcıya taşı ve yeniden build/smoke yap.

Sunucudan alınan mevcut pasif kanıtlar
`debug_logs/server_vehicle_test_20260814/evidence` altındadır. Comms,
telemetry, P1/P2 camera, lidar, P3 negative, fusion shadow ve autonomy shadow
PASS artifact'ları yerelde saklanmıştır.

## Paket 3 — Gerçek Jetson ve 5 L PET şişe kabulü

Durum: araç sabit, pervane/motor fiziksel olarak güvensiz hale getirilmişken
uygulanabilir.

Sıra:

1. YKİ backend ve Jetson producer bağlantısı; `comms` ve `telemetry`.
2. Turuncu/sarı mat boyalı 5 L PET şişeyi merkezde 2/5/10 m, sonra sağ/sol
   yaklaşık 20 derecede kamera testi.
3. Aynı yerleşimlerde lidar menzil/açı testi.
4. Kamera-lidar fusion: tek nesne, iki nesne, ters renk yerleşimi, belirsiz
   üst üste açı, lidar-only, camera-only ve bayat veri.
5. Araç sabitken `autonomy_shadow`; yalnız karar/komut okunur, aktüatöre
   iletilmez.
6. Logging kanıtı ve artifact SHA kontrolü.

PET şişe bir laboratuvar vekilidir; gerçek duba performansını kanıtlamaz.
Yanlış yüksek-güvenli renk eşleşmesi kabul edilmez. Belirsiz eşleşmede lidar
nesnesi renk almadan hard-unknown engel olarak kalmalıdır.

## Paket 4 — ARM ve motor testi (kilitli)

Durum: yazılım arayüzünden açılmayacak.

Aşağıdakiler kurulmadan YKİ test sayfası ARM veya motor komutu veremez:

- tahrik gücünü fiziksel kesen mantar acil durdurma,
- yerel anahtarlı TEST_ENABLE,
- basılı tutulması gereken dead-man,
- iki farklı yetkili kişi/onay oturumu,
- tek kullanımlı süreli permit ve araç tarafında kısa motor lease'i,
- taze telemetriyle ARM/durma durum doğrulaması,
- pervane sökülü ve sonra bağlı/düşük güç test düzeneği.

Mevcut `ida_vehicle_test` ve YKİ Mühendislik sayfası bilinçli olarak pasiftir;
`actuation_enabled=false` kanıtı her artifact'ta bulunur.
