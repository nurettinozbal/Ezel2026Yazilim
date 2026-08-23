# Otonomi görev akışı

## Durum makinesi

Otonomi tek seferde her şeyi yapmaya çalışmaz. Görevi sıralı aşamalara böler:

```text
Başlatma
  → görev bekleme
  → görev hazır
  → Parkur 1
  → Parkur 2
  → Parkur 3
  → görev tamam
```

Gerekli bir veri uzun süre gelmezse sistem `FAILSAFE` durumuna geçer ve güvenli
komut üretir. Veri geri geldiğinde yalnız güvenli şartlar sağlanıyorsa devam eder.

## Parkur 1

Araç verilen waypoint sırasını takip eder. Turuncu dubalar koridor sınırı ve sert
engel olarak değerlendirilir. Araç gerçek rota çizgisine göre koridorun içinde mi
dışında mı olduğunu ayrıca izler.

Waypoint yalnız yeterince yaklaşıldığında veya çok küçük bir kaçırma bandında
geride kaldığında geçilmiş sayılır. Uzak bir waypoint, araç yönü değişti diye
atlanmaz.

## Parkur 2

Waypoint yönü korunurken lidarın gördüğü engeller yerel maliyet haritasına eklenir.
Planlayıcı kısa bir gelecek için farklı hız ve dönüş seçeneklerini dener; çarpışma
riski olmayan ve hedefe ilerleyen seçeneği kullanır.

Bu yaklaşım önceden hazırlanmış engel haritası gerektirmez. Yalnız o anda görülen
yakın çevre kullanılır.

## Parkur 3

Kara arayüzünden gelen hedef rengi kullanılır. Kamera aday hedefleri bulur; sistem
doğru renge yönelir, yanlış renkteki hedefleri risk olarak değerlendirir. Görüntü
ve hedef bilgisi bayatsa angajman yapılmaz.

## Komutun araca ulaşması

1. Otonomi ileri hız ve dönüş isteği üretir.
2. Komut sınırlayıcı ani veya fiziksel olarak uygunsuz değerleri sınırlar.
3. MAVLink köprüsü komutu ArduRover’a “ileri hız + dönüş hızı” olarak gönderir.
4. Pixhawk motor çıkışını uygular ve telemetriyi geri yollar.

## Güvenlik katmanları

- Telemetri, algı veya görev verisi bayatsa hareket sınırlandırılır ya da durdurulur.
- Turuncu duba koruması Parkur 1 başladıktan sonra görev boyunca çözülmez.
- Aynı görev mesajının tekrar gelmesi görevi baştan başlatmaz.
- Aynı kapı tekrar görülürse sabit kimlikli sim verisinde iki kez sayılmaz.
- Gerçek araçta simülasyon gerçeği ve world bilgisi kullanılmaz.
- Komut çıkışı önce kuru çalışma modunda doğrulanır.

## Optimizasyon nasıl yapılmalı?

Önce güvenlik korunur, sonra sürüş yumuşatılır. Önerilen sıra:

1. zaman adımını ve komut sıklığını kararlı hale getirmek,
2. waypoint hedefini daha yumuşak bir bakış noktasıyla takip etmek,
3. salınım ölçümlerini düşürmek,
4. Parkur 2’de gereksiz recovery kullanımını azaltmak,
5. en son hız ve maliyet ağırlıklarını ayarlamak.

Ayrıntılı ölçümler `OPTIMIZASYON_PLANI_2026-08-14.md` dosyasındadır.
