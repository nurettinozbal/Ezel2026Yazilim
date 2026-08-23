# Saha test planı

## Amaç

Bu plan, tekneyi doğrudan tam otonom göreve çıkarmadan önce sorunları küçük ve
güvenli adımlarda bulmak içindir. Bir aşama geçmeden sonraki aşamaya geçilmez.

## Roller

Her testte en az şu roller belirlenir:

- test sorumlusu: aşamayı ve sonucu kaydeder,
- güvenlik sorumlusu: alanı ve acil durdurmayı izler,
- operatör: manuel kontrolü devralabilir,
- gözlemci: teknenin davranışını ve dubalara mesafesini izler.

## Test öncesi kontrol listesi

- [ ] Hava, su ve görüş koşulları uygundur.
- [ ] Test alanında insan ve başka araç yoktur.
- [ ] Gövde, pervane, batarya ve acil durdurma kontrol edilmiştir.
- [ ] Pixhawk, Jetson, kamera, lidar ve haberleşme bağlantıları sabittir.
- [ ] Sistem saati ve disk boşluğu uygundur.
- [ ] Doğru kod build’i ve doğru model dosyaları kullanılıyor.
- [ ] Model dosyasının SHA256 değeri kayıtlı manifestle aynı.
- [ ] Kamera/lidar yön testi geçmiştir.
- [ ] Manuel devralma denenmiştir.
- [ ] Rosbag ve video kaydı başlamıştır.

## Aşama 0 — masaüstü ve kuru çalışma

Tekne hareket ettirilmez; mümkünse pervaneler bağlı değildir.

1. Launch `dry_run:=true` ile açılır.
2. Tüm gerekli topic’lerin aktığı kontrol edilir.
3. Görev noktaları ve hedef rengi gönderilir.
4. Otonominin doğru durum sırasına geçtiği gözlenir.
5. Üretilen ileri hız ve dönüş komutlarının işareti kontrol edilir.

Geçme şartı: veri kesildiğinde failsafe görülmeli, gerçek motor çıkışı oluşmamalıdır.

## Aşama 1 — sensör testi

Araç sabitken merkez/sağ/sol duba testleri yapılır. Lidar mesafesi metre ile,
kamera sınıfı gözle kontrol edilir. Kamera kapatılarak ve lidar önü kısa süre
kapatılarak bayat veri davranışı denenir.

Geçme şartı: sağ/sol yönü doğru, mesafe makul, bayat veri yeni tespit gibi
sunulmuyor ve bilinmeyen lidar nesnesi engel olarak kalıyor olmalıdır.

Ek kontrol: `/perception/fusion/status` içinde `ready` ve `accepted` doğru olmalı;
canonical buoy ve obstacle topic'lerinde birer yayıncı bulunmalıdır.

## Aşama 2 — bağlı veya emniyetli düşük güç testi

Tekne emniyet halatıyla sınırlandırılır ya da güvenli test düzeneği kullanılır.
Önce yalnız çok küçük ileri komut, sonra sağ ve sol dönüş denenir.

Geçme şartı: pozitif ileri komut her heading’de ileri hareket üretmeli; sağ/sol
dönüş işareti fiziksel hareketle eşleşmelidir.

## Aşama 3 — açık suda düz hat

Engelsiz alanda iki yakın waypoint kullanılır. Önce düşük hızla gidilir. Rota
etrafındaki salınım, waypoint geçişi ve durma davranışı izlenir.

Geçme şartı: uzak waypoint atlanmamalı, araç geri sürüşe geçmemeli, salınım giderek
büyümemeli ve operatör her an devralabilmelidir.

## Aşama 4 — Parkur 1 koridoru

Önce düz bir koridor, sonra tek bir 90 derece dönüş kurulur. Duba aralığı gerçek
yarış ölçüsüne yakın seçilir. Hız düşük tutulur.

Geçme şartı:

- turuncu dubaya temas yok,
- dönüşte iç köşeye yığılma veya koridordan çıkma yok,
- `inside_course_geometry` fiziksel durumla uyumlu,
- kapı sayısı gerçek kapı sayısıyla uyumlu,
- rota salınımı kabul edilen sınır içinde.

## Aşama 5 — Parkur 2 engelden kaçış

Önce tek sarı engel, sonra aralıklı iki engel, en son yarışa benzer sıra denenir.
Her yerleşim ölçülür fakat koordinatlar üretim koduna girilmez.

Geçme şartı: çarpışma olmamalı, bilinmeyen nesne güvenli tarafta kalmalı, araç
gereksiz yere uzun süre recovery durumunda kalmamalı, hedefe ilerlemeye devam
etmeli ve `distance_from_course_m` 5.5 m çıkış eşiğini aşmamalıdır. Son
waypoint aracın arkasına geçerse ileri hızla geniş bir yeniden-yakalama yayı
kabul edilmez.

## Aşama 6 — Parkur 3 hedefi

Kırmızı, yeşil ve siyah hedefler ayrı ayrı ve birlikte denenir. Hedef rengi kara
arayüzünden değiştirilir. Yanlış hedef yakına konularak risk davranışı gözlenir.

Geçme şartı: yalnız doğru hedefe angajman, bayat/boş görüntüde angajman yok ve
yanlış hedefe yönelme yoktur.

Hedef kamera menzilinde fakat lidar menzili dışındaysa sistem temas kararı
vermemelidir. Test düzeni iki sensörün ortak menzilinde kurulmalıdır.

## Aşama 7 — tam görev

Tüm aşamalar tek koşuda birleştirilir. İlk tam koşu düşük hızda yapılır. Başarılı
iki tekrar görülmeden hız artırılmaz.

Geçme şartı:

- görev sırası P1 → P2 → P3 → COMPLETE,
- failsafe nedeni boş veya açıklanmış,
- duba/engel teması yok,
- operatör müdahalesi yok,
- rosbag, video ve test formu eksiksiz.

## Derhal durdurma nedenleri

- telemetri veya manuel kontrol kaybı,
- ters yönde motor komutu,
- büyüyen salınım veya kontrolsüz dönüş,
- lidar yönünün ters olduğunun görülmesi,
- kamera/lidar verisinin bayat olmasına rağmen taze sayılması,
- güvenlik alanına insan veya başka araç girmesi,
- su alma, aşırı ısınma veya güç sorunu.

## Test sonrası kısa rapor

Her koşu için şu beş satır yeterlidir:

1. tarih, yer ve yazılım sürümü,
2. test aşaması ve parkur düzeni,
3. geçti/kaldı sonucu,
4. gözlenen en önemli davranış,
5. rosbag ve video klasörü.
