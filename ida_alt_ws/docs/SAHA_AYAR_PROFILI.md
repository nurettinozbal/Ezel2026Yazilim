# Saha ayar profili

Gerçek araçta davranış ayarlarının tek kaynağı:

`src/ida_bringup/config/field_profile.yaml`

Saha günü hız, dönüş, kaçınma veya hedef kilidi için başka Python, launch,
servis ya da env dosyası değiştirilmez. `ida_cli start`, bu profili otonomiye,
komut sınırlayıcıya, GUIDED köprüsüne, kameraya, lidara ve sensör füzyonuna
dağıtır. Değişiklikten sonra workspace yeniden derlenmeli ve servis yeniden
başlatılmalıdır.

## Taşınabilir kurulum sözleşmesi

- Workspace yolu, servis kullanıcısı ve grubu kurulum anında otomatik türetilir.
- Pixhawk, lidar ve Arducam numaralı `/dev/tty*` veya `/dev/video*` adına
  bağlanmaz; kalıcı cihaz kimliğiyle keşfedilir. Belirsizlikte stack açılmaz.
- YKİ adresi sabit IP değildir; `auto://yki` yerel ağ keşfi kullanılır.
- Windows/Linux YKİ başlatıcıları tek RFD portunu keşfeder; sıfır veya birden
  fazla adayda araç komut yolunu açmadan salt-okunur başlar.
- General YOLO modeli repo içindeki sürümlü konumdadır ve başlangıçta SHA-256
  manifestiyle doğrulanır.
- Kullanıcının elle hazırlaması gereken gizli bir `.env` yoktur. Kurucu,
  kaynak kontrollü şablonları gerçek workspace yoluyla üretir ve pasif YKİ
  eşleme anahtarını paketli kontrattan kurar.

Donanıma özgü ama kod dışı zorunluluklar yine vardır: Linux cihaz erişim izni,
ROS 2 Humble, derlenmiş workspace ve Slamtec sürücü paketi. Bunlar bir algoritma
ayarının başka bilgisayarda değişmesi değil, çalışma zamanı ön koşullarıdır.

## Parkur 1 — waypoint takibi

- `ida_field_envelope.hard_max_speed_mps`: Uygulama katmanındaki değiştirilemez
  saha tavanı: **1,60 m/s**. Daha yüksek değer launch sırasında reddedilir.
- `max_speed_mps`: Kullanılan ileri hız tavanı. Limiter ve GUIDED köprüsü de
  aynı değeri alır; hard tavanı aşamaz.
- `max_yaw_rate_deg_s`: Genel dönüş tavanı.
- `dwa_align_before_drive_deg`: Hedef açısı bundan büyükse araç geniş yay
  çizmez; önce olduğu yerde hedefe hizalanır.
- `dwa_align_yaw_rate_deg_s`: Bu ilk hizalanmanın dönüş hızı.
- `dwa_w_heading`: Waypoint yönüne dönme isteği.
- `dwa_w_corridor`: Parkur 1 koridor merkezinde kalma isteği.

YKİ'deki HOME yalnız dönüş/RTL işaretidir. Mission waypoint listesine otomatik
eklenmez; haritadaki ilk tıklama doğrudan **İlk Hedef #1** olur.

## Parkur 2 — engelden kaçınma

Fiziksel engelin konumu lidar kaynaklıdır. Kamera yalnız renk etiketi ekler.
Kamera bir dubanın rengini okuyamasa dahi lidar kümesi `unknown/hard obstacle`
olarak kalır ve DWA çarpışma kontrolüne girer. Lidar akışı kesilirse P1/P2
hareketi fail-safe durur.

- `ida_field_envelope.min_navigation_command_mps`: Normal hareket isteyen
  planlayıcı yollarının komut tabanı: **0,70 m/s**.
- `dwa_min_drive_vx`: Yukarıdaki komut tabanıyla aynı olmak zorundadır.
- `dwa_slow_vx`: Kaçınırken kullanılan yavaş hız tavanı.
- `dwa_min_turn_rate_deg_s`: Etkisiz küçük dönüşleri engelleyen alt sınır.
- `dwa_w_obstacle`: Engel açıklığı puanı.
- `dwa_w_avoid`: Renkli engelden uzaklaşma puanı.
- `costmap_bot_radius_m`: Araç gövdesinin planlamadaki yarıçapı.
- `costmap_safety_m`: Gövde dışındaki ek güvenlik payı.

Lidar sürücüsü ve köprüsü launch içinde otomatik yeniden başlar. İkisinden biri
görünmüyorsa `ida_cli start` başarılı mesajı vermez ve stack'i durdurur.

## Parkur 3 — hedef arama ve angajman

- `target_min_confidence`: Hedef sayılacak en düşük model güveni.
- `p3_search_yaw_rate_deg_s`: Hedef yokken kesintisiz 360 derece arama hızı.
- `p3_lock_confirm_s`: Hedefin en az görünme süresi.
- `p3_lock_confirm_frames`: Süre içinde gereken bağımsız kamera karesi.
- `p3_target_loss_grace_s`: Kısa görüntü kaybında aynı lidar track kimliğini
  koruma süresi; bu sürede ileri itki verilmez.
- `p3_lock_yaw_gain_deg_s`: Hedefi görüntü merkezine alma kuvveti.
- `p3_engage_center_max`: Temas sekansı için yatay merkez toleransı.
- `p3_engage_distance_m`: Lidar mesafe eşiği.
- Temas yakınlığı yalnız fusion/lidar `distance` ölçümünden doğrulanır; kamera
  kutu büyüklüğü tek başına fiziksel temas izni vermez.
- `p3_lock_min_speed_mps`: Hedef takibinde hareket isteyen en düşük komut.
- `p3_engage_speed_mps`: Doğrulanmış hedefe temas penceresinin ileri komutu.
- `engage_window_s`: Doğrulanmış hedefe son temas komutu süresi.

Varsayılan saha profili yüzde 20 güven eşiğini kullanır fakat hedefi tek karede
kabul etmez: güven eşiğini geçen hedef en az 0,4 saniye ve en az 3 bağımsız
kare görülmelidir. Arama 30 derece/s ile yaklaşık 12 saniyelik tam turdur.
Doğrulamadan ve yatay merkezleme tamamlanmadan ileri itki sıfırdır. Yaklaşmada
yaw düzeltmesi sürekli sürer. Temas sırasında hedef kaybolur veya merkezden
çıkarsa aynı kontrol tick'inde ileri itki kesilir ve yeniden hizalama başlar.

## Motor yüzdesi ile hız komutu aynı şey değildir

YKİ motor yüzdesini gerçek `SERVO_OUTPUT_RAW` mesajından gösterir. Arayüzdeki
hesapta 1500 PWM nötr, 1600 PWM yaklaşık +%20'dir. Sahadaki güncel kanıt:

- +%10 üstünde motor açık havada dönebilir,
- 1600 PWM bench'te motor çıkışı üretmiştir,
- bunların hiçbiri su yükünde yeterli itki kanıtı değildir.

Jetson, GUIDED yolunda Pixhawk'a PWM yüzdesi değil hedef hız (m/s) gönderir.
Pixhawk hedef hızı `CRUISE_SPEED`, `CRUISE_THROTTLE` ve `ATC_SPEED_*` hız
kontrolcüsüyle motor çıkışına çevirir. Bu nedenle yüzdeyi m/s'ye doğrusal
çeviren bir katsayı koda eklenmemiştir.

Uygulamanın kesin üst sınırı 1,60 m/s'dir. Elektriksel/mekanik motor üst sınırı
ise Pixhawk `MOT_THR_MAX` parametresidir; ESC, motor, batarya akımı ve sıcaklık
ölçümü olmadan bu değer kaynak kod tarafından otomatik yazılmaz. Denizde sabit
hız ölçümü yapıldıktan sonra `CRUISE_SPEED` ile o hızı sürdüren gerçek
`CRUISE_THROTTLE` eşleştirilmelidir. `MOT_THR_MAX` ayrıca donanımın doğrulanmış
akım ve sıcaklık sınırına göre belirlenmelidir.

## Tak-çalıştır YKİ ve lidar görünümü

YKİ bilgisayarında yalnız proje kökündeki `start.bat` (Windows) veya `start.sh`
(Linux) çalıştırılır. Script gerekli bağımlılıkları, ortak browser tokenını ve
ayrı salt-okunur Jetson eşleme anahtarını kendi hazırlar.

Jetson tarafında `ida_cli start`:

1. Canonical stack'i açar fakat ARM/START göndermez.
2. Lidar sürücüsü ve köprüsünü doğrular.
3. Salt-okunur YKİ algı servisini açar.
4. YKİ bilgisayarını sabit IP olmadan yerel ağ yayınıyla bulur.

Bu otomatik eşleme yalnız lidar/füzyon/otonomi teşhis verisini taşır; motor veya
Pixhawk komut yetkisi içermez. Windows Güvenlik Duvarı sorarsa özel ağda backend
ve UDP yerel ağ keşfine izin verilmelidir.

## Değişiklik sonrası kısa kontrol

1. Workspace'i derle ve source et.
2. `ida_cli unit` çalıştır.
3. `ida_cli start` sonrasında `ida_cli status` içinde lidarın iki node'unu ve
   pasif YKİ servisini gör.
4. YKİ lidar panelinde veri yaşı sürekli güncellenmeden ARM/START verme.
5. İlk saha turunu düşük riskli iki gerçek waypoint ile yap; HOME'u missiona
   waypoint olarak ekleme.
