# Sarı Duba Sensör Füzyonu ve Motor Yönü Bench Testi

Bu profil, araç hareket edemeyecek biçimde sabitlenmişken sarı dubanın sağ/sol
algılanmasını, otonomi dönüş kararını ve Pixhawk motor mixer yönünü ayrı ayrı
doğrulamak içindir. Gerçek kaçınma mesafesini kanıtlamaz; bunun için saha koşusu
gerekir.

## Yazılım sınırları

- Normal `autonomy.yaml` değiştirilmez.
- Bench hızı varsayılan `0.25 m/s`, üst sınırı `0.30 m/s` olur.
- `stuck_timeout_s` yalnız bu profilde `30 s` olur; saha varsayılanı `3 s` kalır.
- Launch varsayılan olarak `dry_run=true`, takeover/router/GUIDED/motor kapalıdır.
- Launch veya script kendi başına ARM ya da mission START göndermez.
- Motor kapısı ancak bütün sahiplik/mode kapıları ve exact fiziksel güvenlik onayı
  birlikteyse açılır.
- Telemetri, obstacle timeout, bridge command timeout ve STOP korumaları aynen
  kalır.

## Kullanılacak model

Bugünkü testte sarıyı güvenilir gören genel orange/yellow checkpoint kullanılmalı;
başarısız olan `parkur12_best.pt` yalnız dosya adı benziyor diye seçilmemelidir.
Model yolu Jetson'da doğrulanıp `IDA_MODEL_P1P2` olarak verilir. Runtime model
manifesti tam olarak `orange,yellow` olmalıdır.

## Aşama B0 — Henüz hiçbir şey başlatmadan

1. Takımın çalışan stack'i ve servis adı kaydedilir.
2. Bizim alternatif workspace ayrı tutulur.
3. Genel modelin yolu ve SHA-256 değeri kaydedilir.
4. Motor gücü kapalı tutulur.
5. Araç sağlam sehpa üzerinde hareket/dönüş yapamayacak şekilde sabitlenir.
6. Fiziksel tahrik güç kesicisi ve güvenlik gözlemcisi hazır edilir.

## Aşama B1 — Kod ve launch doğrulaması

Bu aşamada ROS launch başlatılmaz:

```bash
cd ~/ida_alt_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 tools/test_bench_avoidance_launch.py
ros2 launch ida_bringup bench_avoidance.launch.py --show-args
```

Beklenen güvenli varsayılanlar:

```text
canonical_takeover_enabled=false
dry_run=true
mavlink_router_enabled=false
guided_mode_enabled=false
motor_command_enabled=false
bench_max_speed_mps=0.25
bench_stuck_timeout_s=30.0
```

## Aşama B2 — Motorsuz sensör/karar doğrulaması

Motor/ESC gücü fiziksel olarak ayrık kalır. Önce genel model, lidar ve füzyon
fresh/ready doğrulanır. Sarı duba yaklaşık 4–6 m ileriye ve merkezden 1–1.5 m
yana konur.

Sağ duba kabulü:

- kamera `yellow`, confidence en az `0.45`,
- fused `lateral_m > 0`,
- costmap sağ-önde sert engel,
- otonomi sola kaçış kararı.

Sol duba kabulü:

- fused `lateral_m < 0`,
- costmap sol-önde sert engel,
- otonomi sağa kaçış kararı.

Her taraf ayrı mission/run olur. Duba taşınırken mission STOP durumundadır.

## Aşama B3 — Dry-run bridge doğrulaması

Pixhawk'a veya motorlara komut çıkmadan `/autonomy/cmd_vel_body` ile limiter
çıktısı karşılaştırılır. Yaw yönü fiziksel duba tarafının tersi olmalıdır. Bu
aşamada motor gücü hâlâ kapalıdır.

## Aşama B4 — Motorlu yön testi

Bu aşamaya yalnız B0–B3 YEŞİL ise geçilir. Komutu çalıştırmadan hemen önce:

- takım stack'i tamamen durmuş,
- Pixhawk seri portu tek sahibinde,
- araç sabit,
- motor tehlike alanı boş,
- fiziksel güç kesici gözlemcinin elinde,
- thrusterların kısa kuru çalışması üretici açısından uygun,
- motor gücü en son adımda açılmış olmalıdır.

Canlı motor profili ek olarak şu exact oturum değişkenini ister:

```bash
export IDA_BENCH_PHYSICAL_SAFETY_ACK=VEHICLE_RESTRAINED_MOTOR_AREA_CLEAR
```

Bu metin PASS kanıtı değildir; yalnız yanlışlıkla motor yolunun açılmasını önleyen
ikinci bir yazılım kilididir. ARM ve mission START YKİ'den ayrıca, operatörün o
an verdiği açık onayla yapılır.

Beklenen fiziksel yön:

- sarı sağda: araç sola dönmek ister; sağ taraf ileri itkisi soldan baskın,
- sarı solda: araç sağa dönmek ister; sol taraf ileri itkisi sağdan baskın.

PWM büyüklüğü tek başına yön kanıtı değildir. Servo reverse ve mixer ayarları
nedeniyle fiziksel itki ve YKİ/Pixhawk servo çıktısı birlikte kaydedilir.

Her koşu 6–8 saniyeyi aşmadan STOP ile sonlandırılır; 30 saniyelik bench stuck
eşiğine yaklaşılmaz. STOP sonrası iki çıkışın nötr, modun HOLD ve aracın DISARMED
olduğu doğrulanır. Sonra motor gücü fiziksel olarak kesilir.

## KIRMIZI durumlar

- fused yön fiziksel yönün tersi,
- duba lidar ve kamera tarafından ayrı iki fiziksel nesne gibi çıkıyor,
- fusion stale/not-ready/overflow,
- canonical topicte birden fazla publisher,
- yaw dubaya doğru,
- beklenmeyen `stuck_recovery`,
- STOP sonrası çıkışın nötre dönmemesi,
- bağlantı kaybında komutun devam etmesi,
- model confidence eşiğini geçmek için sahada threshold düşürme ihtiyacı.

Bu durumlardan birinde önce fiziksel motor gücü kesilir; aynı oturumda karşı taraf
testine geçilmez.

