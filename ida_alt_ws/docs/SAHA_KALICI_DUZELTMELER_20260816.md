# 16 Ağustos saha bulguları — kalıcı kapanış listesi

Bu liste “o an çalışsın” diye yapılan geçici müdahaleleri değil, ana kaynakta
korunan davranışları tarif eder. Yeni bilgisayar/Jetson kurulumu bu kaynak ve
testlerle yapılmalıdır.

| Sahada görülen sorun | Kalıcı davranış |
|---|---|
| Harita aracın konumunu sentetik HOME waypoint yaptı; ilk hedefte motorlar salındı | HOME yalnız RTL/dönüş işaretidir. İlk harita tıklaması doğrudan İlk Hedef #1 olur. |
| İlk waypoint ve engel sonrası dönüşler geniş kaldı | Hedef açısı 42° veya daha büyükse araç önce 30°/sn ile yerinde hizalanır. 23° altına kadar hizalanma fazı korunur; eşikte ileri/dön titreşimi olmaz. |
| Küçük yaw komutları su üzerinde yön değiştirmedi | DWA hareketli dönüş alt sınırı 20°/sn; recovery 25°/sn; ilk hizalanma 30°/sn'dir. |
| Kaçınırken itki etkisiz kaldı | DWA normal/yavaş/recovery ve P3 hareket yollarında komut tabanı 0,70 m/s olacak şekilde tek profilde tutulur. Bu değer PWM/yeterli su itkisi garantisi değildir. Genel uygulama tavanı 1,60 m/s'dir ve otonomi-limiter-GUIDED aynı değeri alır. |
| Kamera rengi okuyamazsa engel kaybolur kaygısı | Lidar geometrisi otoritedir. Kamera yalnız renk ekler. Eşleşmeyen lidar kümesi `unknown hard obstacle` kalır. Kamera bayat olsa bile taze lidar engelleri yayımlanır. |
| Lidar node'u/servisi açılmadı veya sessiz kaldı | Lidar sürücüsü ve köprüsü otomatik respawn olur. `ida_cli start`, iki node yanında gerçek `/scan` ve raw obstacle mesajını görmeden hazır sayılmaz; aksi halde stack kapanır. |
| YKİ lidar görünümü için IP/WebSocket/token elle ayarlandı | YKİ tek başlatıcıyla ayrı salt-okunur eşleme anahtarını yükler ve LAN keşif yayını yapar. Jetson `auto://yki` ile sabit IP olmadan bağlanır; `ida_cli start` pasif akış servisini de açar. |
| P3 hedef araması hızlı veya kararsızdı | Hedef yokken sürekli 10°/sn 360° arama vardır. Hedef güven eşiği %20'dir fakat en az 0,4 saniye ve 3 bağımsız kare görülmeden angajman açılmaz. |
| SCR_USER6 START/STOP pending durumu görevi kilitledi | Mailbox sırası tek kontrattan yürür; pending token üzerine yazılmaz. START/STOP ACK zaman aşımı HOLD'a geçer. Araç DISARM+HOLD iken kontrollü mailbox kurtarma STOP_ACK'e alır. |
| Görev silme “bilinmeyen komut” verdi | `CLEAR_IDA_MISSION` backend komut listesinde ve YKİ akışında tanımlıdır; yalnız doğrulanmış Pixhawk clear sonucu başarılı gösterilir. |
| Acil stop sonrası reset/disarm kilitlendi | DISARM acil durumda izinli öncelikli komuttur. Emergency reset yalnız araç disarm ve güvenli mod doğrulandıktan sonra kilidi kaldırır. |

## Tek ayar kaynağı

Saha davranış ayarları yalnız `src/ida_bringup/config/field_profile.yaml`
dosyasından değiştirilir. Ayrıntılı açıklama `docs/SAHA_AYAR_PROFILI.md` içindedir.

## Yerel doğrulama özeti

- Python/ROS-saf ana tarama: 494 test geçti, Windows symlink yetkisi isteyen 1
  Linux testi atlandı.
- YKİ backend: 73 test geçti.
- YKİ frontend: 65 test, lint ve üretim build geçti.
- Shell başlangıç scriptleri: sözdizimi kontrolü geçti.

Bu sonuçlar araç donanımına bağlanmadan alınmıştır. Jetson'a dağıtım sonrası
aynı testler yeniden koşulmadan kaynakların birebir eş olduğu kabul edilmez.
