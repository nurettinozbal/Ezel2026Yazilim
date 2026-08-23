# Saha servisi ve YKİ görev akışı

Bu profil, takımın çalışan `idaws` kurulumunu silmeden veya değiştirmeden alternatif
olarak kullanılmak üzere hazırlanmıştır. Servis açılışta çalışmaz. Kurulum işlemi
servisi başlatmaz ve boot'a eklemez.

## Ne çalışır?

Servis operatör tarafından başlatıldığında şu bileşenleri tek süreç ağacı altında
açar:

- Arducam kamera sürücüsü (varsayılan 960x600, 30 FPS)
- Tek general YOLO modeli ve iki renk rolü
  - P1/P2: yalnız `orange`, `yellow`
  - P3: yalnız `red`, `green`, `black`
- S2 lidar ve araç-merkezli lidar köprüsü
- Kamera-lidar sensör füzyonu
- Otonomi, komut sınırlayıcı ve loglama
- Pixhawk seri hattının tek sahibi olan TTY router ve YKİ görev posta kutusu

General model yalnız bir kez inference yapar. Sonuç iki ham kamera kanalına renk
rolüne göre ayrılır; aynı model iki kez çalıştırılıp GPU yükü ikiye katlanmaz.

Servis şunları **yapmaz**:

- ARM göndermez.
- GUIDED moda geçirmez.
- Görev yüklemez veya başlatmaz.
- Pixhawk parametrelerini otomatik değiştirmez.
- Çalışan `idaws.service` sürecini otomatik durdurmaz.

## Bir kez yapılacak kurulum

Jetson'a workspace kopyalanıp derlendikten sonra:

```bash
cd ~/ida_alt_ws
chmod +x scripts/start_field_stack.sh scripts/install_field_service.sh
./scripts/install_field_service.sh
```

Bu komut yalnız systemd dosyasını ve ilk örnek ayar dosyasını yerleştirir. Servis
başlamaz. Ayarlar:

```bash
sudo nano /etc/ida/field-test.env
```

Model dosyalarının gerçek yolları kontrol edilmelidir. Bu proje için P1/P2 ve P3
model yolları aynı `general_yolo11s_20260808.pt` dosyasını göstermelidir.

## İki güvenli çalışma seviyesi

### 1. Algı/karar kontrolü, motor yolu kapalı

Önce bu seviyede başlanır:

```text
IDA_CANONICAL_TAKEOVER=true
IDA_FIELD_DRY_RUN=false
IDA_MAVLINK_ROUTER_ENABLED=true
IDA_GUIDED_MODE_ENABLED=false
IDA_MOTOR_COMMAND_ENABLED=false
```

Bu seviye Pixhawk/YKİ telemetrisi, kamera, lidar, füzyon, costmap ve otonomi
kararını gösterir; motor komutunu bridge'den geçirmez.

### 2. YKİ kontrollü görev ve motor yolu

Fiziksel saha güvenliği sağlandıktan sonra:

```text
IDA_CANONICAL_TAKEOVER=true
IDA_FIELD_DRY_RUN=false
IDA_MAVLINK_ROUTER_ENABLED=true
IDA_GUIDED_MODE_ENABLED=true
IDA_MOTOR_COMMAND_ENABLED=true
IDA_FIELD_PHYSICAL_SAFETY_ACK=FIELD_OPERATOR_READY_NO_AUTO_ARM
```

Dört füzyon hazır bayrağı da ancak ilgili bench kanıtları kabul edildiyse `true`
olmalıdır. Bu ayarlar motor komut yolunu açar; buna rağmen araç hâlâ ARM olmaz ve
görev başlamaz. ARM ve START yalnız YKİ'deki doğrulanmış komut zincirinden gelir.

## Her çalıştırmadan önce

1. `idaws.service` ve aynı cihazları kullanan başka stack kapalı olmalıdır.
2. Pixhawk ve lidar benzersiz `/dev/serial/by-id` kimliğiyle; Arducam benzersiz
   `/dev/v4l/by-id` kimliğiyle otomatik bulunur. Birden fazla aday varsa sistem
   tahmin yürütmez ve fail-closed durur.
3. Model yolları ve general modelin sınıf sırası doğrulanır:
   `black,green,orange,red,yellow`.
4. YKİ bilgisayarında backend ve arayüz açılır.
5. Araç çevresi ve motor güvenliği ekip tarafından fiziksel olarak kontrol edilir.

Servisi manuel başlatma:

```bash
sudo systemctl start ida-canonical-field.service
sudo journalctl -fu ida-canonical-field.service
```

Servis preflight sırasında başka süreç cihazları kullanıyorsa kendi kendine durur;
diğer süreci kapatmaz.

## YKİ'de görev hazırlama

1. Görev Planlama ekranında **Yeni Nokta Parkuru** seçilir.
2. Haritada ilk tıklamada araç konumu HOME/P1 olarak eklenir.
3. Sonraki noktalar seçili P1, P2 veya P3 etiketiyle eklenir.
4. Parkur sırası geriye gitmemelidir: P1 → P2 → P3.
5. Noktalar kontrol edilip **Görev Yükle** seçilir.
6. YKİ Pixhawk yükleme sonucunu doğrulamadan START aktif kabul edilmez.
7. Fiziksel güvenlik kontrolünden sonra ARM verilir.
8. **Görev Başlat** seçilir. Bridge ancak YKİ/Pixhawk posta kutusundan doğrulanmış
   START isteğini gördüğünde otonomi komut yolunu açar.
9. Herhangi bir anormallikte önce **Görev Durdur**, gerekirse **Acil Durdur** kullanılır.

## Parkur bazlı kullanım

- P1 testi: HOME ve hedef noktaları P1 olarak işaretle.
- P2 testi: önce en az bir P1 HOME/anchor, sonra P2 hedefleri ekle.
- P3 testi: önce P1 HOME/anchor; görev akışına göre P2 ve ardından P3 ekle.
- Tam test: waypoint etiketleri sırasıyla P1 → P2 → P3 olmalıdır.

Arayüz sabit GPS koordinatı üretmez. Saha koordinatlarını operatör haritadan seçer;
bu sayede yanlış bölgede gömülü bir demo rotası çalıştırılamaz.

## Durdurma ve takım stack'ine dönüş

```bash
sudo systemctl stop ida-canonical-field.service
sudo systemctl status ida-canonical-field.service --no-pager
```

Canonical servis tamamen durduktan ve seri/kamera/lidar aygıtları serbest kaldıktan
sonra takımın kendi servisi ayrıca operatör tarafından başlatılabilir. Canonical
servis bunu otomatik yapmaz.

## Loglar

Her açılış benzersiz bir `run_...` klasörü üretir. YKİ'deki görev sonucu ile aynı
zaman aralığı şu kaynaklarda aranır:

- servis journal'ı;
- telemetry CSV;
- autonomy/fusion JSONL;
- costmap kayıtları;
- işlenmiş kamera kaydı.

Servis başlamazsa önce `journalctl` içindeki `RED:` satırı çözülmelidir; güvenlik
kontrolü atlanmamalıdır.
