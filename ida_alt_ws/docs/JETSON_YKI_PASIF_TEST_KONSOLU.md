# Jetson – YKİ pasif test konsolu

## Ne yapar?

Jetson'daki ROS 2 verilerini YKİ'nin **Mühendislik & Test** sayfasında özetler.
Kamera tespitleri, lidar noktaları/kümeleri, fusion nesneleri, otonomi durumu ve
seçilen hareket komutu araç merkezli 2B görünümde izlenebilir. Testler yalnız
allowlist'teki pasif vakalardır; shell komutu veya serbest topic çalıştırılamaz.

Bu konsol ARM etmez, mod değiştirmez ve motor komutu yayınlamaz.

Takımın Jetson'da halen kullandığı kodu değiştirmeden test etme ve alternatif
rosbag-replay akışı için
[Jetson alternatif test planı](JETSON_ALTERNATIF_TEST_PLANI.md) izlenmelidir.

## Veri yolu

```text
ROS sensör/otonomi topic'leri
          |
          v
ida_vehicle_test_monitor ---- kanıt/artifact
          |
ida_vehicle_test_producer (ayrı WebSocket kimliği)
          |
YKİ FastAPI /ws/jetson-debug
          |
Mühendislik & Test sayfası
```

Ham video bu WebSocket'e konmaz. Kamera görüntüsü gerekiyorsa laboratuvarda
ayrı, sınırlı bir taşıma kanalı kullanılmalıdır; yarış build'ine otomatik
eklenmez.

## Jetson'da pasif overlay

Her terminalde ROS ortamı yüklenir. Producer tokenı ROS parametresi veya launch
argümanı değil, proses ortam değişkenidir. Token belgelerde/loglarda tutulmaz.

```bash
cd ~/ida_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export EZEL_JETSON_DEBUG_TOKEN='<ayri-jetson-tokeni>'
ros2 launch ida_bringup vehicle_test_lab.launch.py \
  yki_url:=ws://<YKI-IP>:5000/ws/jetson-debug \
  evidence_root:=/home/ezel/ida_test_evidence \
  log_root:=/home/ezel/ida_logs \
  simulation_sources:=false
```

Simülasyonda son argüman `simulation_sources:=true` yapılır. Gerçek araç
launch'unda false kalmalıdır.

Python 3.10 ortamında eski `websockets 9.1` uyumsuzdur. Producer
`websockets >=10.4,<14` aralığını zorunlu tutar ve bunun dışında fail-closed
devre dışı kalır. Jetson kurulumu:

```bash
python3 -m pip install --user 'websockets>=10.4,<14'
python3 -c 'import websockets; print(websockets.__version__)'
```

Kurulu sürüm saha manifestine yazılmalıdır.

## YKİ backend

Lab modu varsayılan olarak kapalıdır. Açılırken browser ve Jetson için iki
farklı, boş olmayan token gerekir:

```bash
export EZEL_LAB_DEBUG_ENABLED=true
export EZEL_WS_TOKEN='<operator-tokeni>'
export EZEL_JETSON_DEBUG_TOKEN='<operator-tokeninden-farkli-token>'
cd ezel-yazilim_yeni/ezel-yazilim/arayuz/backend
uvicorn main:app --host 0.0.0.0 --port 5000
```

Frontend lab build'i:

```bash
cd ezel-yazilim_yeni/ezel-yazilim/arayuz
VITE_ENABLE_LAB_DEBUG=true VITE_WS_URL=ws://<YKI-IP>:5000/ws npm run dev
```

Saha ağında `ws://` açık internete verilmez. Kapalı güvenilir LAN,
firewall/VPN veya TLS/WSS kullanılır.

## Test sırası

1. `comms`
2. `telemetry`
3. `camera_p1p2`
4. `camera_p3`
5. `lidar`
6. `fusion_shadow`
7. `autonomy_shadow`
8. `logging`

Her test yeni oturumdan sonra gelen benzersiz acquisition stamp'li örnekleri
ister. Bayat, tekrarlanan veya test başlamadan önce alınan veri PASS sayılmaz.
Canonical buoy/obstacle topic'lerinde tek writer, doğru tip ve beklenen node
kimliği tekrar kontrol edilir.

PASS/FAIL/CANCELLED sonuçları JSON artifact olarak diske yazılır. PASS, artifact
başarıyla yazılmadan yayınlanamaz. Artifact'taki `actuation_enabled` her zaman
`false` olmalıdır.

## PET şişe yerleşimleri

- Tek turuncu veya sarı: merkezde 2, 5 ve 10 m
- Aynı mesafelerde yaklaşık 20 derece sağ ve sol
- İki şişe: farklı renk/açı, sonra yerleri ters
- Birbirine yakın açı: fusion renk uydurmamalı
- Kamera kapalı: lidar hard-unknown engel kalmalı
- Lidar kapalı/menzil dışı: kamera-only tespit karar verici fused duba olmamalı
- Aynı stamp replay, kamera/lidar durdurma ve saat geri alma: bayat veri taze
  görünmemeli

Her poz en az 10 saniye tutulur. Şişeler mat boyanır ve ölçülen mesafe/açı
test formuna yazılır.

## Bilinen sınırlar

- PET şişe testi gerçek yarış dubasının yerini tutmaz.
- DDS node adı bir kimlik doğrulama mekanizması değildir; ROS domain'i
  güvenilir ve erişim kontrollü tutulmalıdır.
- Producer/relay kuyrukları RAM tabanlıdır; proses yeniden başlatılırsa aktif
  oturum yeniden başlatılmalıdır.
- Gerçek ARM/motor testi bu konsolun kapsamında değildir.
