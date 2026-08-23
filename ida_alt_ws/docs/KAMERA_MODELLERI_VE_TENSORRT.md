# Kamera modelleri ve TensorRT

## Kullanılacak modeller

Görüntü işleme ekibinin ana modelleri şunlardır:

| Görev | Dosya | Gerçek model | Sınıflar |
|---|---|---|---|
| Parkur 1/2 | `parkur12_best.pt` | YOLO11n | orange, yellow |
| Parkur 3 | `parkur3_best.pt` | YOLO11n | red, green, black |

Model klasörü:
`ezel-yazilim_yeni/ezel-yazilim/teknofest_goruntu_isleme-main/models`

Doğrulanmış SHA256 değerleri:

- `parkur12_best.pt`:
  `ea063e2b96f5f1e8c8d5049e0cbdf79b1edc58cd021e15ec0346b04e79b56961`
- `parkur3_best.pt`:
  `8fd27648d41f9ed365706f53883d7c517c0ab33184f8cf5d37303500f6a58313`

Belgelerde YOLOv8n yazan eski ifade doğru değildir; checkpoint metadata'sı iki
modelin de YOLO11n olduğunu gösterir.

## Neden önce `.pt`?

`.pt` taşınabilir geliştirme modelidir. `.engine` ise onu üreten Jetson'ın GPU,
JetPack, CUDA ve TensorRT sürümlerine bağlıdır. Başka bilgisayarda üretilen engine
hedef Jetson'da açılmayabilir veya farklı sonuç verebilir.

ROS tarafında `.pt` ve `.engine` için ortak, fail-closed Ultralytics adapterı
hazırlanmıştır. Model sınıfları yanlışsa veya inference sonucu bozuksa veri taze
sayılmaz. Yine de gerçek engine henüz hedef Jetson'da üretilip `.pt` ile
karşılaştırılmadığı için saha seçeneği olarak açılmamalıdır.

## Güvenli geçiş sırası

1. İki `.pt` dosyasının SHA256 değerini doğrula.
2. Sabit test görüntülerinde `.pt` ile sınıf, kutu ve confidence çıktısını kaydet.
3. Gazebo veya kayıtlı gerçek kamera videosunda raw camera topic'lerini doğrula.
4. Hedef Jetson'ın JetPack, Torch, Ultralytics ve TensorRT sürümlerini kaydet.
5. Engine'i aynı Jetson üzerinde, 640 px, batch 1 ve FP16 ile üret.
6. Aynı görüntülerde `.pt` ve `.engine` sonuçlarını karşılaştır.
7. Gecikme, FPS, sıcaklık ve bellek testini en az 30 dakika çalıştır.
8. Sonuçlar eşleşmiyorsa `.pt` dosyasına geri dön.

## Yarın için eksik veri

Model kalitesini ölçmek için her sınıftan etiketli veya en azından ekipçe gözle
doğrulanmış sabit görüntüler gerekir. Mevcut klasörde örnek görüntü/video yoktur.
Sunucuda Torch/Ultralytics ve GPU runtime da kurulu değildir. Bu iki eksik
tamamlanmadan gerçek model inference testi yapılmış sayılmaz.
