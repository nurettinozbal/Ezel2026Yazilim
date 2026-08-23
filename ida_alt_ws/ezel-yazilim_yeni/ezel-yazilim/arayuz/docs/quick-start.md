# Quick Start

Bu projede su an en az riskli hizli baslatma yolu `.sh` scriptidir.

## Neden exe degil?

- Frontend React/Vite, backend FastAPI/pymavlink olarak iki ayri runtime kullanir.
- Exe paketleme icin Electron/Tauri/PyInstaller gibi yeni paketleme kararlari gerekir.
- Pixhawk/RFD seri port erisimi paketli uygulamada yine saha bilgisayari izinlerine bagli kalir.

## Neden Docker simdilik ikinci tercih?

- Backend gercek donanim icin `/dev/ttyUSB0` ve `/dev/ttyUSB1` gibi seri cihazlara erismelidir.
- Docker ile bu cihazlar ayrica map edilmelidir ve Linux/Windows farklari saha riskini artirir.
- Harita ve komut guvenligi icin network ayarlari da ek kontrol ister.

## Secilen yol

Yerel makinede tek komut:

```bash
./start.sh
```

Windows icin:

```bat
start.bat
```

Script sunlari yapar:

- backend icin `backend/.venv` olusturur
- Python dependency'lerini kurar
- `node_modules` yoksa `npm ci` calistirir
- backend'i `http://localhost:5000` uzerinde baslatir
- frontend'i `http://localhost:5173` uzerinde baslatir
- `EZEL_WS_TOKEN` ve `VITE_WS_TOKEN` yoksa gecici bir token uretip ikisine de uygular
- tokenlar farkliysa baslamadan durur

Windows `start.bat`, backend ve frontend icin iki ayri terminal penceresi acar. Durdurmak icin bu pencerelerde `Ctrl+C` kullanin veya pencereleri kapatin.

Windows'ta gercek RFD/Pixhawk kullanmadan once COM portlarini kesinlestirin:

```bat
set EZEL_IDA_PORT=COM3
set EZEL_IHA_PORT=COM4
start.bat
```

## Saha token'i ile baslatma

Kalici saha token'i kullanmak icin:

```bash
EZEL_WS_TOKEN="uzun-rastgele-saha-token" VITE_WS_TOKEN="uzun-rastgele-saha-token" ./start.sh
```

Windows CMD:

```bat
set EZEL_WS_TOKEN=uzun-rastgele-saha-token
set VITE_WS_TOKEN=uzun-rastgele-saha-token
start.bat
```

## Port veya WebSocket adresi degistirme

```bash
BACKEND_PORT=5001 FRONTEND_PORT=5174 VITE_WS_URL=ws://localhost:5001/ws ./start.sh
```

Windows CMD:

```bat
set BACKEND_PORT=5001
set FRONTEND_PORT=5174
set VITE_WS_URL=ws://localhost:5001/ws
start.bat
```

Baska bir cihazdan frontend acilacaksa `localhost` yerine YKI bilgisayarinin IP adresini kullanin:

```bash
VITE_WS_URL=ws://192.168.1.10:5000/ws ./start.sh
```

## Durdurma

Terminalde `Ctrl+C` kullanin. Script backend ve frontend sureclerini birlikte kapatir.

## Docker sonraki adim

Docker gerekiyorsa ayri bir `docker-compose.yml` eklenmeli ve en az sunlar dogrulanmalidir:

- `/dev/ttyUSB0` -> IDA modem mapping
- `/dev/ttyUSB1` -> IHA modem mapping
- `EZEL_WS_TOKEN`/`VITE_WS_TOKEN` runtime aktarimi
- host network veya port mapping
- Linux/Windows saha bilgisayari farklari
