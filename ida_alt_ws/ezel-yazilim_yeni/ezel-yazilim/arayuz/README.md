# EZEL GCS

React/Vite frontend ve FastAPI/pymavlink backend ile calisan yer kontrol istasyonu arayuzu.

## Hizli Baslatma

Yerel makinede backend ve frontend'i birlikte baslatmak icin:

```bash
./start.sh
```

Windows icin:

```bat
start.bat
```

Script backend'i `http://localhost:5000`, frontend'i `http://localhost:5173` uzerinde baslatir. Detaylar icin [docs/quick-start.md](docs/quick-start.md) dosyasina bak.

Saha token'i ile baslatma:

```bash
EZEL_WS_TOKEN="uzun-rastgele-saha-token" VITE_WS_TOKEN="uzun-rastgele-saha-token" ./start.sh
```

## Gelistirme Komutlari

```bash
npm run lint
npm test
npm run test:backend
npm run build
```

## Not

Gercek Pixhawk/RFD davranisi donanim baglanmadan kanitlanamaz. Saha kullanimi oncesi `backend/README.md`, `docs/risk-registry.md` ve `docs/test-scenarios.md` kontrol edilmelidir.

<!-- Original Vite template notes kept below for dependency context. -->

This template provides a minimal setup to get React working in Vite with HMR and some ESLint rules.

Currently, two official plugins are available:

- [@vitejs/plugin-react](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react) uses [Babel](https://babeljs.io/) (or [oxc](https://oxc.rs) when used in [rolldown-vite](https://vite.dev/guide/rolldown)) for Fast Refresh
- [@vitejs/plugin-react-swc](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react-swc) uses [SWC](https://swc.rs/) for Fast Refresh

## React Compiler

The React Compiler is not enabled on this template because of its impact on dev & build performances. To add it, see [this documentation](https://react.dev/learn/react-compiler/installation).

## Expanding the ESLint configuration

If you are developing a production application, we recommend using TypeScript with type-aware lint rules enabled. Check out the [TS template](https://github.com/vitejs/vite/tree/main/packages/create-vite/template-react-ts) for information on how to integrate TypeScript and [`typescript-eslint`](https://typescript-eslint.io) in your project.
