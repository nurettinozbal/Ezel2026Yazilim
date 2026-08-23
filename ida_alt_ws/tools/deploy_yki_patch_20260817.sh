#!/usr/bin/env bash
# Source-only YKI (arayuz) sync for the 2026-08-17 batch fix.
# Covers: ACK13 mission upload transaction (mavlink_vehicle.py),
# P3 bench mode (bench_mode env, BENCH rozeti, P3-only rota gevsetmesi).
# Yalniz YKI bilgisayarinda (Windows/Linux) calistirilir — Jetson'da degil.
# Servis baslatmaz, ARM/motor/mod komutu gondermez.
set -euo pipefail

archive="/tmp/ida_yki_20260817.zip"
expected="b81130ba46e8225861960882a4914238805517439aa5c6e1bd9016b1c2fd5490"

if [[ ! -f "$archive" ]]; then
  echo "RED: arsiv yok: $archive" >&2
  echo "  scp ida_yki_20260817.zip <yki-bilgisayar>:/tmp/" >&2
  exit 2
fi
actual="$(sha256sum "$archive" | awk '{print $1}')"
if [[ "${actual,,}" != "$expected" ]]; then
  echo "RED: Archive SHA-256 uyusmazligi" >&2
  echo "  beklenen: $expected" >&2
  echo "  gelen   : $actual" >&2
  exit 2
fi

# YKI kokunu buradan turet: script bu repoda calisirsa kok bellidir.
if [[ -d "ezel-yazilim_yeni/ezel-yazilim/arayuz" ]]; then
  YKI_ROOT="$(pwd)/ezel-yazilim_yeni/ezel-yazilim/arayuz"
elif [[ -d "arayuz" ]]; then
  YKI_ROOT="$(pwd)/arayuz"
else
  echo "RED: YKI kok bulunamadi; scripti repo kokunden calistirin" >&2
  exit 2
fi

mkdir -p "$YKI_ROOT/backups"
# Yedek: zip icindeki dosyalari arayuz agaci altinda bul.
existing=()
while IFS= read -r line; do
  rel="${line#ezel-yazilim_yeni/ezel-yazilim/arayuz/}"
  if [[ "$rel" != "$line" && -e "$YKI_ROOT/$rel" ]]; then
    existing+=("$YKI_ROOT/$rel")
  fi
done < <(unzip -Z1 "$archive" | grep -v '/$')
if ((${#existing[@]})); then
  tar -czf "$YKI_ROOT/backups/pre_yki_fix_20260817_$(date +%H%M%S).tar.gz" "${existing[@]}"
fi

unzip -q -o "$archive" -d "$YKI_ROOT/../../.."

echo "SYNC_OK: 2026-08-17 YKI batch fix (ACK13 + bench mode)"
echo "Sonraki adimlar (YKI bilgisayarinda):"
echo "  cd ezel-yazilim_yeni/ezel-yazilim/arayuz"
echo "  backend test: cd backend && python -m unittest discover -s tests"
echo "  frontend test: npm test"
echo "  EZEL_BENCH_MODE=true ile baslat (start.bat / start.sh)"
