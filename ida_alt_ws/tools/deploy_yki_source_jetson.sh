#!/usr/bin/env bash
# Deploy canonical YKI source only.  No backend/frontend process is started.
set -euo pipefail

archive="/tmp/yki_canonical_source_20260816.zip"
target="/home/ezelproject/ida_alt_ws/ezel-yazilim_yeni/ezel-yazilim/arayuz"
expected="85bf6ea468fa1532ad650764d186e8ee4c8f31d831651a18124e88af0dd80249"
actual="$(sha256sum "$archive" | awk '{print $1}')"
[[ "$actual" == "$expected" ]] || { echo "YKİ archive hash mismatch" >&2; exit 2; }

workspace="/home/ezelproject/ida_alt_ws"
mkdir -p "$workspace/backups"
if [[ -d "$target" ]]; then
  tar -czf "$workspace/backups/pre_yki_20260816_$(date +%H%M%S).tar.gz" \
    --exclude=node_modules --exclude=dist --exclude=backend/logs \
    -C "$target" .
fi
mkdir -p "$target"
unzip -q -o "$archive" -d "$target"
echo "YKI_SYNC_OK"
ls -l "$target/package.json" "$target/package-lock.json" "$target/vite.config.js"
