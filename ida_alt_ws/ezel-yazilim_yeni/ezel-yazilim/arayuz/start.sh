#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$ROOT_DIR/backend"
BACKEND_VENV="$BACKEND_DIR/.venv"
REQUIREMENTS_FILE="$BACKEND_DIR/requirements.txt"
REQUIREMENTS_STAMP="$BACKEND_VENV/.requirements-installed"

BACKEND_HOST="${BACKEND_HOST:-0.0.0.0}"
BACKEND_PORT="${BACKEND_PORT:-5000}"
FRONTEND_HOST="${FRONTEND_HOST:-0.0.0.0}"
FRONTEND_PORT="${FRONTEND_PORT:-5173}"
export VITE_WS_URL="${VITE_WS_URL:-ws://localhost:${BACKEND_PORT}/ws}"

PIDS=()

log() {
  printf '[EZEL GCS] %s\n' "$*"
}

fail() {
  printf '[EZEL GCS] ERROR: %s\n' "$*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || fail "$1 is required but not installed."
}

port_in_use() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn | awk '{print $4}' | grep -Eq "[:.]${port}$"
    return
  fi
  if command -v lsof >/dev/null 2>&1; then
    lsof -iTCP:"$port" -sTCP:LISTEN >/dev/null 2>&1
    return
  fi
  return 1
}

ensure_ports_available() {
  if port_in_use "$BACKEND_PORT"; then
    fail "Backend port $BACKEND_PORT is already in use. Stop the old backend or set BACKEND_PORT=..."
  fi
  if port_in_use "$FRONTEND_PORT"; then
    fail "Frontend port $FRONTEND_PORT is already in use. Stop the old frontend or set FRONTEND_PORT=..."
  fi
}

sync_tokens() {
  if [[ -n "${EZEL_WS_TOKEN:-}" && -n "${VITE_WS_TOKEN:-}" && "$EZEL_WS_TOKEN" != "$VITE_WS_TOKEN" ]]; then
    fail "EZEL_WS_TOKEN and VITE_WS_TOKEN do not match. Commands would be rejected."
  fi

  if [[ -z "${EZEL_WS_TOKEN:-}" && -z "${VITE_WS_TOKEN:-}" ]]; then
    local generated_token
    if command -v openssl >/dev/null 2>&1; then
      generated_token="$(openssl rand -hex 24)"
    else
      generated_token="ezel-$(date +%s)-${RANDOM}-${RANDOM}"
    fi
    export EZEL_WS_TOKEN="$generated_token"
    export VITE_WS_TOKEN="$generated_token"
    log "Generated a temporary WebSocket command token for this launch."
    return
  fi

  if [[ -z "${EZEL_WS_TOKEN:-}" ]]; then
    export EZEL_WS_TOKEN="$VITE_WS_TOKEN"
  fi
  if [[ -z "${VITE_WS_TOKEN:-}" ]]; then
    export VITE_WS_TOKEN="$EZEL_WS_TOKEN"
  fi
}

ensure_backend_env() {
  require_command python3

  if [[ ! -x "$BACKEND_VENV/bin/python" ]]; then
    log "Creating backend virtual environment..."
    python3 -m venv "$BACKEND_VENV"
  fi

  if [[ ! -f "$REQUIREMENTS_STAMP" || "$REQUIREMENTS_FILE" -nt "$REQUIREMENTS_STAMP" ]]; then
    log "Installing backend Python dependencies..."
    "$BACKEND_VENV/bin/python" -m pip install --upgrade pip
    "$BACKEND_VENV/bin/python" -m pip install -r "$REQUIREMENTS_FILE"
    touch "$REQUIREMENTS_STAMP"
  fi
}

ensure_frontend_env() {
  require_command npm

  if [[ ! -d "$ROOT_DIR/node_modules" ]]; then
    log "Installing frontend Node dependencies..."
    (cd "$ROOT_DIR" && npm ci)
  fi
}

cleanup() {
  trap - INT TERM EXIT
  if [[ ${#PIDS[@]} -gt 0 ]]; then
    log "Stopping services..."
    kill "${PIDS[@]}" >/dev/null 2>&1 || true
    wait "${PIDS[@]}" >/dev/null 2>&1 || true
  fi
}

start_backend() {
  log "Starting backend on ${BACKEND_HOST}:${BACKEND_PORT}"
  (
    cd "$BACKEND_DIR"
    exec "$BACKEND_VENV/bin/python" -m uvicorn main:app --host "$BACKEND_HOST" --port "$BACKEND_PORT"
  ) &
  PIDS+=("$!")
}

start_frontend() {
  log "Starting frontend on ${FRONTEND_HOST}:${FRONTEND_PORT}"
  (
    cd "$ROOT_DIR"
    exec "$ROOT_DIR/node_modules/.bin/vite" --host "$FRONTEND_HOST" --port "$FRONTEND_PORT" --strictPort
  ) &
  PIDS+=("$!")
}

main() {
  cd "$ROOT_DIR"
  ensure_ports_available
  sync_tokens
  ensure_backend_env
  ensure_frontend_env

  log "Frontend URL: http://localhost:${FRONTEND_PORT}"
  log "Backend health: http://localhost:${BACKEND_PORT}/health"
  log "WebSocket URL: ${VITE_WS_URL}"
  log "Use Ctrl+C to stop both services."

  trap cleanup INT TERM EXIT
  start_backend
  start_frontend

  wait -n "${PIDS[@]}"
}

main "$@"
