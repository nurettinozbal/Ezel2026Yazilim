# Product Context

## What This Project Is

This project is a React/Vite Ground Control Station interface backed by a Python FastAPI/pymavlink communication service.

It presents a control-room style UI for monitoring and operating a vehicle mission workflow, including:

- operation view
- mission planning
- engineering charts
- settings and logs

## Current Product Scope

The current repository covers the GCS MVP control loop:

- real telemetry is received from the backend WebSocket by default
- mock telemetry remains available with `VITE_USE_MOCK=true`
- vehicle commands go through backend WebSocket auth and `CommandGate`
- command results expose `status` values such as `state_verified`, `acked`, and `sent_unconfirmed`
- camera feeds and modem write settings are still placeholder/product gaps

## Main User-Facing Areas

- `/` shows the main operation view with telemetry, map, and camera panels
- `/mission` shows waypoint planning on the map
- `/engineering` shows chart-based engineering telemetry views
- `/settings` shows communication settings and a live log terminal

## Important Product Constraints

- Map tiles depend on external network access through Leaflet/react-leaflet
- hardware execution still must be confirmed with Pixhawk ACK/state telemetry during controlled field tests
- `sent_unconfirmed` command results mean the message was written to MAVLink but vehicle execution was not proven
- command buttons require matching `VITE_WS_TOKEN` and `EZEL_WS_TOKEN`

## Repo Notes For AI

- `node_modules/` is present in the repo and should not be treated as source code
- `yazilmis_kodlar.txt` has unclear status and should not be treated as source of truth unless confirmed
