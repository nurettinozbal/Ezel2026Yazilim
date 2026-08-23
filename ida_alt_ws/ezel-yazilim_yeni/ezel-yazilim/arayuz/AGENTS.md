# Repository AI Instructions

## Project Summary

This repository is a React/Vite Ground Control Station interface with a Python FastAPI/pymavlink backend for two Pixhawk/RFD links.

The backend publishes telemetry over WebSocket and routes vehicle commands through a centralized safety gate.

## Current Tech Stack

- React 19
- Vite 7
- React Router
- Context API
- Leaflet / react-leaflet
- Recharts
- CSS Modules
- Python 3.10+
- FastAPI / uvicorn
- pymavlink

## Important Current Truths

- Frontend defaults to backend mode; set `VITE_USE_MOCK=true` for mock telemetry.
- Vehicle commands require WebSocket command auth: backend `EZEL_WS_TOKEN` must match frontend `VITE_WS_TOKEN`.
- Command results include a `status` field; do not treat `sent_unconfirmed` as vehicle execution.
- `EMERGENCY_STOP_ALL` latches backend emergency state and reconnect reasserts emergency action while the latch is active.
- Map tiles depend on external network access through Leaflet/react-leaflet tile loading.
- `node_modules/` is present in the repository and must not be treated as source code.
- `yazilmis_kodlar.txt` has unclear status and must not be treated as source of truth unless explicitly confirmed.

## Source of Truth

Treat these as the current source of truth:

- `src/` for application code
- `backend/` for MAVLink backend code
- `public/` for static assets
- `package.json` for scripts and dependencies
- `docs/` for project-specific AI onboarding notes

Do not treat these as source of truth by default:

- `node_modules/`
- `yazilmis_kodlar.txt`

## Working Rules

- Inspect relevant files before editing.
- Keep changes small and reviewable.
- Follow existing React, CSS Module, and feature-folder patterns.
- Do not rewrite unrelated files.
- Do not add dependencies unless clearly needed.
- Do not assume alerts or local logs mean real hardware behavior exists.
- If a task touches telemetry, command handling, or map behavior, verify whether the flow is real or simulated before changing it.

## Key Files To Read First

- `AI_HANDOFF.md`
- `src/App.jsx`
- `src/context/VehicleContext.jsx`
- `src/services/telemetryService.js`
- `src/context/useVehicle.jsx`
- `src/features/MapSystem/components/MapView.jsx`
- `backend/main.py`
- `backend/services/command_gate.py`
- `backend/links/mavlink_vehicle.py`
- `docs/product-context.md`
- `docs/architecture.md`
- `docs/project-map.md`

## Known Documentation Gaps

- Hardware acceptance results are not documented yet.
- Offline map procedure is not implemented in code.
- RFD900x frequency write flow exists in code but is not verified against real hardware yet.
