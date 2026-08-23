# Architecture

## Current Architecture Summary

This repository contains a React/Vite Ground Control Station frontend and a Python FastAPI/pymavlink communication backend.

## Main Technical Shape

- `src/main.jsx` boots the React app.
- `src/context/VehicleContext.jsx` holds frontend telemetry, mission, emergency, and log state.
- `src/context/useVehicle.js` exposes the shared vehicle context hook.
- `src/services/telemetryService.js` owns WebSocket/mock transport.
- `backend/main.py` owns WebSocket clients and backend lifecycle.
- `backend/links/mavlink_vehicle.py` owns the two independent Pixhawk/RFD MAVLink links.
- `backend/services/command_gate.py` is the only frontend command execution entry point.

## Telemetry Flow

1. Separate IDA and IHA link threads read MAVLink and reconnect independently.
2. `TelemetryState` normalizes messages into the frontend contract.
3. FastAPI publishes telemetry at 2 Hz over `ws://localhost:5000/ws`.
4. `telemetryService` passes telemetry, log, and command-result messages into `VehicleContext`.

Frontend defaults to the real backend. Set `VITE_USE_MOCK=true` to use mock telemetry. Field configuration, WebSocket command auth, and startup are documented in `backend/README.md`.

## Command And Control Behavior

ARM/DISARM, emergency, mission upload/start/stop, target lock, mode changes, and target forwarding pass through `CommandGate`.

Vehicle commands require WebSocket command auth. Backend `EZEL_WS_TOKEN` must match frontend `VITE_WS_TOKEN`; otherwise telemetry may still stream but commands are rejected as `unauthorized`.

Command results include a `status` field. `state_verified` means telemetry confirmed the requested state, `acked` means Pixhawk ACK was received, and `sent_unconfirmed` means the MAVLink frame was written without vehicle confirmation.

The backend emergency latch blocks ARM, mission start, and mission upload until `RESET_EMERGENCY` succeeds. If a vehicle reconnects while emergency is active, the backend reasserts that vehicle's configured emergency action.

The backend also records each vehicle's first valid GPS position as a home/breakpoint. If `EZEL_AUTO_RETURN_ON_LINK_LOSS=true`, heartbeat timeout marks return-home pending and reconnect attempts the configured `EZEL_RETURN_HOME_MODE` once. This is not a substitute for Pixhawk-level failsafe because the GCS cannot command a vehicle while the MAVLink/RFD link is actually unavailable.

## Map Architecture

- Map rendering uses Leaflet and `react-leaflet`.
- Map tiles load from an external Esri tile service.
- MAVLink telemetry and commands remain independent from map tile availability.
- Map startup centers on the first valid IDA GPS once, then stays independent from live vehicle movement unless the operator uses focus controls.
