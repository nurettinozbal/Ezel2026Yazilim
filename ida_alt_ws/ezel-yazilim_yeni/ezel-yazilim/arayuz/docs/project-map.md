# Project Map

## Project Overview

EZEL GCS is a React/Vite Ground Control Station with a Python FastAPI/pymavlink communication backend for separate IDA and IHA Pixhawk/RFD links.

## Tech Stack

- Frontend: React 19, Vite, React Router, Context API
- Maps: Leaflet, react-leaflet
- Charts: Recharts
- Backend: Python 3.10+, FastAPI, uvicorn, pymavlink
- Transport: WebSocket at `ws://localhost:5000/ws`

## Main Folders

```text
src/ -> frontend application source
backend/ -> MAVLink links, command gate, mission/target services, field logs
public/ -> static assets
docs/ -> project architecture, risks, quality gates, and test scenarios
node_modules/ -> installed dependencies, not source code
```

## Key Files By Task

### Backend startup and configuration

```text
backend/main.py
backend/config.py
backend/README.md
```

### MAVLink and commands

```text
backend/links/mavlink_vehicle.py
backend/services/command_gate.py
backend/services/mission_manager.py
backend/services/target_manager.py
backend/services/telemetry_state.py
```

### Frontend transport and state

```text
src/services/telemetryService.js
src/context/VehicleContext.jsx
```

### Map and mission planning

```text
src/features/MapSystem/components/MapView.jsx
src/features/MissionControl/components/MissionPlanner.jsx
src/features/MissionControl/components/BottomControlBar.jsx
```

## Active Flows

```text
Pixhawk/RFD links -> MAVLinkVehicle -> TelemetryState -> FastAPI WebSocket -> telemetryService -> VehicleContext -> UI
UI command -> telemetryService auth -> FastAPI WebSocket -> CommandGate -> MAVLinkVehicle -> Pixhawk
Mission planner -> VehicleContext -> UPLOAD_IDA_MISSION -> MissionManager -> MAVLink mission protocol
```

## High-Attention Areas

- `backend/config.py`: physical ports, SYS_ID values, and emergency actions must match field setup.
- `backend/services/command_gate.py`: central safety boundary.
- `backend/links/mavlink_vehicle.py`: reconnect, mission protocol, outbound commands, ACK/state status.
- `src/context/VehicleContext.jsx`: UI command intent and backend telemetry.
- `src/context/useVehicle.js`: shared frontend vehicle context hook.
- `MapView.jsx`: external map tile network dependency.
- `yazilmis_kodlar.txt`: snapshot only, not source of truth.
