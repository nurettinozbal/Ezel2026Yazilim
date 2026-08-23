# Quality Gates

## Before Field Use

- `python -m unittest discover -s tests -v` passes from `backend/`.
- `npm run lint` passes from repository root.
- `npm test` passes from repository root.
- `npm run build` passes from repository root.
- Backend remains running when both COM ports are unavailable.
- `/health` and WebSocket telemetry report both links accurately.
- `EZEL_WS_TOKEN` and `VITE_WS_TOKEN` are set to the same non-empty field token.
- Unauthorized WebSocket command attempts are rejected.
- Correct physical modem-to-vehicle port mapping is labeled and verified.
- IDA/IHA SYS_ID and GCS SYS_ID are verified on real hardware.

## Controlled Hardware Gates

- ARM/DISARM is tested without propeller or motor hazard.
- Each configured mode exists in the corresponding autopilot.
- Critical commands produce `acked` or `state_verified`; `sent_unconfirmed` is treated as a warning.
- Mission upload receives a successful MAVLink mission ACK.
- Emergency behavior is observed: IDA `HOLD`, IHA `RTL`.
- Emergency reconnect reassertion is observed while emergency latch is active.
