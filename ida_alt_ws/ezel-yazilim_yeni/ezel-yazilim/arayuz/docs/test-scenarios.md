# Test Scenarios

## Automated

1. Command gate always accepts and latches emergency.
2. Emergency latch rejects ARM and mission start.
3. Invalid vehicle modes and empty missions are rejected.
4. Locked target cannot change without force.
5. Disconnected telemetry matches the WebSocket contract.
6. Mission start is rejected until a successful upload.
7. Emergency reset is rejected for armed or disconnected vehicles.
8. Emergency state rejects mission upload.
9. IHA connection requires HEARTBEAT, not only non-heartbeat telemetry.
10. Target delivery status is recorded after `SEND_TARGET_TO_IDA`.
11. First valid GPS locks home/breakpoint and return-home status is recorded.
12. Frontend lint, unit tests, and production build succeed.

## Integration Without Hardware

1. Start backend with unavailable COM ports.
2. Confirm `/health` remains `status=ok`.
3. Connect to `/ws` and receive telemetry every 500 ms.
4. Send `ARM_IDA`; confirm rejection and command log entry.
5. Disconnect frontend; confirm backend continues running.
6. Confirm `/health` exposes target, mission, and emergency state.
7. Connect without `VITE_WS_TOKEN`/auth; confirm telemetry streams but commands return `unauthorized`.
8. Connect with matching `EZEL_WS_TOKEN`/`VITE_WS_TOKEN`; confirm commands reach `CommandGate`.
9. Confirm telemetry exposes IDA/IHA home/breakpoint after first valid GPS.

## Controlled Field Sequence

1. Verify IDA heartbeat and telemetry.
2. Verify IHA heartbeat and telemetry.
3. Verify ARM/DISARM on a physically safe platform.
4. Upload a short IDA mission and confirm mission ACK.
5. Start and stop the mission while observing command `status` and mode telemetry.
6. Trigger emergency and verify IDA `HOLD` plus IHA `RTL`.
7. Power-cycle or temporarily disconnect one link while emergency is active; verify reconnect reassertion log.
8. Verify reset rejection while either vehicle is armed, then reset while both are disarmed.
9. Manually lock a target and confirm automatic `SEND_TARGET_TO_IDA` delivery-status log.
10. Verify Pixhawk-level failsafe/RTL returns to the start position on real link loss; GCS reconnect return-home is only a secondary attempt.
