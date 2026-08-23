# Risk Registry

| Risk | Impact | Current Mitigation | Required Field Action |
|---|---|---|---|
| Wrong COM port mapped to vehicle | Command reaches wrong vehicle | Separate links and expected SYS_ID validation | Label modems and verify heartbeat SYS_ID |
| IHA disarmed in flight | Vehicle falls | Default emergency action is `RTL` | Do not change to `DISARM` without approved procedure |
| Command sent but not executed | Operator assumes success | Command result `status` separates `state_verified`, `acked`, and `sent_unconfirmed` | Treat `sent_unconfirmed` as warning and observe telemetry after each critical command |
| Emergency cleared accidentally | Unsafe ARM/start | Reset requires both links connected and both vehicles disarmed | Verify armed reset rejection before field use |
| Emergency sent while link is down | Vehicle does not receive safety mode | Emergency remains latched and reconnect reasserts configured emergency action | Verify reassertion log and mode telemetry after link recovery |
| Link loss return-home assumed guaranteed by GCS | Vehicle may not return while MAVLink/RFD link is actually down | First valid GPS is stored as home/breakpoint; backend can only attempt configured `RTL` after reconnect | Configure and field-verify Pixhawk failsafe/RTL independently of GCS software |
| Simultaneous ARM and emergency commands | Unsafe race between clients | Critical command gate actions are serialized; emergency blocks unsafe modes | Verify emergency scenario with one active GCS |
| Unauthenticated WebSocket on field network | Unauthorized command sender | WebSocket command auth requires matching `EZEL_WS_TOKEN`/`VITE_WS_TOKEN`; origin is checked | Use non-empty field token, trusted isolated network, and firewall port 5000 |
| Mission upload incomplete | Incorrect route | Request/ACK protocol, validation, timeout logs | Confirm mission ACK and vehicle mission list |
| Mission start without uploaded route | GUIDED autonomy has no route | Command gate requires last successful upload; SCR_USER6 command becomes ACK only after Jetson downloads and applies the route | Confirm mission upload and Jetson ACK before movement |
| Locked target not delivered to IDA | IDA operates without target | Successful manual lock automatically triggers target send and records delivery status | Treat `sent_unconfirmed` as not proven until custom ACK/vehicle behavior is validated |
| Map tiles unavailable | Reduced situational awareness | Telemetry remains independent | Prepare separate offline/navigation procedure |
