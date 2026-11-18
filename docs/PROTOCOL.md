# Dashboard ⇄ vehicle wire protocol — v1

Transport-independent, binary, versioned, CRC-protected. The same bytes work over UDP, TCP, a
serial/radio link, or a WebSocket/gRPC bridge operated by a dashboard backend.

All multi-byte integers and floats are **little-endian**. Floats are IEEE-754 binary32.

|---|---|
| 1 | Start (requires armed + a loaded mission) |
| 2 | Pause |
| 3 | Resume |
| 4 | Abort |

Telemetry enums: `safety` 0 Nominal, 1 Degraded, 2 Failsafe, 3 EmergencyStop;
`mission_state` 0 Idle, 1 Running, 2 Paused, 3 Completed, 4 Aborted.

## 4. MissionUpload payload

```
