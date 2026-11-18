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
u32 mission_id
u8  loop        (0 or 1)
u16 count       (1..128)
count × {
  f32 x, f32 y, f32 z        local ENU metres
  f32 speed_mps              cruise speed toward this waypoint
  f32 acceptance_m           reached when within this 3-D radius
  f32 hold_s                 dwell on arrival (0 = fly through)
}
```

Payload size = 7 + 24 × count bytes (max 3 079). Uploads are rejected while a mission is Running or
Paused. Mission control is separate: upload, then 