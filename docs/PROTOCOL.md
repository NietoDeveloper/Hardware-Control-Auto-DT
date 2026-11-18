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
Paused. Mission control is separate: upload, then `MissionControl(Start)`.

## 5. Telemetry payload (52 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u64 | `stamp` — vehicle monotonic time, ns |
| 8 | f32 ×3 | `position` x, y, z (m) |
| 20 | f32 ×3 | `velocity` x, y, z (m/s) |
| 32 | f32 | `yaw` (rad) |
| 36 | u8 | `armed` (0/1) |
| 37 | u8 | `mode` |
| 38 | u8 | `safety` |
| 39 | u8 | `mission_state` |
| 40 | u16 | `mission_index` (current waypoint) |
| 42 | u32 | `faults` (bit-field, see SAFETY.md §4) |
| 46 | u16 | `tracks` (confirmed radar obstacles) |
| 48 | u32 | `rejected_commands` (monotonic counter) |

Sent every 100 ms by default (`RuntimeConfig::telemetry_period`). A dashboard that watches
`rejected_commands` can tell when the vehicle refused something.

## 6. Typical session

```
dashboard                                    vehicle
  Heartbeat (10 Hz) ───────────────────────►
  ◄──────────────────────────── Telemetry (10 Hz): armed=0, faults=0
  Arm(1) ──────────────────────────────────►  pre-arm checks → armed, mode=Hold
  MissionUpload(id=7, 12 wp) ──────────────►  stored
  MissionControl(Start) ───────────────────►  mode=Mission, mission_state=Running
  ◄──────────────────────────── Telemetry: mission_index advancing, tracks=n
  SetMode(Manual) / ManualControl(...) ────►  operator takes over live
  MissionControl(Resume) / SetMode(Mission)►  back to the route
  Arm(0) ──────────────────────────────────►  accepted only when nearly stationary
```

## 7. Versioning policy

* `version` changes on any incompatible layout change. A decoder drops other versions
  (`bad_version`).
* New message types and new trailing telemetry fields will be introduced with a version bump until
  a length-negotiated extension mechanism exists (planned).
