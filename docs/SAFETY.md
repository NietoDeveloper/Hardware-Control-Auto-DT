# Safety concept

This document describes how `sdt-vehicle-core` tries to stay safe, what it does when things go
wrong, and — just as importantly — what it does **not** claim.

## 1. Scope and honesty

v1 applies safety-critical *engineering practices* (determinism, static allocation, layered
protection, strict validation, fault injection). It is **not certified** to any 
| 0 | `kLinkLost` | No valid command frame within `link_hold_after` |
| 1 | `kStateStale` | No valid state estimate within `state_stale_after` |
| 2 | `kRadarStale` | No radar scan within `radar_degraded_after` (when radar is required) |
| 3 | `kGeofence` | Vehicle is outsidtive conditions.

### Hysteresis and latching

* A failsafe action is held until all triggering conditions have been absent for
  `recovery_time` (default 1 s). This prevents flapping when a link is marginal.
* `EmergencyStop` latches. It is cleared only by an explicit operator **disarm**
  (`supervisor.reset()`), never automatically.
* While disarmed the supervisor still reports faults (used by pre-arm checks) but never escalates.

## 5. Pre-arm checks

Arming is refused (and counted in `rejected_commands`) unless:

* the supervisor is not latched,
* a state estimate is fresh,
* the radar is fresh (when `radar_required`),
* the link is fresh,
* the vehicle is inside the geofence.

On arming, home is recorded, the guard's slew limiter is seeded with the measured velocity, and the
mode is `Hold`.

## 6. Command guard (last line of defence)

Applied in this order to every setpoint, in every mode:

1. Reject non-finite vectors (replace with zero).
2. Clamp vertical speed.
3. Limit horizontal speed to `min(platform limit, verdict speed limit)`.
4. **Geofence barrier:** compute the stopping point (v²/2a plus a margin); if it would lie outside
   *and* be worse than the current position, command zero. A vehicle already outside may still
   move back inward.
5. Acceleration slew (separate limits for speeding up and braking), relative to the previous command.
6. Clamp yaw rate.

## 7. Emergency stop contract

`IActuatorSink::emergencyStop()` must be **idempotent, non-blocking and vehicle-appropriate**:

* ground robot: cut drive power and apply brakes;
* aircraft: land, disarm, deploy a parachute, or hand over to the autopilot's own failsafe — a
  decision for the platform owner, because a motor cut that is safe on the ground is catastrophic
  in the air;
* mechanism: remove drive enable / apply holding brake.

The core calls it exactly when the supervisor demands `EmergencyStop` and the vehicle is armed.

## 8. Validation evidence in this repository

Each row is an automated test that injects the fault and asserts the response.

| Scenario | Test |
|---|---|
| Arming refused without fresh sensors/link | `sys_prearm_rejected_without_fresh_sensors_or_link` |
| Mission completes and avoids static obstacle | `sys_mission_completes_and_avoids_static_obstacle` |
| Manual driving cannot hit an obstacle | `sys_manual_drive_cannot_hit_obstacle` |
| Manual input stops when sticks go stale | `sys_manual_timeout_stops_vehicle` |
| Link loss → hold → return home | `sys_link_loss_holds_then_returns_home` |
| Estimator loss → latched e-stop until disarm | `sys_estimator_loss_latches_emergency_stop_until_disarm` |
| Radar loss → degrade → hold | `sys_radar_loss_degrades_then_holds` |
| Geofence containment | `sys_geofence_contains_vehicle` |
| Disarm refused while moving | `sys_disarm_refused_while_moving` |
| Real threads + loopback transport | `runtime_threads_arm_and_stream_telemetry` |
| No heap allocation on the control path | `sys_control_path_never_allocates` |
| Supervisor state machine, hysteresis, latch | `supervisor_state_machine` |
| Codec rejects corruption / NaN / out-of-range | `codec_rejects_corruption_and_resyncs`, `codec_rejects_nan_and_out_of_range` |
| Avoidance reports "boxed in" | `avoidance_flags_no_safe_option_when_boxed_in` |

## 9. Known gaps (planned)

* Authenticated and encrypted command channel (CRC is not security).
* Battery/power, thermal, GNSS-quality and motor-health fault sources.
* Independent watchdog / second-processor supervisor.
* Dynamic-obstacle prediction beyond constant velocity; concave-obstacle escape.
* Formal hazard analysis (FMEA/FTA), requirements traceability and hardware-in-the-loop campaign.
