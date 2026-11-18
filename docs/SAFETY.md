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

* A failsafe action is held until all tr
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

The core calls it exactly when the supervisor demands `EmergencyStop` 