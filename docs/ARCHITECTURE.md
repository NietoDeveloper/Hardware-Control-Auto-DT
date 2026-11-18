# Architecture

This document explains how `sdt-vehicle-core` is structured, how data moves through it, how
threads interact, and — most importantly — **why** it is built this way.

## 1. Goals and non-goals

**Goals**

* Control air, ground or mechani
* Be one backend among many: it talks to other Software DT backends and frontenrough a
  small, stable, transport-independent poutputs a *velocity setpoint* and leaves inner loops
  to the platform (autopilot, motor controller, joint servos).
* State estimation / SLAM — the core consumes a fused `Ve─────────────────────────────────────────────────────────────┐(router)                               │  edges
├─────────────────────────────────eads
or vendor code. The only code allowed to know about the physical world is an adapter implementing a
port.

## 3. Modules

| Module | Responsibility | Key types |
|---|---|---|
| `common/` | Units, time, logging, wait-free queue | `Vec3`, `VehicleState`, `VelocityCommand`, `IClock`/`SteadyClock`/`ManualClock`, `SpscQueue` |
| `hal/` | Everything physical, as interfaces | `IStateSource`, `IRadarSource`, `IActuatorSink`, `IMediaSource`, `IMediaSink` |
| `perception/` | Radar detections → stable obstacle tracks | `RadarTracker`, `Obstacle` (max 64 tracks) |
| `avoidance/` | Desired velocity → safe velocity | `CollisionAvoidance`, `AvoidanceResult` |
| `nav/` | Missions and path following | `Mission` (max 128 waypoints), `MissionExecutor`, `PathFollower` |
| `safety/` | Health → verdict; last-line limiting | `SafetySupervisor`, `CommandGuard`, `Geofence` |
| `control/` | Generic control primitives | `Pid` (anti-windup, output clamp) |
| `comms/` | Dashboard edge | `proto::` codec, `FrameDecoder`, `CommandGateway`, `ITransport`, `LoopbackTransport` |
| `media/` | Video/audio path | `MediaRouter` |
| `runtime/` | Wiring and threads | `VehicleController`, `VehicleRuntime`, `PeriodicThread` |
| `sim/` | Software-in-the-loop adapters | `SimVehicle`, `SimRadar`, `SimObstacle` |

### Coordinate frames and units

* Local **ENU** frame: x = East, y = North, z = Up. SI units everywhere (m, m/s, rad, s).
* Yaw is counter-clockwise from East. Time is monotonic nanoseconds (`TimeNs`).
* `VelocityCommand` is expressed in the **world frame**; platform adapters convert it to whatever
  their hardware needs (body-frame velocity, wheel speeds, joint rates…).
* Radar detections are in the **sensor frame**; the tracker transforms them using the vehicle
  state. In v1 the radar boresight is assumed to be the vehicle's forward axis.

## 4. The control cycle

`VehicleController::step(now)` is the heart of the system. One call = one cycle, in this exact order:

1. **Sense.** Poll `IStateSource` (reject non-finite estimates) and drain at most two radar scans
   from `IRadarSource` into `RadarTracker` (bounded work). Confirmed tracks are copied into a
   fixed array.
2. **Judge.** Build `SafetyInputs` (link age, state age, radar age, geofence margin, collision flag)
