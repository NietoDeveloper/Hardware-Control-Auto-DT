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
   and let `SafetySupervisor` return a `SafetyVerdict` (state, action, fault bits, speed limit).
3. **Terminate if required.** `EmergencyStop` → call `IActuatorSink::emergencyStop()`, disarm,
   publish telemetry, return. Disarmed → command zero velocity, return.
4. **Decide intent.** Failsafe `ReturnHome` forces that mode. Then, by mode:
   `Manual` (sticks → world-frame velocity), `Mission` (`MissionExecutor::step`), `ReturnHome`
   (follow a straight line to home), `Hold` (zero). A `Hold` verdict zeroes the intent.
5. **Avoid.** If obstacles exist and the intent is non-zero, `CollisionAvoidance::resolve` returns a
   modified velocity. If it finds *no* safe candidate it sets `no_safe_option`, which feeds back as
   the `collision_imminent` input of the next cycle's verdict.
6. **Limit.** `CommandGuard::filter` applies vertical clamp → horizontal limit (min of platform
   limit and verdict limit) → geofence barrier → acceleration slew → yaw clamp.
7. **Steer.** Manual: sticks drive yaw rate. Otherwise a PID turns the vehicle to face its
   direction of travel when moving faster than `face_travel_min_speed_mps`.
8. **Act and report.** `IActuatorSink::apply(cmd)`; copy a `Telemetry` snapshot.

Properties of `step()`:

* **Single-threaded by contract.** Only the control thread calls it.
* **Deterministic.** No wall-clock reads; time is the `now` argument.
* **Bounded.** Fixed-capacity arrays everywhere; loop bounds are compile-time or config constants.
* **Allocation-free and non-throwing.** Verified by a test that fails on any `operator new` call.

## 5. Threading model

```
 comms thread  (200 Hz, prio 50)   control thread (100 Hz, prio 80)     media thread (best effort)
 ───────────────────────────────   ─────────────────────────────────    ──────────────────────────
 ITransport.receive                drain command queue                  poll IMediaSource(s)
 FrameDecoder / CRC / decode  ──►  VehicleController.handle(env)        MediaRouter.pump()
   CommandQueue (SPSC, 32)         VehicleController.step(now)            drop on backpressure
 pop telemetry queue          ◄──  push Telemetry every 100 ms           resync on keyframe
 encode + ITransport.send          (SPSC, 8; drop if full)
```

* **One writer per queue, wait-free.** `SpscQueue` uses acquire/release atomics and cache-line
  alignment; no mutexes appear on any path the control thread touches.
* **Back-pressure policy is "drop, never block".** A full command queue drops (and counts) commands;
  a full telemetry queue drops telemetry; a refusing media sink drops frames and requests a
  keyframe. The control thread is never the one waiting.
* **Absolute-deadline scheduling.** `PeriodicThread` sleeps until `start + n·period`, so jitter does
  not accumulate into drift; it counts overruns, worst execution time and worst wake-up lateness.
* **Real-time priority is optional and best-effort.** `RtConfig{priority, cpu}` requests
  `SCHED_FIFO` and CPU pinning on Linux (needs `CAP_SYS_NICE`); failure is logged, not fatal.
* **Shutdown order matters.** Media, then control (stop issuing setpoints), then comms.

## 6. Data ownership

| Data | Owner (only writer) | Readers |
|---|---|---|
| `VehicleState`, tracks, mission, mode, verdict | control thread | control thread (telemetry copy goes out by queue) |
| Decoder buffer, tx sequence | comms thread | comms thread |
| Media frames | media thread until handed to the sink | sink |
| Configuration | immutable after construction | everyone |

Because ownership is exclusive, the core contains no locks.

## 7. Error and fault handling philosophy

* **Reject at the edge.** The codec validates length, enums, ranges and finiteness before a command
  can exist as a C++ object.
* **Validate against state.** The controller refuses commands that make no sense *now* and counts
  them (`rejected_commands`).
* **Sensor loss is a first-class event.** State, radar and link each have an age, and each age maps
  to a defined degradation, not an undefined behaviour.
* **Latch the severe, hysteresis the mild.** Emergency stop latches until an explicit disarm;
  recoverable failsafes clear only after `recovery_time` of healthy conditions so they cannot
  oscillate.
* **Never forward garbage.** Non-finite values are replaced with zero before reaching hardware.

Details: [SAFETY.md](SAFETY.md).

## 8. Key design decisions and rationale

| Decision | Why |
|---|---|
| Velocity setpoint as the core's output | Works for aircraft, wheeled robots and mechanisms; inner loops stay with platform experts. |
| Injected clock instead of `steady_clock::now()` inside logic | Deterministic tests, replay, faster-than-real-time simulation. |
| Fixed-capacity containers (`std::array`) | No allocation, bounded time, trivially copyable → safe through lock-free queues. |
| Supervisor is a pure function of inputs | Exhaustively unit-testable; could later move to a separate process or processor. |
| Command guard *after* avoidance | Limits hold even if avoidance, mission or manual input is wrong. |
| Binary protocol with CRC and resync | Small, fast, works on lossy radios; one decoder for all transports. |
| Media isolated on its own thread and queue | Video encoders stall; flight/drive control must not. |
| Simulator implements the same ports | The binary logic under test is the logic that flies/drives. |
| Header-light, dependency-free | Easy to audit, vendor and cross-compile for embedded Linux targets. |

## 9. Extension points

| To add… | Do this |
|---|---|
| A new vehicle or mechanism | Implement `IStateSource`, `IRadarSource` (optional), `IActuatorSink`. |
| A new radio / network | Implement `ITransport`. |
| A WebSocket/gRPC bridge for web dashboards | Run it in the dashboard backend; it forwards the binary frames unchanged. |
| A new message | Add a `MsgType`, payload codec, `Command` variant alternative and `onCommand` overload. |
| A better planner | Replace `CollisionAvoidance` / `PathFollower`; keep the `Vec3 → Vec3` contract. |
| New fault sources | Add a bit in `safety::fault`, an input in `SafetyInputs`, a rule in `SafetySupervisor`. |

## 10. Testing strategy