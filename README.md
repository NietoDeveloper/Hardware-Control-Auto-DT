# sdt-vehicle-core

**Real-time vehicle & mechanism control core — a SoftwareDT.com product.**
Version 1.0.0 · C++20 · no third-party dependencies · Linux / macOS

`sdt-vehicle-core` is a backend-only C++ library that sits between a **control dashboard**
(or any other backend/frontend in the Software DT ecosystem) and **physical hardware**: a drone, a
ground robot, or any velocity-controlled mechanism. It accepts operator intent — manual driving
or a programmed route — and turns it into safe, bounded, real-time actuator commands, while
watching a radar for obles so the vehicle neither collides nor leaves its route unnecessarily.

> **Version 1 is about architectureThe module boundaries, threading model, safet
> wire protocol and test strategy aredeliverable. Individual algorithms are deliberaking those boundaries. See
> [docs/ROADMAP.md](docs/ROADM
## Table of con

1. [What it does](#what-it-does)
2. [Architecture at a glance](#architecture-at-ae)
3. [Design principles](#design-principles)
4. [Repository layout](#repository-layout)
5. [Building and testing](#building-and-testing)
6. [Quick start](#quick-start)
7. [Control modes](#control-modes)
8. [Safety model](#safety-mo
9. [Dashboard protocol](#dashboard-protocol)
10. [Integrating real hardware](#integrating-real-hardware)
11. [Quality evidence](#quality-evidence)
12. [Known limitations of v1](#known-limitations-of-v1)
13. [Documentation index](#documentation-index)
14. [License](#license)

---

## What it does

| Capability | Where | Status in v1 |
|---|---|---|
| Manual (live) control from any dashboard | `VehicleController`, mode `Manual` | Implemented |
| Programmed route (waypoint mission, pause/resume/abort, optional loop, dwell) | `nav/` | Implemented |
| Radar obstacle detection and multi-target tracking | `perception/` | Implemented |
| Collision avoidance (re-plans velocity around obstacles; also assists manual driving) | `avoidance/` | Implemented |
| Independent safety supervisor (link loss, sensor loss, geofence, imminent collision) | `safety/` | Implemented |
| Last-line command guard (speed/accel limits, geofence barrier, NaN rejection) | `safety/command_guard` | Implemented |
| Return-to-home and hold failsafes | `VehicleController` | Implemented |
| Binary, CRC-protected, resynchronising dashboard protocol | `comms/` | Implemented |
| Transport abstraction (UDP / TCP / serial / WebSocket bridge plug in here) | `comms/transport.hpp` | Port + in-process loopback |
| Video/audio routing, isolated from the control loop | `media/` | Router + backpressure logic; no encoders yet |
| Multi-threaded real-time runtime (control 100 Hz, comms 200 Hz, media) | `runtime/` | Implemented |
| Software-in-the-loop simulator (kinematic vehicle, noisy radar, moving obstacles) | `sim/` | Implemented |
| Air, ground and generic mechanism platforms | `VehicleKind` + HAL | Ground and Aerial kinds; mechanisms integrate through the same velocity port |

---

## Architecture at a glance

```
   Dashboards / other Software DT backends and frontends
                         │  binary frames (CRC-32, versioned)
                         ▼
 ┌──────────────────────────────────────────────────────────────────────────┐
 │ comms thread (200 Hz)       CommandGateway ⇄ ITransport                  │
 └───────────────┬───────────────────────────────▲──────────────────────────┘
        lock-free SPSC queue                lock-free SPSC queue
          (commands)                            (telemetry)
 ┌───────────────▼───────────────────────────────┴──────────────────────────┐
 │ control thread (100 Hz, SCHED_FIFO optional)    VehicleController::step()│
 │                                                                          │
 │  poll state ─ poll radar ─► RadarTracker ─► obstacles                    │
 │        │                                         │                       │
 │        ▼                                         ▼                       │
 │  SafetySupervisor (verdict)   intent: Manual │ Mission │ RTH │ Hold      │
 │        │                                         │                       │
 │        └──────────────► CollisionAvoidance ◄─────┘                       │
 │                               │                                          │
 │                      CommandGuard (limits · geofence · slew)             │
 │                               │                                          │
 │                      heading control ─► IActuatorSink::apply()           │
 └──────────────────────────────────────────────────────────────────────────┘
        ▲ IStateSource            ▲ IRadarSource            ▼ IActuatorSink
 ┌──────┴───────────────────────────┴───────────────────────────┴───────────┐
 │ HAL adapters: simulator today · autopilot / CAN / serial / GPIO tomorrow │
 └──────────────────────────────────────────────────────────────────────────┘

 media thread (best effort):  IMediaSource ─► MediaRouter ─► IMediaSink
                              (never shares a lock or queue with the control loop)
```

The whole story, with data-flow and timing rationale, is in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

---

## Design principles

1. **Hardware is behind ports.** The core never includes a vendor SDK. Everything physical is an
   interface in `hal/ports.hpp` (state, radar, actuators, media) or `comms/transport.hpp`.
2. **The control step is deterministic.** `VehicleController::step(now)` is a function of its
   previous state, its polled inputs and an *injected* timestamp. Tests drive it with a manual
   clock, faster than real time and bit-for-bit repeatable.
3. **No allocation, no blocking, no exceptions on the control path.** Enforced by a test that
   replaces global `operator new` and fails if the control cycle allocates.
4. **Threads are isolated by wait-free queues.** Only the control thread ever touches vehicle
   state. A slow network, encoder or dashboard cannot delay a control cycle.
5. **Defence in depth.** Intent → avoidance → safety verdict → command guard → actuator. Each layer
   assumes the one before it is wrong.
6. **Fail safe, and make failure visible.** Every fault is a bit in telemetry; every rejected
   command is counted; every protocol error is counted.
7. **Strict at the edges.** The decoder rejects wrong lengths, unknown enums, out-of-range sticks
   and non-finite floats, and resynchronises after garbage or bit errors.
8. **Extensible without rewrites.** New vehicle = new HAL adapter. New radio = new `ITransport`.
   New planner = swap a module behind the same data types.

---

## Repository layout

```
sdt-vehicle-core/
├── CMakeLists.txt
├── README.md
├── docs/
│   ├── ARCHITECTURE.md          layers, data flow, threading, timing, design decisions
│   ├── SAFETY.md                fault model, failsafe matrix, what is and is not claimed
│   ├── PROTOCOL.md              binary wire protocol v1 (frames, payloads, enums)
│   ├── HARDWARE_INTEGRATION.md  how to write adapters for a drone, robot or mechanism
│   └── ROADMAP.md               v1 → v2 → v3 plan
├── include/sdt/
│   ├── common/    types, injected clock, logger, wait-free SPSC queue
│   ├── hal/       ports: IStateSource, IRadarSource, IActuatorSink, IMediaSource/Sink
│   ├── perception/ radar clustering + multi-target alpha-beta tracker (fixed capacity)
│   ├── avoidance/ velocity-sampling collision avoidance (discretised velocity obstacles)
│   ├── nav/       mission data, mission state machine, carrot-chasing path follower
│   ├── safety/    safety supervisor, command guard, polygonal geofence
│   ├── control/   PID with anti-windup
│   ├── comms/     wire codec, transport port, command gateway, message model
│   ├── media/     media router (drop-on-backpressure, keyframe resync)
│   ├── runtime/   deterministic VehicleController, periodic RT threads, VehicleRuntime
│   └── sim/       software-in-the-loop vehicle, radar and obstacles
├── src/           implementations (mirrors include/)
├── apps/
│   ├── sil_demo/    end-to-end simulated mission and link-loss scenarios
│   └── bench_step/  latency histogram of one control cycle
├── tests/         27 unit, system and runtime tests (dependency-free test framework)
└── .github/workflows/ci.yml
```

---

## Building and testing

Requirements: a C++20 compiler (GCC 11+ or Clang 14+; developed and tested with GCC 13), CMake ≥ 3.20. Nothing else.

```bash
cmake -S . -B build -DSDT_WERROR=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

| CMake option | Default | Meaning |
|---|---|---|
| `SDT_BUILD_TESTS` | `ON` | Build `sdt_tests` and register CTest cases |
| `SDT_BUILD_APPS` | `ON` | Build `sil_demo` and `bench_step` |
| `SDT_WERROR` | `OFF` | Treat warnings as errors (CI uses `ON`) |
| `SDT_SANITIZER` | *(empty)* | e.g. `address,undefined` or `thread` |

The warning set is strict (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
-Wold-style-cast -Wnull-dereference …`).

Sanitizer builds:

```bash
cmake -S . -B build-asan -DSDT_SANITIZER=address,undefined && cmake --build build-asan -j && ctest --test-dir build-asan
cmake -S . -B build-tsan -DSDT_SANITIZER=thread            && cmake --build build-tsan -j && ctest --test-dir build-tsan
```

---

## Quick start

### Run the simulated mission

```bash
./build/sil_demo nominal      # waypoint mission through a field of moving obstacles
./build/sil_demo link-loss    # dashboard disappears mid-mission → hold → return home → hold
./build/bench_step            # control-cycle latency histogram
```

### Embed the core

```cpp
#include "sdt/runtime/vehicle_controller.hpp"
#include "sdt/runtime/vehicle_runtime.hpp"
#include "sdt/sim/sim_world.hpp"          // swap for your hardware adapters

using namespace sdt;

VehicleConfig cfg;                         // every limit, timeout and gain lives here
cfg.kind = VehicleKind::Aerial;

MyState    state;                          // implements IStateSource
MyRadar    radar;                          // implements IRadarSource
MyActuator actuator;                       // implements IActuatorSink
VehicleController controller(cfg, VehiclePorts{state, radar, actuator});

MyUdpTransport link;                       // implements ITransport
SteadyClock clock;
VehicleRuntime runtime(controller, link, clock);   // control + comms (+ optional media) threads
runtime.start();                           // stop() or destructor shuts down cleanly
```

For deterministic tests, skip `VehicleRuntime` and call `controller.handle(envelope)` and
`controller.step(now)` yourself with a `ManualClock`.

---

## Control modes

| Mode | Intent source | Notes |
|---|---|---|
| `Hold` | none | Zero velocity. Default on boot, after disarm, after any hold-failsafe. |
| `Manual` | Normalised sticks `[-1, 1]` (forward, lateral, vertical, yaw) | Stale sticks (> 500 ms) command zero velocity. Avoidance and geofence still apply. |
| `Mission` | Waypoint list uploaded by the dashboard | Start / pause / resume / abort; per-waypoint speed, acceptance radius and dwell; optional loop. |
| `ReturnHome` | Autonomous | Entered by failsafe or on command. Home = position at arming. |

Commands are validated against state: you cannot start a mission that is not loaded, switch out of
`Hold` while disarmed, or disarm a moving vehicle. Rejections are counted in telemetry
(`rejected_commands`) rather than silently ignored.

---

## Safety model

Four independent layers, summarised here and specified in [docs/SAFETY.md](docs/SAFETY.md):

1. **Collision avoidance** reshapes the desired velocity using tracked radar obstacles. If *no*
   candidate velocity is safe, it raises `no_safe_option`, which the supervisor turns into a hold.
2. **Safety supervisor** (pure state machine, no side effects) turns health inputs into a verdict:
   `Nominal → Degraded → Failsafe → EmergencyStop`, with hysteresis so failsafes do not flap.
3. **Command guard** applies hard limits to whatever the layers above produced: vertical clamp,
   horizontal speed limit, geofence barrier, acceleration slew, yaw-rate clamp, non-finite rejection.
4. **Actuator port** provides a vehicle-specific `emergencyStop()` that must be idempotent and
   non-blocking (ground: cut drive and brake; aerial: land / disarm / parachute — your adapter decides).

| Fault | Response |
|---|---|
| Link silent > 2 s | Hold |
| Link silent > 10 s | Return home |
| State estimate lost > 250 ms | **Latched emergency stop** (cleared only by an explicit operator disarm) |
| Radar silent > 500 ms | Speed cap (degraded) |
| Radar silent > 2.5 s | Hold |
| Outside geofence | Hold; barrier blocks motion that would exit further |
| No collision-free velocity | Hold |

All thresholds are configuration, not constants.

---

## Dashboard protocol

Compact binary frames, little-endian, CRC-32 protected, versioned:

```
magic(2: D7 7A) | version(1) | type(1) | seq(2) | length(2) | payload(0..4096) | crc32(4)
```

Message types: Heartbeat, Arm, SetMode, ManualControl, MissionUpload, MissionControl (dashboard →
vehicle) and Telemetry (vehicle → dashboard, 10 Hz by default). The decoder is incremental,
allocation-free and re-synchronises on the magic bytes after corruption. Full byte layouts:
[docs/PROTOCOL.md](docs/PROTOCOL.md).

Because the protocol is transport-independent, the same bytes travel over UDP, TCP, a serial/LoRa
radio, or a thin WebSocket/gRPC bridge owned by the dashboard backend.

---

## Integrating real hardware

Implement the ports; nothing in the core changes.

| Port | You provide | Typical backing |
|---|---|---|
| `IStateSource::poll` | Fused position / velocity / yaw | GNSS+IMU estimator, autopilot telemetry, wheel odometry + SLAM, encoders |
| `IRadarSource::poll` | Radar scans in sensor frame | Vendor radar SDK / CAN / UART |
| `IActuatorSink::apply` / `emergencyStop` | Map a world-frame velocity + yaw-rate to the platform | Autopilot velocity setpoints, differential-drive mixer, joint-space velocity controller |
| `ITransport` | Non-blocking byte pipe | UDP, TCP, serial, radio modem |
| `IMediaSource` / `IMediaSink` | Encoded video/audio frames in, network out | Hardware H.264/H.265 encoder, Opus, RTP/WebRTC |

Step-by-step guidance with skeletons for a ground robot, a drone and a mechanism:
[docs/HARDWARE_INTEGRATION.md](docs/HARDWARE_INTEGRATION.md).

---

## Quality evidence

All results below were produced by this repository's own test suite and apps.

* **27 automated tests, 0 failures** — 16 unit (queue, codec, PID, guard, geofence, tracker,
  avoidance, mission, supervisor, media router) and 11 system/runtime (pre-arm refusal, mission with
  obstacle avoidance, manual-drive collision protection, manual timeout, link-loss hold → return
  home, estimator-loss latched e-stop, radar loss, geofence containment, disarm refusal while
  moving, real multi-threaded runtime streaming telemetry, **zero allocations on the control path**).
* Builds warning-free with `-Werror` under the strict warning set above. The full suite also
  passes clean under **ASan + UBSan** and under **TSan** (no reports).
* `sil_demo nominal`: mission completes with **2.18 m minimum clearance** to the nearest obstacle
  surface and 0 rejected commands.
* `sil_demo link-loss`: after the dashboard disappears the vehicle holds, returns, and ends in `Hold` (safety state `Failsafe`) 1.2 m from home.
* `bench_step` (200 000 cycles, 19 confirmed tracks, general-purpose Linux container, no
  real-time scheduling): **p50 7.8 µs · p99 22.3 µs · p99.9 41.7 µs** against a 10 000 µs budget.
  The 2.06 ms worst case is an OS scheduling artefact of the non-real-time environment; on a
  `SCHED_FIFO`, pinned thread this tail is expected to shrink and must be re-measured on target hardware.

---

## Known limitations of v1

Stated plainly so nobody mistakes an architecture release for a certified product:

* **No real hardware drivers yet.** Only simulator adapters and an in-process loopback transport
  ship in v1. The ports are the product; the adapters come with the first target platform.
* **Not certified, not formally verified.** The engineering practices follow aerospace and
  safety-critical habits (determinism, static allocation, layered safety, strict input validation,
  exhaustive fault-injection tests), but v1 carries no DO-178C / ISO 26262 / IEC 61508 claim.
* **The wire protocol is integrity-checked, not authenticated or encrypted.** CRC-32 catches
  corruption, not an attacker. Authenticated frames are a v2 item; until then put the link on a
  trusted network or tunnel.
* **Radar boresight is the vehicle's forward axis** (no mounting-offset calibration yet).
* **Avoidance and tracking are first-generation:** constant-velocity targets, spherical obstacles,
  no map memory, no global replanning around concave obstacles.
* **No battery/power, temperature or motor-health faults** yet (the fault word has room for them).
* Single vehicle per process; home point is the position at arming.

The prioritised fix list is in [docs/ROADMAP.md](docs/ROADMAP.md).

---

## Documentation index

| Document | Read it to learn… |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | How the system is structured and why |
| [docs/SAFETY.md](docs/SAFETY.md) | Fault model, failsafe matrix, safety claims and non-claims |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | Exact bytes on the wire |
| [docs/HARDWARE_INTEGRATION.md](docs/HARDWARE_INTEGRATION.md) | How to connect a real platform |
| [docs/ROADMAP.md](docs/ROADMAP.md) | What comes after v1 |

---

## License

Proprietary — © SoftwareDT.com. All rights reserved. (Replace this section if the project is
later released under an open-source license.)
