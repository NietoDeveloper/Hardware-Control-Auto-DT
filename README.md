# sdt-vehicle-core

**Real-time vehicle & mechanism control core — a SoftwareDT.com product.**
Version 1.0.0 · C++20 · no third-party dependencies · Linux / macOS

`sdt-vehicle-core` is a backend-only C++ library that sits between a **control dashboard**
(or any other backend/frontend in the Software DT ecosystem) and **physical hardware**: a drone, a
ground robot, or any velocity-controlled mechanism. It accepts operator intent — manudriving
or a programmed route — and turns it into safe, bounded, real-time actuator commands, while
watching a radar for obles so the vehicle nei─────────────────────────────┘
 ┌──────┴──────────────────────────
## Building and tes
Requirements: a C++20 compiler (GCC 11+ or Clang 14+; developed and tested with GCC 13), CMake ≥ 3.20. Nothing ele.

ctest --test-dir build --output-on-failure`ON`) |
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

## Quality evidence

All results below were produced by this repository's own test suite and apps.

* **27 automated tests, 0 failures** — 16 unit (queue, codec, PID, guard, geofence, tracker,
  avoidance, mission, supervisor, media routd 11 system/runtime (pre-arm refusal, mission with
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
* Single vehicle per process; home point is