# Roadmap

v1.0 fixes the architecture. Later versions harden and extend what sits inside the boundaries.
This is a plan, not a commitment; order follows risk reduction first, features second.

## v1.0 — Architecture baseline (this release)

* Ports for state, radar, actuation, media and transport.
* Deterministic, allocation-free `VehicleController` with manual, mission, return-home and hold.
* Radar tracking, velocity-sampling avoidance, layered safety, geofence.
* Binary protocol with CRC and resynchronisation.
* Three-thread runtime, simulator, 27 tests, benchmark, sanitizer build options.ial or mechanism target), with hardware-in-the-loop tests.
* Radar mounting-offset calibration.
* Configuration loading from file with validation (today configuration is in code).
* Structured event log / flight-data recorder with deterministic replay into the controller.

## v1.2 — Trust and health

* Authenticated (and optionally encrypted) command frames; replay protection.
* Battery/power, thermal, GNSS-quality and motor-health faults and failsafes.
* Operator-visible fault descriptions in telemetry; command acknowledgements.
* Length-negotiated protocol extensions.

## v2.0 — Smarter navigation

* Dynamic-obstacle prediction beyond constant velocity; non-spherical obstacle shapes.
* Local map memory and escape from concave obstacles; global planning between waypoints.
* Terrain/altitude-aware 3-D avoidance for aerial platforms.
* Multi-sensor fusion (radar + lidar/camera/ultrasonic) behind `IRadarSource`-style ports.
* Real media pipeline: hardware encoders, Opus audio, RTP/WebRTC sink, adaptive bitrate.

## v3.0 — Assurance

* Independent supervisor on a separate process/processor with its own watchdog.
* Formal hazard analysis (FMEA/FTA), requirements traceability, coverage targets (MC/DC where
  applicable).
* Static analysis and coding-standard compliance gates in CI.
* Redundant sensors/links and voting.
* Certification path evaluation for target markets.

## Cross-cutting, every release

* Keep the control path allocation-free and deterministic (the allocation test stays).
* Every new fault source ships with a fault-injection test.
* Protocol changes bump the version and update `docs/PROTOCOL.md`.
* Benchmarks tracked per release on target hardware.
