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

* Independent 