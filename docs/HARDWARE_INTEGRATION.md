# Integrating real hardware

The core never talks to hardware directly. You write **adapters** that implement the ports in
`include/sdt/hal/ports.hpp` and `include/sdt/comms/transport.hpp`. The simulator in `sim/` is the
reference implementation of the same ports.

> The skeletons below are illustrative starting points, not shipped, tested drivers.nd::Aerial;
cfg.guard.max_speed_mps    = 6.0;
cfg.geofence.enabled       = true;           // fill vertices, min_z, max_z
6. **Fault-inject on hardware:** pull the link, unplug the radar, freeze the estimator — confirm
   the documented response for each (SAFETY.md §4).
7. **Tighten limits** to the platform, then expand gradually.
8. **Secure the link** (VPN/TLS tunnel) before exposing it beyond a bench network.
