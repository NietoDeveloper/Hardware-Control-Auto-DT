# Integrating real hardware

The core never talks to hardware directly. You write **adapters** that implement the ports in
`include/sdt/hal/ports.hpp` and `include/sdt/comms/transport.hpp`. The simulator in `sim/` is the
reference implementation of the same ports.

> The skeletons below are illustrative starting points, not shipped, tested drivers.nd::Aerial;
cfg.guard.max_speed_mps    = 6.0;
cfg.geofence.enabled       = true;           // fill vertices, min_z, max_z
cfg.safety.radar_required  = true;
rc.control_rt = sdt::RtConfig{80, 2};        // SCHED_FIFO 80 pinned to CPU 2 (needs CAP_SYS_NICE)
2. **Adapters on the bench, actuators disabled.** Log what `apply()` would send; verify signs,
   units, frame rotations and update rates against a trusted reference.
3. **Check timestamps.** State and radar must be fresh (< 250 ms / 500 ms) or arming is refused.
4. **Prove the emergency stop** on its own, with the vehicle restrained, before anything else.
5. **Measure timing on target:** use `runtime.controlStats()` (`overruns`, `max_exec_ns`,
   `max_lateness_ns`) and `bench_step`; configure real-time priority and CPU pinning.
6. **Fault-inject on hardware:** pull the link, unplug the radar, freeze the estimator — confirm
   the documented response for each (SAFETY.md §4).
7. **Tighten limits** to the platform, then expand gradually.
8. **Secure the link** (VPN/TLS tunnel) before exposing it beyond a bench network.
