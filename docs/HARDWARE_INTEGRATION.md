# Integrating real hardware

The core never talks to hardware directly. You write **adapters** that implement the ports in
`include/sdt/hal/ports.hpp` and `include/sdt/comms/transport.hpp`. The simulator in `sim/` is the
reference implementation of the same ports.

> The skeletons below are illustrative starting points, not shipped, tested drivers.

## 1. Port contracts

| Port | Method | Contract |. |

Rules for every adapter that runs on the control thread (`IStateSource`, `IRadarSource`,
`IActuatorSink`): **no allocation, no locks that another thread can hold for long, no blocking I/O,
no exceptions.** Do I/O on your own thread and hand data over with a latest-value slot or a
ard (v1 assumes that).

## 3. Platform recipes

### 3.1 Ground robot (differential drive)

```cpp
class DiffDriveActuator final : public sdt::IActuatorSink {
 public:
  void apply(const sdt::VelocityCommand& cmd) noexcept override {
    const double yaw = yaw_from_estimator_.load(std::memory_order_relaxed);
    const double v   = cmd.velocity.x * std::cos(yaw) + cmd.velocity.y * std::sin(yaw);  // forward
    const double w   = cmd.yaw_rate;
    left_.store ((v - w * track_ * 0.5) / wheel_r_, std::memory_order_relaxed);  // rad/s
    right_.store((v + w * track_ * 0.5) / wheel_r_, std::memory_order_relaxed);
    // a separate motor-driver thread reads left_/right_ and talks CAN / UART
  }
  void emergencyStop() noexcept override { estop_.store(true); /* driver thread cuts power + brakes */ }
 private:
  std::atomic<double> left_{0}, right_{0}, yaw_from_estimator_{0};
  std::atomic<bool> estop_{false};
  double track_{0.5}, wheel_r_{0.08};
};
```

Configure `VehicleConfig::kind = VehicleKind::Ground` (vertical velocity forced to zero, lateral
stick ignored).

### 3.2 Aerial vehicle (autopilot in the loop)

Keep the autopilot's attitude and motor control; use the core as the *outer* (navigation +
safety) loop.

* `IStateSource`: translate the autopilot's local-position/velocity/attitude stream to ENU.
* `IActuatorSink::apply`: send a velocity setpoint + yaw rate over the autopilot link
  (MAVLink offboard velocity setpoints, a vendor SDK, …). Send at a steady rate; most autopilots
  require a continuous setpoint stream to stay in offboard mode.
* `emergencyStop`: choose deliberately — command land, trigger the autopilot's own failsafe,
  disarm only if the aircraft is on the ground, or fire a parachute system. Document the decision.
* Configure `VehicleKind::Aerial`, the altitude band in `Geofence::min_z/max_z`, and tighten
  speed/accel limits to the airframe.

### 3.3 Mechanism (gantry, arm tip, pan-tilt, any velocity-controlled stage)

Treat the end effector as a point with a world-frame velocity.

* `IStateSource`: forward kinematics from encoders → position/velocity.
* `IActuatorSink::apply`: inverse kinematics / Jacobian → joint velocity targets to the servo
  controllers. Enforce joint limits *in the adapter* in addition to the core's geofence.
* `IRadarSource`: any proximity sensor can be adapted into `RadarScan` (range + azimuth/elevation).
* `emergencyStop`: remove drive enable and apply brakes (or hold torque if gravity loads the axis).

## 4. Transport adapters

```cpp
class UdpTransport final : public sdt::ITransport {
 public:
  std::size_t receive(std::uint8_t* buf, std::size_t cap) noexcept override {
    const ssize_t n = ::recv(fd_, buf, cap, MSG_DONTWAIT);   // fd_ is a non-blocking UDP socket
    return n > 0 ? static_cast<std::size_t>(n) : 0;
  }
  bool send(const std::uint8_t* d, std::size_t n) noexcept override {
    return ::sendto(fd_, d, n, MSG_DONTWAIT, peer_, peer_len_) == static_cast<ssize_t>(n);
  }
 private: int fd_{-1}; const sockaddr* peer_{nullptr}; socklen_t peer_len_{0};
};
```

The decoder tolerates arbitrary chunking, so stream (TCP, serial) and datagram (UDP) transports
both work unchanged. A web dashboard reaches the vehicle through its own backend, which relays the
binary frames; the core does not embed an HTTP/WebSocket server.

## 5. Wiring it together

```cpp
sdt::VehicleConfig cfg;
cfg.kind                   = sdt::VehicleKind::Aerial;
cfg.guard.max_speed_mps    = 6.0;
cfg.geofence.enabled       = true;           // fill vertices, min_z, max_z
cfg.safety.radar_required  = true;

MyEstimator est;  MyRadar radar;  MyActuator act;  MyUdpTransport link;
sdt::VehicleController ctl(cfg, sdt::VehiclePorts{est, radar, act});

sdt::SteadyClock clock;
sdt::RuntimeConfig rc;                       // control 100 Hz / comms 200 Hz by default
rc.control_rt = sdt::RtConfig{80, 2};        // SCHED_FIFO 80 pinned to CPU 2 (needs CAP_SYS_NICE)
rc.comms_rt   = sdt::RtConfig{50, 3};

sdt::MediaRouter media;                      // optional
media.addSource(my_camera);  media.setSink(my_rtp_sink);

sdt::VehicleRuntime rt(ctl, link, clock, rc, &media);
rt.start();
```

## 6. Bring-up checklist

1. **Simulator first.** Run `sil_demo`, then your own scenario in the simulator with your limits.
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
