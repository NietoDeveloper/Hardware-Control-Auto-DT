// System-level software-in-the-loop tests: real controller + simulated vehicle/radar/obstacles.
#include <atomic>
#include <cstdlib>
#include <new>
#include <chrono>
#include <thread>

#include "sdt/comms/codec.hpp"
#include "sdt/runtime/vehicle_controller.hpp"
#include "sdt/runtime/vehicle_runtime.hpp"
#include "sdt/sim/sim_world.hpp"
#include "test_framework.hpp"

using namespace sdt;

namespace {
struct Rig {
  ManualClock clock;
  sim::SimVehicle veh;
  std::vector<sim::SimObstacle> obs;
  sim::SimRadar radar;
  VehicleController ctl;
  double min_clearance{kInf};

  explicit Rig(const VehicleConfig& cfg = VehicleConfig{})
      : veh(VehicleState{}), radar(veh, obs, clock), ctl(cfg, VehiclePorts{veh, radar, veh}) {}

  void send(const Command& c) { ctl.handle(CommandEnvelope{clock.now(), c}); }

  void run(double seconds, bool heartbeat = true) {
    const int n = static_cast<int>(seconds / 0.01);
    for (int i = 0; i < n; ++i) {
      if (heartbeat && i % 10 == 0) send(CmdHeartbeat{});
      ctl.step(clock.now());
      for (auto& o : obs) o.position += o.velocity * 0.01;
      veh.step(clock.now(), 0.01);
      for (const auto& o : obs) {
        min_clearance = std::min(min_clearance, (o.position - veh.truth().position).norm() - o.radius_m);
      }
      clock.advance(millis(10));
    }
  }

  void armAndWait() {
    run(0.6);
    send(CmdArm{true});
    run(0.1);
  }
};

Mission line(double x, double speed = 4.0) {
  Mission m;
  m.id = 1;
  m.count = 1;
  m.waypoints[0] = {{x, 0.0, 0.0}, static_cast<float>(speed), 1.5f, 0.0f};
  return m;
}
}  // namespace

SDT_TEST(sys_prearm_rejected_without_fresh_sensors_or_link) {
  Rig r;
  r.send(CmdArm{true});  // nothing has run yet: no state, no radar
  CHECK(!r.ctl.armed());
  r.run(0.01);
  CHECK(r.ctl.telemetry().rejected_commands == 1);
  r.armAndWait();
  CHECK(r.ctl.armed());
}

SDT_TEST(sys_mission_completes_and_avoids_static_obstacle) {
  Rig r;
  r.obs.push_back({{25.0, 0.2, 0.0}, {0, 0, 0}, 1.5});
  r.send(CmdMissionUpload{line(50.0)});
  r.armAndWait();
  r.send(CmdMissionControl{MissionAction::Start});
  r.run(40.0);
  CHECK(r.ctl.missionState() == MissionState::Completed);
  CHECK(r.min_clearance > 0.5);
  CHECK((r.veh.truth().position - Vec3{50, 0, 0}).norm() < 2.0);
}

SDT_TEST(sys_manual_drive_cannot_hit_obstacle) {
  Rig r;
  r.obs.push_back({{20.0, 0.0, 0.0}, {0, 0, 0}, 1.5});
  r.armAndWait();
  r.send(CmdSetMode{ControlMode::Manual});
  for (int i = 0; i < 2000; ++i) {  // operator holds full forward for 20 s
    r.send(CmdManual{1.0f, 0.f, 0.f, 0.f});
    r.run(0.01);
  }
  CHECK(r.min_clearance > 0.0);
}

SDT_TEST(sys_manual_timeout_stops_vehicle) {
  Rig r;
  r.armAndWait();
  r.send(CmdSetMode{ControlMode::Manual});
  r.send(CmdManual{1.0f, 0.f, 0.f, 0.f});
  r.run(0.4);
  CHECK(r.veh.truth().velocity.norm() > 0.8);
  r.run(3.0);  // heartbeats continue, sticks go stale (> 500 ms)
  CHECK(r.veh.truth().velocity.norm() < 0.05);
}

SDT_TEST(sys_link_loss_holds_then_returns_home) {
  Rig r;
  r.send(CmdMissionUpload{line(60.0)});
  r.armAndWait();
  r.send(CmdMissionControl{MissionAction::Start});
  r.run(8.0);
  const double x_before = r.veh.truth().position.x;
  CHECK(x_before > 10.0);
  r.run(4.0, /*heartbeat=*/false);  // link silent > 2 s
  CHECK(r.ctl.verdict().action == FailsafeAction::Hold);
  CHECK(r.veh.truth().velocity.norm() < 0.05);
  r.run(20.0, false);  // > 10 s of silence: RTH, arrive, hold
  CHECK(r.ctl.mode() == ControlMode::Hold);  // arrived home and holds
  CHECK(r.veh.truth().position.norm2d() < 3.0);
}

SDT_TEST(sys_estimator_loss_latches_emergency_stop_until_disarm) {
  Rig r;
  r.send(CmdMissionUpload{line(60.0)});
  r.armAndWait();
  r.send(CmdMissionControl{MissionAction::Start});
  r.run(5.0);
  r.veh.freezeEstimate(true);
  r.run(0.5);
  CHECK(r.veh.estopped());
  CHECK(!r.ctl.armed());
  CHECK(r.ctl.verdict().state == SafetyState::EmergencyStop);
  r.veh.freezeEstimate(false);
  r.run(0.5);
  r.send(CmdArm{true});  // latched: must be rejected
  CHECK(!r.ctl.armed());
  r.send(CmdArm{false});  // explicit operator acknowledge clears the latch
  r.run(0.1);
  CHECK(r.ctl.verdict().state != SafetyState::EmergencyStop);
}

SDT_TEST(sys_radar_loss_degrades_then_holds) {
  Rig r;
  r.send(CmdMissionUpload{line(200.0)});
  r.armAndWait();
  r.send(CmdMissionControl{MissionAction::Start});
  r.run(6.0);
  r.radar.setEnabled(false);
  r.run(1.5);
  CHECK(r.ctl.verdict().state == SafetyState::Degraded);
  CHECK(r.veh.truth().velocity.norm() <= 1.6);
  r.run(3.0);
  CHECK(r.ctl.verdict().action == FailsafeAction::Hold);
  CHECK(r.veh.truth().velocity.norm() < 0.05);
}

SDT_TEST(sys_geofence_contains_vehicle) {
  VehicleConfig cfg;
  cfg.geofence.enabled = true;
  cfg.geofence.vertex_count = 4;
  cfg.geofence.vertices[0] = {-10, -10, 0};
  cfg.geofence.vertices[1] = {30, -10, 0};
  cfg.geofence.vertices[2] = {30, 10, 0};
  cfg.geofence.vertices[3] = {-10, 10, 0};
  Rig r(cfg);
  r.send(CmdMissionUpload{line(100.0)});  // waypoint far outside the fence
  r.armAndWait();
  r.send(CmdMissionControl{MissionAction::Start});
  r.run(30.0);
  CHECK(r.veh.truth().position.x < 30.0);
  CHECK(r.veh.truth().position.x > 20.0);  // got as close as the barrier allows
}

SDT_TEST(sys_disarm_refused_while_moving) {
  Rig r;
  r.send(CmdMissionUpload{line(100.0)});
  r.armAndWait();
  r.send(CmdMissionControl{MissionAction::Start});
  r.run(5.0);
  r.send(CmdArm{false});
  CHECK(r.ctl.armed());
}

// ---- threaded runtime (exercised under TSan in CI) ----------------------------------------------
namespace {
struct StubPorts final : IStateSource, IRadarSource, IActuatorSink {
  std::atomic<int> applies{0};
  SteadyClock clock;
  bool poll(VehicleState& s) noexcept override {
    s = VehicleState{};
    s.stamp = clock.now();
    return true;
  }
  bool poll(RadarScan&) noexcept override { return false; }
  void apply(const VelocityCommand&) noexcept override { applies.fetch_add(1); }
  void emergencyStop() noexcept override {}
};
}  // namespace

SDT_TEST(runtime_threads_arm_and_stream_telemetry) {
  StubPorts ports;
  VehicleConfig cfg;
  cfg.safety.radar_required = false;
  VehicleController ctl(cfg, VehiclePorts{ports, ports, ports});
  auto [dash, board] = LoopbackTransport::makePair();
  SteadyClock clock;
  RuntimeConfig rc;
  rc.control_rt = RtConfig{0, -1};  // no SCHED_FIFO in CI containers
  rc.comms_rt = RtConfig{0, -1};
  VehicleRuntime rt(ctl, *board, clock, rc);
  CHECK(rt.start());

  auto send = [&](const Command& c) {
    std::uint8_t buf[proto::kMaxFrame];
    const std::size_t n = proto::encodeCommand(c, 0, buf, sizeof buf);
    dash->send(buf, n);
  };
  proto::FrameDecoder dec;
  bool armed_seen = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  int tick = 0;
  while (!armed_seen && std::chrono::steady_clock::now() < deadline) {
    send(CmdHeartbeat{});
    if (++tick > 5) send(CmdArm{true});
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::uint8_t buf[512];
    const std::size_t n = dash->receive(buf, sizeof buf);
    dec.feed(buf, n, [&](const proto::Frame& f) {
      Telemetry t;
      if (f.type == proto::MsgType::Telemetry && proto::decodeTelemetry(f.payload, f.length, t) && t.armed) {
        armed_seen = true;
      }
    });
  }
  rt.stop();
  CHECK(armed_seen);
  CHECK(rt.controlStats().cycles > 10);
  CHECK(ports.applies.load() > 10);
}

// ---- zero-allocation guarantee on the control path ------------------------------------------------
// The global operator new is replaced for the whole test binary so that any heap allocation made
// while the flag is set is counted. VehicleController::handle()/step() must never allocate.
namespace {
std::atomic<bool> g_count_allocs{false};
std::atomic<long> g_allocs{0};
}  // namespace

void* operator new(std::size_t n) {
  if (g_count_allocs.load(std::memory_order_relaxed)) g_allocs.fetch_add(1, std::memory_order_relaxed);
  if (void* p = std::malloc(n)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

SDT_TEST(sys_control_path_never_allocates) {
  Rig r;
  for (int i = 0; i < 12; ++i) r.obs.push_back({{14.0 + 4.0 * i, -3.0 + 1.1 * i, 0.0}, {-0.3, 0.0, 0.0}, 1.0});
  r.send(CmdMissionUpload{line(300.0)});
  r.armAndWait();
  r.send(CmdMissionControl{MissionAction::Start});

  g_allocs.store(0);
  long counted = 0;
  for (int i = 0; i < 3000; ++i) {  // 30 s of flight through an obstacle field
    g_count_allocs.store(true);
    if (i % 10 == 0) r.ctl.handle(CommandEnvelope{r.clock.now(), CmdHeartbeat{}});
    if (i == 1500) r.ctl.handle(CommandEnvelope{r.clock.now(), CmdMissionControl{MissionAction::Pause}});
    if (i == 1600) r.ctl.handle(CommandEnvelope{r.clock.now(), CmdMissionControl{MissionAction::Resume}});
    r.ctl.step(r.clock.now());
    g_count_allocs.store(false);
    for (auto& o : r.obs) o.position += o.velocity * 0.01;
    r.veh.step(r.clock.now(), 0.01);
    r.clock.advance(millis(10));
  }
  counted = g_allocs.load();
  CHECK(counted == 0);
  CHECK(r.ctl.obstacleCount() > 0);
}
