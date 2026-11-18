// Software-in-the-loop demo. A simulated dashboard talks to the vehicle core over the real wire
// protocol (loopback transport), uploads a mission, arms, and watches the vehicle fly it while
// avoiding a static and a moving obstacle. Exit code 0 only if the mission completed with no
// collision, so the demo doubles as a CI smoke test.
//
//   sil_demo [nominal|link-loss]
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "sdt/comms/codec.hpp"
#include "sdt/comms/command_gateway.hpp"
#include "sdt/runtime/vehicle_controller.hpp"
#include "sdt/sim/sim_world.hpp"

using namespace sdt;

namespace {
void sendCmd(ITransport& t, const Command& c, std::uint16_t& seq) {
  std::uint8_t buf[proto::kMaxFrame];
  const std::size_t n = proto::encodeCommand(c, seq++, buf, sizeof buf);
  t.send(buf, n);
}
const char* modeName(ControlMode m) {
  switch (m) {
    case ControlMode::Hold: return "HOLD";
    case ControlMode::Manual: return "MANUAL";
    case ControlMode::Mission: return "MISSION";
    case ControlMode::ReturnHome: return "RTH";
  }
  return "?";
}
const char* safetyName(SafetyState s) {
  switch (s) {
    case SafetyState::Nominal: return "NOMINAL";
    case SafetyState::Degraded: return "DEGRADED";
    case SafetyState::Failsafe: return "FAILSAFE";
    case SafetyState::EmergencyStop: return "E-STOP";
  }
  return "?";
}
}  // namespace

int main(int argc, char** argv) {
  const std::string scenario = argc > 1 ? argv[1] : "nominal";
  const bool link_loss = scenario == "link-loss";

  ManualClock clock;
  sim::SimVehicle vehicle(VehicleState{});
  std::vector<sim::SimObstacle> obstacles = {
      {{20.0, 0.3, 0.0}, {0.0, 0.0, 0.0}, 1.2},      // static, sits on the first leg
      {{60.0, 15.0, 0.0}, {-1.5, 0.0, 0.0}, 1.0},    // crosses the second leg
  };
  sim::SimRadar radar(vehicle, obstacles, clock);

  VehicleConfig cfg;
  cfg.kind = VehicleKind::Ground;
  VehicleController ctl(cfg, VehiclePorts{vehicle, radar, vehicle});

  auto [dashboard, onboard] = LoopbackTransport::makePair();
  CommandQueue queue;
  CommandGateway gateway(*onboard, queue);

  Mission m;
  m.id = 1;
  m.count = 3;
  m.waypoints[0] = {{40.0, 0.0, 0.0}, 4.0f, 1.5f, 0.0f};
  m.waypoints[1] = {{40.0, 30.0, 0.0}, 4.0f, 1.5f, 0.0f};
  m.waypoints[2] = {{0.0, 30.0, 0.0}, 4.0f, 1.5f, 1.0f};

  std::uint16_t seq = 0;
  constexpr TimeNs dt = millis(10);
  double min_clearance = kInf;
  bool uploaded = false, armed_sent = false, started = false;
  std::printf("%7s %-8s %-9s %7s %7s %6s %5s\n", "t[s]", "mode", "safety", "x[m]", "y[m]", "v[m/s]", "trks");

  for (int i = 0; i < 12000; ++i) {  // up to 120 s of simulated time
    const TimeNs now = clock.now();
    const double t = toSeconds(now);

    // --- simulated dashboard operator ---
    const bool link_up = !(link_loss && t > 8.0);
    if (link_up && i % 10 == 0) sendCmd(*dashboard, CmdHeartbeat{}, seq);
    if (!uploaded && t > 0.2) { sendCmd(*dashboard, CmdMissionUpload{m}, seq); uploaded = true; }
    if (!armed_sent && t > 0.8) { sendCmd(*dashboard, CmdArm{true}, seq); armed_sent = true; }
    if (!started && t > 1.0) { sendCmd(*dashboard, CmdMissionControl{MissionAction::Start}, seq); started = true; }

    // --- vehicle side: comms -> control -> physics ---
    gateway.poll(now);
    CommandEnvelope env;
    while (queue.pop(env)) ctl.handle(env);
    ctl.step(now);
    if (i % 10 == 0) gateway.sendTelemetry(ctl.telemetry());

    for (auto& o : obstacles) o.position += o.velocity * toSeconds(dt);
    vehicle.step(now, toSeconds(dt));
    for (const auto& o : obstacles) {
      min_clearance = std::min(min_clearance, (o.position - vehicle.truth().position).norm() - o.radius_m);
    }
    clock.advance(dt);

    if (i % 100 == 0) {
      const Telemetry tl = ctl.telemetry();
      std::printf("%7.1f %-8s %-9s %7.1f %7.1f %6.2f %5u\n", t, modeName(tl.mode), safetyName(tl.safety),
                  tl.position.x, tl.position.y, tl.velocity.norm(), static_cast<unsigned>(tl.tracks));
    }
    if (ctl.missionState() == MissionState::Completed && vehicle.truth().velocity.norm() < 0.05) break;
    if (link_loss && t > 40.0) break;
  }

  std::printf("\nmin clearance to any obstacle surface: %.2f m\n", min_clearance);
  std::printf("final mission state: %d, rejected commands: %u\n", static_cast<int>(ctl.missionState()),
              ctl.telemetry().rejected_commands);
  if (link_loss) {
    const Telemetry tl = ctl.telemetry();
    std::printf("link-loss scenario ended in mode %s, %.1f m from home\n", modeName(tl.mode), tl.position.norm2d());
    return (tl.mode == ControlMode::Hold && tl.position.norm2d() < 3.0) ? 0 : 1;
  }
  return (ctl.missionState() == MissionState::Completed && min_clearance > 0.0) ? 0 : 1;
}
