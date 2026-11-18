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
