// Measures the wall-clock cost of one VehicleController::step() under load
// (20 radar-tracked obstacles, mission running, avoidance active). Reports p50/p99/p99.9/max.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "sdt/runtime/vehicle_controller.hpp"
#include "sdt/sim/sim_world.hpp"

using namespace sdt;

int main() {
  ManualClock clock;
l(cfg, VehiclePorts{veh, radar, veh});

  Mission m;;
    clock.advance(millis(10));
  }
