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
  ctl.handle({clock.now(), CmdMissionUpload{m}});
  for (int i = 0; i < 80; ++i) {
  for (int i = 0; i < kN; ++i) {
    if (i % 10 == 0) ctl.handle({clock.now(), CmdHeartbeat{}});
    const auto t0 = std::chrono::steady_clock::now();
    ctl.step(clock.now());
    const auto t1 = std::chrono::steady_clock::now();
    ns.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    veh.step(clock.now(), 0.01);
    clock.advance(millis(10));
  }
  std::sort(ns.begin(), ns.end());
  auto pct = [&](double p) { return static_cast<double>(ns[static_cast<std::size_t>(p * (kN - 1))]) / 1e3; };
  std::printf("step() over %d cycles, %zu confirmed tracks, vehicle at x=%.1f m\n", kN, ctl.obstacleCount(),
              veh.truth().position.x);
  std::printf("  p50 %.1f us | p99 %.1f us | p99.9 %.1f us | max %.1f us   (budget at 100 Hz: 10000 us)\n",
              pct(0.5), pct(0.99), pct(0.999), static_cast<double>(ns.back()) / 1e3);
  return 0;
}
