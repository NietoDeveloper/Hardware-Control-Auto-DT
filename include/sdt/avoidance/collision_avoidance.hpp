#pragma once
// Velocity-sampling collision avoidance (a discretised velocity-obstacle method).
// Bounded compute: speed_levels x (2K+1) candidates x <= 64 tracks, no allocation.
#include <cstddef>

#include "sdt/common/types.hpp"
#include "sdt/perception/radar_tracker.hpp"

namespace sdt {

struct AvoidanceConfig {
  double vehicle_radius_m{0.6};
  double safety_margin_m{1.0};
  double horizon_s{6.0};        // start shaping the velocity when a conflict is closer than this
  double critical_ttc_s{1.5};   // candidates with time-to-collision below this are unsafe
  int heading_steps{12};        // K: candidates at base +/- k * (max_deflection / K)
