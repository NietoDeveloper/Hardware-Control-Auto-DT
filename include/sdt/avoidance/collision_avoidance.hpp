#pragma once
// Velocity-sampling collision avoidance (a discretised velocity-obstacle method).
// Bounded compute: speed_levels x (2K+1) candidates x <= 64 tracks, no allocation.
#include <cstddef>

#include "sdt/common/types.hpp"
#include "sdt/perception/radar_tracker.hpp"

namespace sdt {

struct AvoidanceConfig {
