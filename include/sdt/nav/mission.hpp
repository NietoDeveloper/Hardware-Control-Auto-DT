#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "sdt/common/types.hpp"

namespace sdt {

struct Waypoint {
  Vec3 position;             // local ENU, m
  float speed_mps{2.0f};     // cruise speed toward this waypoint
  float acceptance_m{1.0f};  // reached when within this 3D radius
  float hold_s{0.0f};        // dwell time on arrival (0 = fly through)
};

