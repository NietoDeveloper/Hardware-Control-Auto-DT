#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "sdt/common/types.hpp"

namespace sdt {

struct Waypoint {
  Vec3 position;             // local ENU, m
