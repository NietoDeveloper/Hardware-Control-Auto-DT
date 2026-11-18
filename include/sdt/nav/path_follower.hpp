#pragma once
#include "sdt/common/types.hpp"
#include "sdt/nav/mission.hpp"

namespace sdt {

struct FollowerConfig {
  double lookahead_m{4.0};
  double approach_decel_mps2{1.2};
