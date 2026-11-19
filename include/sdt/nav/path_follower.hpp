#pragma once
#include "sdt/common/types.hpp"
#include "sdt/nav/mission.hpp"

namespace sdt {

struct FollowerConfig {
  double lookahead_m{4.0};
  double approach_decel_mps2{1.2};
  double min_approach_speed_mps{0.3};
};

/// Carrot-chasing along the segment `from -> to`, decelerating to a stop when requested.
class PathFollower {
 public:
  explicit PathFollower(const FollowerConfig& cfg = {}) noexcept : cfg_(cfg) {}
  Vec3 desiredVelocity(const Vec3& pos, const Vec3& from, const Waypoint& to,
                       bool stop_at_end) const noexcept;

 private:
  FollowerConfig cfg_;