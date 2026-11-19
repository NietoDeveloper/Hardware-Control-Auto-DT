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
  double max_deflection_rad{1.75};
  int speed_levels{4};
  double w_deviation{1.0};
  double w_ttc{2.0};

struct AvoidanceResult {
  Vec3 veloc
  double min_ttc_s{kInf};
  bool modified{false};
  bool no_safe_option{false

class CollisionAvoidance {
 public:
  explicit CollisionAvoidance(const AvoidanceConfig& cfg = {}) noexcept : cfg_(cfg) {}

  AvoidanceResult resolve(const VehicleState& own, const Vec3& desired, const Obstacle* obstacles,
                          std::size_t count, double max_speed) noexcept;

  void reset() noexcept { last_offset_ = 0.0; }

 private:
  double minTtc(const VehicleState& own, const Vec3& v, const Obstacle* obs,
                std::size_t n) const noexcept;
  AvoidanceConfig cfg_;
  double last_offset_{0.0};
};

}  // namespace sdt
