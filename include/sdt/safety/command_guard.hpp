#pragma once
// Last line of defence between the planner and the actuators. Applies, in order:
// vertical clamp -> horizontal speed limit -> geofence barrier -> acceleration slew -> yaw clamp.
#include "sdt/common/types.hpp"
#include "sdt/safety/geofence.hpp"

namespace sdt {

struct GuardConfig {
  double max_speed_mps{10.0};
  double max_vertical_speed_mps{3.0};
  double max_accel_mps2{3.0};
  double max_decel_mps2{5.0};
  double max_yaw_rate_rps{1.5};
  double fence_margin_m{2.0};
};

class CommandGuard {
 public:
  explicit CommandGuard(const GuardConfig& cfg = {}) noexcept : cfg_(cfg) {}

  /// Re-seed the slew limiter (call when arming, with the measured velocity).
  void reset(const Vec3& current_velocity) noexcept { prev_ = current_velocity; }

  VelocityCommand filter(const VelocityCommand& in, const VehicleState& state,
                         double speed_limit_mps, const Geofence* fence, double dt) noexcept;

 private:
  GuardConfig cfg_;
  Vec3 prev_;
};

}  // namespace sdt
