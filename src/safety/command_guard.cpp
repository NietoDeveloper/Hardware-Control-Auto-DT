#include "sdt/safety/command_guard.hpp"

#include <algorithm>

namespace sdt {

VelocityCommand CommandGuard::filter(const VelocityCommand& in, const VehicleState& state,
                                     double speed_limit_mps, const Geofence* fence,
                                     double dt) noexcept {
  Vec3 v = in.velocity;
  if (!v.finite()) v = Vec3{};  // never forward garbage to hardware

  v.z = clampd(v.z, -cfg_.max_vertical_speed_mps, cfg_.max_vertical_speed_mps);

  const double limit = std::max(0.0, std::min(speed_limit_mps, cfg_.max_speed_mps));
  const double h = v.norm2d();
  if (h > limit && h > 1e-9) {
    const double k = limit / h;
    v.x *= k;
    v.y *= k;
  }

  // Geofence barrier: if the point reached after braking is outside AND worse than where we are
  // now, stop. Vehicles already outside may still move back toward the inside.
  if (fence != nullptr && fence->enabled) {
    const double speed = v.norm();
    if (speed > 1e-6) {
      const double brake = speed * speed / (2.0 * cfg_.max_decel_mps2) + cfg_.fence_margin_m;
      const Vec3 look = state.position + v * (brake / speed);
      const double m_now = fence->margin(state.position);
      const double m_look = fence->margin(look);
      if (m_look < 0.0 && m_look < m_now) v = Vec3{};
    }
  }

  // Acceleration slew relative to the previous command.
  const Vec3 delta = v - prev_;
  const double dn = delta.norm();
  const double accel = v.norm() < prev_.norm() ? cfg_.max_decel_mps2 : cfg_.max_accel_mps2;
  const double max_step = accel * std::max(dt, 0.0);
  if (dn > max_step && dn > 1e-12) v = prev_ + delta * (max_step / dn);
  prev_ = v;

  VelocityCommand out;
  out.velocity = v;
  out.yaw_rate = std::isfinite(in.yaw_rate)
                     ? clampd(in.yaw_rate, -cfg_.max_yaw_rate_rps, cfg_.max_yaw_rate_rps)
                     : 0.0;
  return out;
}

}  // namespace sdt
