#include "sdt/avoidance/collision_avoidance.hpp"

#include <algorithm>
#include <cmath>

namespace sdt {
namespace {
/// Earliest t >= 0 at which |p - v t| <= R, where p = obstacle - own, v = own - obstacle velocity.
double timeToCollision(const Vec3& p, const Vec3& v, double radius) noexcept {
  const double pv = p.dot(v);
  const double c = p.dot(p) - radius * radius;
  if (c <= 0.0) return pv > 0.0 ? 0.0 : kInf;  // already inside: only "separating" is acceptable
  const double a = v.dot(v);
  if (a < 1e-9 || pv <= 0.0) return kInf;
  const double disc = pv * pv - a * c;
  if (disc < 0.0) return kInf;
  return (pv - std::sqrt(disc)) / a;
}
}  // namespace

double CollisionAvoidance::minTtc(const VehicleState& own, const Vec3& v, const Obstacle* obs,
                                  std::size_t n) const noexcept {
  double m = kInf;
  for (std::size_t i = 0; i < n; ++i) {
    const double r = obs[i].radius_m + cfg_.vehicle_radius_m + cfg_.safety_margin_m;
    m = std::min(m, timeToCollision(obs[i].position - own.position, v - obs[i].velocity, r));
  }
  return m;
}

AvoidanceResult CollisionAvoidance::resolve(const VehicleState& own, const Vec3& desired,
                                            const Obstacle* obs, std::size_t n,
                                            double max_speed) noexcept {
  AvoidanceResult out;
  out.velocity = desired;
  if (n == 0) {
    last_offset_ = 0.0;
    return out;
  }
  out.min_ttc_s = minTtc(own, desired, obs, n);
  if (out.min_ttc_s >= cfg_.horizon_s) {
    last_offset_ = 0.0;
    return out;  // nothing within the planning horizon
  }

  const double base = std::atan2(desired.y, desired.x);
  const double top = std::min(desired.norm2d(), std::max(max_speed, 0.0));
  const double norm = std::max(top, 0.5);
  const int K = std::max(cfg_.heading_steps, 1);
  const double step = cfg_.max_deflection_rad / static_cast<double>(K);
  const int levels = std::max(cfg_.speed_levels, 1);

  double best_cost = kInf;
  double best_ttc = 0.0;
  double best_offset = 0.0;
  Vec3 best_v = desired;
  bool found_safe = false;

  double fb_ttc = -1.0;  // fallback = candidate with the largest time-to-collision
  Vec3 fb_v{0.0, 0.0, desired.z};

  for (int s = 0; s <= levels; ++s) {  // s == levels -> full stop candidate
    const double speed = top * (1.0 - static_cast<double>(s) / static_cast<double>(levels));
    const int k_max = (s == levels) ? 0 : K;
    for (int k = -k_max; k <= k_max; ++k) {
      const double offset = static_cast<double>(k) * step;
      const double th = base + offset;
      const Vec3 v{speed * std::cos(th), speed * std::sin(th), desired.z};
      const double ttc = minTtc(own, v, obs, n);

      if (ttc > fb_ttc) {
        fb_ttc = ttc;
        fb_v = v;
      }
      if (ttc < cfg_.critical_ttc_s) continue;

      double cost = cfg_.w_deviation * (v - desired).norm() / norm +
                    cfg_.w_hysteresis * std::fabs(offset - last_offset_);
      if (ttc < cfg_.horizon_s) cost += cfg_.w_ttc * (cfg_.horizon_s - ttc) / cfg_.horizon_s;
      if (cost < best_cost) {
        best_cost = cost;
        best_v = v;
        best_ttc = ttc;
        best_offset = offset;
        found_safe = true;
      }
    }
  }

  if (found_safe) {
    out.velocity = best_v;
    out.min_ttc_s = best_ttc;
    last_offset_ = best_offset;
  } else {
    out.velocity = fb_v;
    out.min_ttc_s = fb_ttc;
    out.no_safe_option = true;
  }
  out.modified = (out.velocity - desired).norm() > 1e-6;
  return out;
}

}  // namespace sdt
