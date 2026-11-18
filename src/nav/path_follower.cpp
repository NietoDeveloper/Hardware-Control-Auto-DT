#include "sdt/nav/path_follower.hpp"

#include <algorithm>
#include <cmath>

namespace sdt {

Vec3 PathFollower::desiredVelocity(const Vec3& pos, const Vec3& from, const Waypoint& to,
                                   bool stop_at_end) const noexcept {
  const Vec3 seg = to.position - from;
  const double len = seg.norm();
  const Vec3 to_target = to.position - pos;
  const double remaining = to_target.norm();
  if (remaining < 1e-6) return Vec3{};

  Vec3 aim = to_target;
  if (len > 1e-6) {
    const Vec3 dir = seg * (1.0 / len);
    const double s = (pos - from).dot(dir);
    const Vec3 carrot = from + dir * clampd(s + cfg_.lookahead_m, 0.0, len);
    const Vec3 to_carrot = carrot - pos;
    if (to_carrot.norm() > 1e-6) aim = to_carrot;
  }

  double speed = static_cast<double>(to.speed_mps);
  if (stop_at_end) {
    const double usable = std::max(remaining - 0.5 * static_cast<double>(to.acceptance_m), 0.0);
    const double v_stop = std::sqrt(2.0 * cfg_.approach_decel_mps2 * usable);
    speed = std::min(speed, std::max(v_stop, cfg_.min_approach_speed_mps));
  }
  return aim * (speed / aim.norm());
}

}  // namespace sdt
