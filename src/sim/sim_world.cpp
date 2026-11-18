#include "sdt/sim/sim_world.hpp"

#include <algorithm>
#include <cmath>

namespace sdt::sim {

void SimVehicle::step(TimeNs now, double dt) noexcept {
  if (!estopped_) {
    const Vec3 delta = cmd_.velocity - state_.velocity;
    const double dn = delta.norm();
    const double max_step = p_.max_accel_mps2 * dt;
    state_.velocity = (dn > max_step && dn > 1e-12) ? state_.velocity + delta * (max_step / dn)
                                                    : cmd_.velocity;
    state_.yaw_rate = cmd_.yaw_rate;
  } else {
    state_.yaw_rate = 0.0;
  }
  state_.position += state_.velocity * dt;
  state_.yaw = wrapPi(state_.yaw + state_.yaw_rate * dt);
  state_.stamp = now;
}

bool SimVehicle::poll(VehicleState& out) noexcept {
  if (frozen_) return false;
  out = state_;
  return true;
}

double SimRadar::noise() noexcept {
  rng_ ^= rng_ << 13;
  rng_ ^= rng_ >> 7;
  rng_ ^= rng_ << 17;
  return static_cast<double>(rng_ >> 11) / static_cast<double>(1ull << 52) - 1.0;  // [-1, 1)
}

bool SimRadar::poll(RadarScan& out) noexcept {
  const TimeNs now = clock_.now();
  if (!enabled_ || ageOf(now, last_) < p_.period) return false;
  last_ = now;

  out.stamp = now;
  out.count = 0;
  const VehicleState& v = veh_.truth();
  const double cy = std::cos(v.yaw);
  const double sy = std::sin(v.yaw);

  for (const SimObstacle& o : obs_) {
    const Vec3 rel = o.position - v.position;
    const double dist = rel.norm();
    if (dist < 1e-6 || dist - o.radius_m > p_.max_range_m) continue;
    const Vec3 dir = rel * (1.0 / dist);
    const Vec3 perp{-dir.y, dir.x, 0.0};
    // Three scattering points on the visible surface: nearest point and two lateral flanks.
    const Vec3 pts[3] = {o.position - dir * o.radius_m,
                         o.position - dir * (0.7 * o.radius_m) + perp * (0.7 * o.radius_m),
                         o.position - dir * (0.7 * o.radius_m) - perp * (0.7 * o.radius_m)};
    for (const Vec3& p : pts) {
      if (out.count >= kMaxRadarDetections) break;
      const Vec3 d = p - v.position;
      const double xb = d.x * cy + d.y * sy;
      const double yb = -d.x * sy + d.y * cy;
      const double az = std::atan2(yb, xb);
      if (std::fabs(az) > p_.half_fov_rad) continue;
      const double rng = std::sqrt(xb * xb + yb * yb + d.z * d.z);
      if (rng > p_.max_range_m) continue;
      RadarDetection& det = out.detections[out.count++];
      det.range_m = static_cast<float>(rng + p_.range_sigma_m * noise());
      det.azimuth_rad = static_cast<float>(az + p_.angle_sigma_rad * noise());
      det.elevation_rad = static_cast<float>(std::atan2(d.z, std::hypot(xb, yb)));
      det.radial_velocity_mps = 0.0f;
      det.snr_db = 20.0f;
    }
  }
  return true;
}

}  // namespace sdt::sim
