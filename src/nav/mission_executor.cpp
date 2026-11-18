#include "sdt/nav/mission_executor.hpp"

namespace sdt {
namespace {
bool validWaypoint(const Waypoint& w) noexcept {
  return w.position.finite() && std::isfinite(w.speed_mps) && w.speed_mps > 0.0f &&
         w.speed_mps <= 100.0f && std::isfinite(w.acceptance_m) && w.acceptance_m > 0.0f &&
         std::isfinite(w.hold_s) && w.hold_s >= 0.0f && w.hold_s <= 3600.0f;
}
}  // namespace

bool MissionExecutor::load(const Mission& m) noexcept {
  if (state_ == MissionState::Running || state_ == MissionState::Paused) return false;
  if (m.count == 0 || m.count > kMaxWaypoints) return false;
  for (std::size_t i = 0; i < m.count; ++i) {
    if (!validWaypoint(m.waypoints[i])) return false;
  }
  mission_ = m;
  state_ = MissionState::Idle;
  index_ = 0;
  return true;
}

bool MissionExecutor::start(const Vec3& position, TimeNs /*now*/) noexcept {
  if (!hasMission() || state_ == MissionState::Running || state_ == MissionState::Paused) return false;
  index_ = 0;
  from_ = position;
  dwelling_ = false;
  state_ = MissionState::Running;
  return true;
}

bool MissionExecutor::pause() noexcept {
  if (state_ != MissionState::Running) return false;
  state_ = MissionState::Paused;
  return true;
}

bool MissionExecutor::resume() noexcept {
  if (state_ != MissionState::Paused) return false;
  state_ = MissionState::Running;
  return true;
}

void MissionExecutor::abort() noexcept {
  if (state_ == MissionState::Running || state_ == MissionState::Paused) state_ = MissionState::Aborted;
}

void MissionExecutor::advance() noexcept {
  from_ = mission_.waypoints[index_].position;
  dwelling_ = false;
  if (++index_ >= mission_.count) {
    if (mission_.loop) {
      index_ = 0;
    } else {
      index_ = static_cast<std::uint16_t>(mission_.count - 1);
      state_ = MissionState::Completed;
    }
  }
}

Vec3 MissionExecutor::step(const VehicleState& s, TimeNs now) noexcept {
  if (state_ != MissionState::Running) return Vec3{};

  const Waypoint& wp = mission_.waypoints[index_];
  const double dist = (wp.position - s.position).norm();

  if (!dwelling_ && dist <= static_cast<double>(wp.acceptance_m)) {
    if (wp.hold_s > 0.0f) {
      dwelling_ = true;
      dwell_start_ = now;
    } else {
      advance();
    }
  }
  if (dwelling_) {
    if (toSeconds(now - dwell_start_) < static_cast<double>(wp.hold_s)) return Vec3{};
    advance();
  }
  if (state_ != MissionState::Running) return Vec3{};

  const Waypoint& cur = mission_.waypoints[index_];
  const bool last = (index_ + 1 == mission_.count) && !mission_.loop;
  return follower_.desiredVelocity(s.position, from_, cur, last || cur.hold_s > 0.0f);
}

}  // namespace sdt
