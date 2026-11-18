#include "sdt/safety/safety_supervisor.hpp"

#include <algorithm>

namespace sdt {
namespace {
FailsafeAction maxAction(FailsafeAction a, FailsafeAction b) noexcept {
  return static_cast<std::uint8_t>(a) >= static_cast<std::uint8_t>(b) ? a : b;
}
}  // namespace

void SafetySupervisor::reset() noexcept {
  latched_ = false;
  held_action_ = FailsafeAction::None;
  clear_since_ = kNever;
}

SafetyVerdict SafetySupervisor::evaluate(const SafetyInputs& in) noexcept {
  const TimeNs link_age = ageOf(in.now, in.last_link);
  const TimeNs state_age = ageOf(in.now, in.last_state);
  const TimeNs radar_age = ageOf(in.now, in.last_radar);

  std::uint32_t faults = 0;
  if (link_age > cfg_.link_hold_after) faults |= fault::kLinkLost;
  if (state_age > cfg_.state_stale_after) faults |= fault::kStateStale;
  if (cfg_.radar_required && radar_age > cfg_.radar_degraded_after) faults |= fault::kRadarStale;
  if (in.geofence_margin_m < 0.0) faults |= fault::kGeofence;
  if (in.collision_imminent) faults |= fault::kCollision;

  SafetyVerdict v;
  v.faults = faults;
  v.speed_limit_mps = cfg_.nominal_speed_limit_mps;

  if (latched_) {
    v.state = SafetyState::EmergencyStop;
    v.action = FailsafeAction::EmergencyStop;
    v.speed_limit_mps = 0.0;
    return v;
  }

  if (!in.armed) {  // report health for pre-arm checks, never escalate while disarmed
    held_action_ = FailsafeAction::None;
    clear_since_ = kNever;
    return v;
  }

  // Losing the state estimate means we cannot navigate or avoid anything: terminate safely.
  if ((faults & fault::kStateStale) != 0u) {
    latched_ = true;
    v.state = SafetyState::EmergencyStop;
    v.action = FailsafeAction::EmergencyStop;
    v.speed_limit_mps = 0.0;
    return v;
  }

  FailsafeAction candidate = FailsafeAction::None;
  if ((faults & fault::kLinkLost) != 0u) {
    candidate = (cfg_.rth_on_link_loss && link_age > cfg_.link_rth_after)
                    ? FailsafeAction::ReturnHome
                    : FailsafeAction::Hold;
  }
  if ((faults & (fault::kGeofence | fault::kCollision)) != 0u) {
    candidate = maxAction(candidate, FailsafeAction::Hold);
  }
  if (cfg_.radar_required && radar_age > cfg_.radar_hold_after) {
    candidate = maxAction(candidate, FailsafeAction::Hold);
  }

  // Hysteresis: a failsafe only clears after conditions stay healthy for `recovery_time`.
  if (candidate != FailsafeAction::None) {
    held_action_ = maxAction(held_action_, candidate);
    clear_since_ = kNever;
    candidate = held_action_;
  } else if (held_action_ != FailsafeAction::None) {
    if (clear_since_ == kNever) clear_since_ = in.now;
    if (in.now - clear_since_ >= cfg_.recovery_time) {
      held_action_ = FailsafeAction::None;
      clear_since_ = kNever;
    } else {
      candidate = held_action_;
    }
  }

  v.action = candidate;
  if (candidate != FailsafeAction::None) {
    v.state = SafetyState::Failsafe;
  } else if ((faults & fault::kRadarStale) != 0u) {
    v.state = SafetyState::Degraded;
  }
  if ((faults & fault::kRadarStale) != 0u) v.speed_limit_mps = cfg_.degraded_speed_limit_mps;
  return v;
}

}  // namespace sdt
