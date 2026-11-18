#pragma once
// Independent safety supervisor: a small, side-effect-free state machine that turns health inputs
// into a verdict. It never touches actuators; the controller enforces its verdict. Keeping it pure
// makes it exhaustively unit-testable and a candidate for a separate, higher-assurance process.
#include <cstdint>

#include "sdt/common/types.hpp"

namespace sdt {

enum class SafetyState : std::uint8_t { Nominal = 0, Degraded = 1, Failsafe = 2, EmergencyStop = 3 };
enum class FailsafeAction : std::uint8_t { None = 0, Hold = 1, ReturnHome = 2, EmergencyStop = 3 };

namespace fault {
inline constexpr std::uint32_t kLinkLost = 1u << 0;
inline constexpr std::uint32_t kStateStale = 1u << 1;
inline constexpr std::uint32_t kRadarStale = 1u << 2;
inline constexpr std::uint32_t kGeofence = 1u << 3;
inline constexpr std::uint32_t kCollision = 1u << 4;
}  // namespace fault

struct SafetyConfig {
  TimeNs link_hold_after{millis(2000)};      // no valid command frame for this long -> Hold
  TimeNs link_rth_after{millis(10000)};      // ... and for this long -> Return Home
  TimeNs state_stale_after{millis(250)};     // no state estimate -> latched Emergency Stop
  TimeNs radar_degraded_after{millis(500)};  // no radar scan -> speed cap
  TimeNs radar_hold_after{millis(2500)};     // ... still none -> Hold
  TimeNs recovery_time{millis(1000)};        // hysteresis before a failsafe auto-clears
  double nominal_speed_limit_mps{8.0};
  double degraded_speed_limit_mps{1.5};
  bool radar_required{true};
  bool rth_on_link_loss{true};
};

struct SafetyInputs {
  TimeNs now{0};
  TimeNs last_link{kNever};
  TimeNs last_state{kNever};
  TimeNs last_radar{kNever};
  bool armed{false};
  double geofence_margin_m{kInf};
  bool collision_imminent{false};
};

struct SafetyVerdict {
  SafetyState state{SafetyState::Nominal};
  FailsafeAction action{FailsafeAction::None};
  std::uint32_t faults{0};
  double speed_limit_mps{0.0};
};

class SafetySupervisor {
 public:
  explicit SafetySupervisor(const SafetyConfig& cfg = {}) noexcept : cfg_(cfg) {}
  SafetyVerdict evaluate(const SafetyInputs& in) noexcept;
  /// Clears the Emergency-Stop latch. Only invoked by an explicit operator disarm.
  void reset() noexcept;
  bool latched() const noexcept { return latched_; }

 private:
  SafetyConfig cfg_;
  bool latched_{false};
  FailsafeAction held_action_{FailsafeAction::None};
  TimeNs clear_since_{kNever};
};

}  // namespace sdt
