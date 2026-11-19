#pragma once
// The deterministic heart of the system. step(now) is a pure function of (previous state, inputs
// polled through ports, injected time). Single-threaded by contract, allocation-free, bounded
// execution time. Threads live one layer up (VehicleRuntime); tests drive it with a ManualClock.
//
// One control cycle:
//   poll state/radar -> track -> safety verdict -> mode logic (manual | mission | RTH | hold)
//   -> collision avoidance -> command guard (limits, geofence, slew) -> heading control -> actuate
#include <cstdint>

#include "sdt/avoidance/collision_avoidance.hpp"
#include "sdt/comms/messages.hpp"
#include "sdt/control/pid.hpp"
#include "sdt/hal/ports.hpp"
#include "sdt/nav/mission_executor.hpp"
#include "sdt/perception/radar_tracker.hpp"
#include "sdt/safety/command_guard.hpp"
#include "sdt/safety/geofence.hpp"
#include "sdt/safety/safety_supervisor.hpp"

namespace sdt {

  double manual_max_vertical_mps{2.0};
  double manual_max_yaw_rate_rps{1.0};
  TimeNs manual_timeout{millis(500)};  // stale sticks -> zero velocity
  double rth_speed_mps{3.0};
  double rth_arrival_m{1.5};
  double face_travel_min_speed_mps{0.3};
  TimeNs default_dt{millis(10)};
  double max_disarm_speed_mps{0.5};
};

struct VehiclePorts {
  IStateSource& state;
  IRadarSource& radar;
  IActuatorSink& actuator;
};

class VehicleController {
 public:
  VehicleController(const VehicleConfig& cfg, VehiclePorts ports) noexcept;

  /// Apply one decoded command (called by the same thread that calls step()).
  void handle(const CommandEnvelope& env) noexcept;
  /// Run one control cycle at time `now`.
  void step(TimeNs now) noexcept;

  Telemetry telemetry() const noexcept { return telemetry_; }
  bool armed() const noexcept { return armed_; }
  ControlMode mode() const noexcept { return mode_; }
  const VehicleState& state() const noexcept { return state_; }
  const SafetyVerdict& verdict() const noexcept { return verdict_; }
  MissionState missionState() const noexcept { return mission_.state(); }
  std::size_t obstacleCount() const noexcept { return n_obstacles_; }
  const AvoidanceResult& lastAvoidance() const noexcept { return avoid_result_; }

 private:
  void onCommand(const CmdHeartbeat&) noexcept {}
  void onCommand(const CmdArm& c) noexcept;
  void onCommand(const CmdSetMode& c) noexcept;
  void onCommand(const CmdManual& c) noexcept;
  void onCommand(const CmdMissionUpload& c) noexcept;
  void onCommand(const CmdMissionControl& c) noexcept;

  void enterMode(ControlMode m) noexcept;
  bool preArmOk() const noexcept;
  bool fresh(TimeNs last, TimeNs max_age) const noexcept { return ageOf(now_, last) <= max_age; }
  Vec3 desiredVelocity(double& manual_yaw_rate, bool& manual) noexcept;
  void publishTelemetry() noexcept;

  VehicleConfig cfg_;
  VehiclePorts ports_;
  RadarTracker tracker_;
  CollisionAvoidance avoidance_;
  CommandGuard guard_;
  SafetySupervisor supervisor_;
  MissionExecutor mission_;
  PathFollower follower_;
  Pid heading_pid_;

  VehicleState state_{};
  RadarScan scan_{};
  ObstacleList obstacles_{};
  std::size_t n_obstacles_{0};
  SafetyVerdict verdict_{};
  AvoidanceResult avoid_result_{};
  Telemetry telemetry_{};
  CmdManual manual_{};

  TimeNs now_{0};
  TimeNs last_link_{kNever};
  TimeNs last_state_{kNever};
  TimeNs last_radar_{kNever};
  TimeNs last_manual_{kNever};
  bool first_{true};
  bool armed_{false};
  bool collision_flag_{false};
  ControlMode mode_{ControlMode::Hold};
  Vec3 home_{};
  Vec3 rth_from_{};
  std::uint32_t rejected_{0};
};

}  // namespace sdt
