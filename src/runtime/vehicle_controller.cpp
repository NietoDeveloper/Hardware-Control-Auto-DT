#include "sdt/runtime/vehicle_controller.hpp"

#include <algorithm>
#include <cmath>

namespace sdt {

VehicleController::VehicleController(const VehicleConfig& cfg, VehiclePorts ports) noexcept
    : cfg_(cfg),
      ports_(ports),
      tracker_(cfg.tracker),
      avoidance_(cfg.avoidance),
      guard_(cfg.guard),
      supervisor_(cfg.safety),
      mission_(cfg.follower),
      follower_(cfg.follower),
      heading_pid_(cfg.heading) {}

// ---------------------------------------------------------------------------------- commands --

void VehicleController::handle(const CommandEnvelope& env) noexcept {
  last_link_ = env.rx_stamp;  // any valid frame proves the link is alive
  std::visit([this](const auto& c) { onCommand(c); }, env.command);
}

bool VehicleController::preArmOk() const noexcept {
  return !supervisor_.latched() && fresh(last_state_, cfg_.safety.state_stale_after) &&
         (!cfg_.safety.radar_required || fresh(last_radar_, cfg_.safety.radar_degraded_after)) &&
         fresh(last_link_, cfg_.safety.link_hold_after) && cfg_.geofence.margin(state_.position) >= 0.0;
}

void VehicleController::onCommand(const CmdArm& c) noexcept {
  if (c.arm) {
    if (armed_ || !preArmOk()) {
      ++rejected_;
      return;
    }
    armed_ = true;
    home_ = state_.position;
    guard_.reset(state_.velocity);
    heading_pid_.reset();
    avoidance_.reset();
    enterMode(ControlMode::Hold);
    return;
  }
  const bool e_stopped = supervisor_.latched();
  const bool slow = state_.velocity.norm() <= cfg_.max_disarm_speed_mps;
  if (!armed_ && !e_stopped) return;
  if (!e_stopped && !slow) {  // never disarm a moving vehicle on a casual command
    ++rejected_;
    return;
  }
  armed_ = false;
  mission_.abort();
  enterMode(ControlMode::Hold);
  supervisor_.reset();
}

void VehicleController::onCommand(const CmdSetMode& c) noexcept {
  if (!armed_ && c.mode != ControlMode::Hold) {
    ++rejected_;
    return;
  }
  if (c.mode == ControlMode::Mission && !mission_.hasMission()) {
    ++rejected_;
    return;
  }
  enterMode(c.mode);
}

void VehicleController::onCommand(const CmdManual& c) noexcept {
  manual_ = c;
  last_manual_ = last_link_;
}

void VehicleController::onCommand(const CmdMissionUpload& c) noexcept {
  if (!mission_.load(c.mission)) ++rejected_;
}

void VehicleController::onCommand(const CmdMissionControl& c) noexcept {
  bool ok = false;
  switch (c.action) {
    case MissionAction::Start:
      ok = armed_ && mission_.start(state_.position, now_);
      if (ok) enterMode(ControlMode::Mission);
      break;
    case MissionAction::Pause: ok = mission_.pause(); break;
    case MissionAction::Resume: ok = armed_ && mission_.resume(); break;
    case MissionAction::Abort:
      mission_.abort();
      if (mode_ == ControlMode::Mission) enterMode(ControlMode::Hold);
      ok = true;
      break;
  }
  if (!ok) ++rejected_;
}

void VehicleController::enterMode(ControlMode m) noexcept {
  if (m == ControlMode::ReturnHome) {
    rth_from_ = state_.position;
    mission_.pause();
  }
  if (m == ControlMode::Manual) last_manual_ = kNever;
  mode_ = m;
}

// ------------------------------------------------------------------------------------- cycle --

Vec3 VehicleController::desiredVelocity(double& manual_yaw_rate, bool& manual) noexcept {
  manual = false;
  switch (mode_) {
    case ControlMode::Manual: {
      manual = true;
      if (ageOf(now_, last_manual_) > cfg_.manual_timeout) return Vec3{};  // stale sticks -> hold
      const bool ground = cfg_.kind == VehicleKind::Ground;
      const double f = static_cast<double>(manual_.forward) * cfg_.manual_max_speed_mps;
      const double l = ground ? 0.0 : static_cast<double>(manual_.lateral) * cfg_.manual_max_speed_mps;
      const double c = std::cos(state_.yaw);
      const double s = std::sin(state_.yaw);
      manual_yaw_rate = static_cast<double>(manual_.yaw) * cfg_.manual_max_yaw_rate_rps;
      return Vec3{f * c - l * s, f * s + l * c,
                  ground ? 0.0 : static_cast<double>(manual_.vertical) * cfg_.manual_max_vertical_mps};
    }
    case ControlMode::Mission: return mission_.step(state_, now_);
    case ControlMode::ReturnHome: {
      if ((home_ - state_.position).norm2d() <= cfg_.rth_arrival_m) {
        mode_ = ControlMode::Hold;
        return Vec3{};
      }
      Waypoint wp;
      wp.position = Vec3{home_.x, home_.y, cfg_.kind == VehicleKind::Aerial ? state_.position.z : home_.z};
      wp.speed_mps = static_cast<float>(cfg_.rth_speed_mps);
      wp.acceptance_m = static_cast<float>(cfg_.rth_arrival_m);
      return follower_.desiredVelocity(state_.position, rth_from_, wp, true);
    }
    case ControlMode::Hold: break;
  }
  return Vec3{};
}

void VehicleController::step(TimeNs now) noexcept {
  const double dt = first_ ? toSeconds(cfg_.default_dt) : clampd(toSeconds(now - now_), 0.001, 0.1);
  first_ = false;
  now_ = now;

  // 1) Sensing.
  VehicleState s;
  if (ports_.state.poll(s) && s.finite()) {
    state_ = s;
    last_state_ = now;
  }
  if (last_state_ != kNever) {
    for (int i = 0; i < 2 && ports_.radar.poll(scan_); ++i) {  // bounded drain
      tracker_.update(scan_, state_);
      last_radar_ = now;
    }
  }
  n_obstacles_ = tracker_.confirmed(obstacles_);

  // 2) Safety verdict.
  SafetyInputs in;
  in.now = now;
  in.last_link = last_link_;
  in.last_state = last_state_;
  in.last_radar = last_radar_;
  in.armed = armed_;
  in.geofence_margin_m = cfg_.geofence.margin(state_.position);
  in.collision_imminent = collision_flag_;
  verdict_ = supervisor_.evaluate(in);

  if (verdict_.action == FailsafeAction::EmergencyStop) {
    if (armed_) {
      ports_.actuator.emergencyStop();
      armed_ = false;
      mission_.abort();
      mode_ = ControlMode::Hold;
    }
    publishTelemetry();
    return;
  }
  if (!armed_) {
    heading_pid_.reset();
    ports_.actuator.apply(VelocityCommand{});
    publishTelemetry();
    return;
  }
  if (verdict_.action == FailsafeAction::ReturnHome && mode_ != ControlMode::ReturnHome) {
    enterMode(ControlMode::ReturnHome);
  }

  // 3) Intent.
  double manual_yaw_rate = 0.0;
  bool manual = false;
  Vec3 desired = desiredVelocity(manual_yaw_rate, manual);
  if (verdict_.action == FailsafeAction::Hold) desired = Vec3{};
  if (cfg_.kind == VehicleKind::Ground) desired.z = 0.0;

  // 4) Collision avoidance (also assists manual driving).
  const double max_speed = std::min(cfg_.guard.max_speed_mps, verdict_.speed_limit_mps);
  if (n_obstacles_ > 0 && desired.norm() > 0.05) {
    avoid_result_ = avoidance_.resolve(state_, desired, obstacles_.data(), n_obstacles_, max_speed);
    desired = avoid_result_.velocity;
    collision_flag_ = avoid_result_.no_safe_option;
  } else {
    avoid_result_ = AvoidanceResult{};
    collision_flag_ = false;
  }

  // 5) Hard limits, geofence barrier, slew.
  VelocityCommand cmd;
  cmd.velocity = desired;
  cmd = guard_.filter(cmd, state_, verdict_.speed_limit_mps, &cfg_.geofence, dt);
  if (cfg_.kind == VehicleKind::Ground) cmd.velocity.z = 0.0;

  // 6) Heading: sticks drive yaw in manual; otherwise face the direction of travel.
  if (manual) {
    cmd.yaw_rate = clampd(manual_yaw_rate, -cfg_.guard.max_yaw_rate_rps, cfg_.guard.max_yaw_rate_rps);
  } else if (cmd.velocity.norm2d() > cfg_.face_travel_min_speed_mps) {
    const double target = std::atan2(cmd.velocity.y, cmd.velocity.x);
    cmd.yaw_rate = heading_pid_.update(wrapPi(target - state_.yaw), dt);
  } else {
    heading_pid_.reset();
    cmd.yaw_rate = 0.0;
  }

  ports_.actuator.apply(cmd);
  publishTelemetry();
}

void VehicleController::publishTelemetry() noexcept {
  telemetry_.stamp = now_;
  telemetry_.position = state_.position;
  telemetry_.velocity = state_.velocity;
  telemetry_.yaw = state_.yaw;
  telemetry_.armed = armed_;
  telemetry_.mode = mode_;
  telemetry_.safety = verdict_.state;
  telemetry_.mission_state = mission_.state();
  telemetry_.mission_index = mission_.currentIndex();
  telemetry_.faults = verdict_.faults;
  telemetry_.tracks = static_cast<std::uint16_t>(n_obstacles_);
  telemetry_.rejected_commands = rejected_;
}

}  // namespace sdt
