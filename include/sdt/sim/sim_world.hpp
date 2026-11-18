#pragma once
// Software-in-the-loop simulation adapters: a kinematic vehicle, a radar with noise, and moving
// spherical obstacles. They implement the same HAL ports a real platform does, so the exact same
// VehicleController binary logic runs in simulation and on hardware.
#include <cstdint>
#include <vector>

#include "sdt/common/clock.hpp"
#include "sdt/hal/ports.hpp"

namespace sdt::sim {

struct SimObstacle {
  Vec3 position;
  Vec3 velocity;
  double radius_m{1.0};
};

struct SimVehicleParams {
  double max_accel_mps2{3.0};
};

struct SimRadarParams {
  double max_range_m{80.0};
  double half_fov_rad{2.1};   // ~120 deg each side
  TimeNs period{millis(50)};  // 20 Hz
  double range_sigma_m{0.08};
  double angle_sigma_rad{0.004};
};

class SimVehicle final : public IStateSource, public IActuatorSink {
 public:
  using Params = SimVehicleParams;
  explicit SimVehicle(const VehicleState& initial, const Params& p = Params{}) noexcept : p_(p), state_(initial) {}

  void step(TimeNs now, double dt) noexcept;
  const VehicleState& truth() const noexcept { return state_; }
  bool estopped() const noexcept { return estopped_; }
  void freezeEstimate(bool on) noexcept { frozen_ = on; }  // fault injection: estimator goes silent

  bool poll(VehicleState& out) noexcept override;
  void apply(const VelocityCommand& cmd) noexcept override { cmd_ = cmd; }
  void emergencyStop() noexcept override {
    estopped_ = true;
    cmd_ = VelocityCommand{};
    state_.velocity = Vec3{};
  }

 private:
  Params p_;
  VehicleState state_;
  VelocityCommand cmd_{};
  bool estopped_{false};
  bool frozen_{false};
};

class SimRadar final : public IRadarSource {
 public:
  using Params = SimRadarParams;
  SimRadar(const SimVehicle& veh, const std::vector<SimObstacle>& obstacles, const IClock& clock,
           const Params& p = Params{}) noexcept
      : veh_(veh), obs_(obstacles), clock_(clock), p_(p) {}

  void setEnabled(bool on) noexcept { enabled_ = on; }  // fault injection: radar goes silent
  bool poll(RadarScan& out) noexcept override;

 private:
  double noise() noexcept;  // deterministic uniform in [-1, 1]
  const SimVehicle& veh_;
  const std::vector<SimObstacle>& obs_;
  const IClock& clock_;
  Params p_;
  TimeNs last_{kNever};
  bool enabled_{true};
  std::uint64_t rng_{0x9E3779B97F4A7C15ull};
};

}  // namespace sdt::sim
