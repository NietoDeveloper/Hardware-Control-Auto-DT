#include "sdt/runtime/vehicle_runtime.hpp"

namespace sdt {

VehicleRuntime::VehicleRuntime(VehicleController& controller, ITransport& transport,
                               const IClock& clock, const RuntimeConfig& cfg, MediaRouter* media)
    : controller_(controller),
      clock_(clock),
      cfg_(cfg),
      media_(media),
      cmd_q_(std::make_unique<CommandQueue>()),
      gateway_(transport, *cmd_q_) {}

void VehicleRuntime::controlTick() {
  CommandEnvelope env;
  while (cmd_q_->pop(env)) controller_.handle(env);
  const TimeNs now = clock_.now();
  controller_.step(now);
  if (ageOf(now, last_tel_) >= cfg_.telemetry_period) {
    last_tel_ = now;
    tel_q_.push(controller_.telemetry());  // full queue = comms is behind; drop, never block
  }
}

void VehicleRuntime::commsTick() {
  gateway_.poll(clock_.now());
  Telemetry t;
  while (tel_q_.pop(t)) gateway_.sendTelemetry(t);
}

bool VehicleRuntime::start() {
  // Start comms first so commands are flowing before the control loop's first cycle.
  bool ok = comms_.start("sdt-comms", cfg_.comms_period, [this] { commsTick(); }, cfg_.comms_rt);
  ok = ok && control_.start("sdt-control", cfg_.control_period, [this] { controlTick(); }, cfg_.control_rt);
  if (ok && media_ != nullptr) {
    ok = media_thread_.start("sdt-media", cfg_.media_period, [this] { media_->pump(); }, cfg_.media_rt);
  }
  if (!ok) stop();
  return ok;
}

void VehicleRuntime::stop() noexcept {
  media_thread_.stop();
  control_.stop();  // control first: stop issuing setpoints before the link goes away
  comms_.stop();
}

}  // namespace sdt
