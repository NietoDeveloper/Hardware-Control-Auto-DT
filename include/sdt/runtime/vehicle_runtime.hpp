#pragma once
// Threading model. Three loops, isolated by lock-free queues:
//   control (hard-ish RT, 100 Hz)  : drain commands -> VehicleController::step -> telemetry out
//   comms   (soft RT,    200 Hz)   : transport <-> CommandGateway
//   media   (best effort)          : MediaRouter::pump
// Only the control thread ever touches vehicle state; comms and media cannot block it.
#include <memory>

#include "sdt/common/clock.hpp"
#include "sdt/common/spsc_queue.hpp"
#include "sdt/comms/command_gateway.hpp"
#include "sdt/media/media_router.hpp"
#include "sdt/runtime/periodic_thread.hpp"
#include "sdt/runtime/vehicle_controller.hpp"

namespace sdt {

struct RuntimeConfig {ig& cfg = {}, MediaRouter* media = nullptr);
  ~VehicleRuntime() { 
  void stop() noexcept;

  LoopStats controlStats() const noexcept { return control_.stats(); }
  LoopStats commsStats() const noexcept { return comms_.stats(); }

 private:
  void controlTick();
  void commsTick();

  VehicleController& controller_;
  const IClock& clock_;
  RuntimeConfig cfg_;
  MediaRouter* media_;
  std::unique_ptr<CommandQueue> cmd_q_;
  SpscQueue<Telemetry, 8> tel_q_;
  CommandGateway gateway_;
  TimeNs last_tel_{kNever};
  PeriodicThread control_;
  PeriodicThread comms_;
  PeriodicThread media_thread_;
};

}  // namespace sdt
