#pragma once
// Threading model. Three loops, isolated by lock-free queues:
//   control (hard-ish RT, 100 Hz)  : drain commands -> VehicleController::step -> telemetry out
//   comms   (soft RT,    200 Hz)   : transport <-> CommandGateway
//   media   (best effort)          : MediaRouter::pump
// Only the control thread ever touches vehicle state; comms and media cannot block it.
#include <memory>

#include "sdt/common/clock.hpp"
#include "sdt/common/spsc_queue.hpp"
#include "sdt/comms/command_gateway.hpp
  PeriodicThread media_thread_