#pragma once
// Threading model. Three loops, isolated by lock-free queues:
//   control (hard-ish RT, 100 Hz)  : drain commands -> VehicleController::step -> telemetry out
