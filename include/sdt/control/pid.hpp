#pragma once
#include "sdt/common/types.hpp"

namespace sdt {

struct PidGains {
  double kp{1.0};
  double ki{0.0};
  double kd{0.0};
  double out_min{-1.0};
  double out_max{1.0};
  double i_max{1.0};  // integrator clamp
};

/// PID with conditional-integration anti-windup. Header-only, allocation-free.
class Pid {

}  // namespace sdt
