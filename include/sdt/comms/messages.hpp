#pragma once
// Transport-independent message model (dashboard <-> vehicle).
#include <cstdint>
#include <variant>

#include "sdt/common/types.hpp"
#include "sdt/nav/mission.hpp"
#include "sdt/nav/mission_executor.hpp"
#include "sdt/safety/safety_supervisor.hpp"

namespace sdt {

enum class MissionAction : std::uint8_t { Start = 1, Pause = 2, Resume = 3, Abort = 4 };

struct CmdHeartbeat {};
struct CmdArm { bool arm{false}; };
struct CmdSetMode { ControlMode mode{ControlMode::Hold}; };
/// Normalised stick input in [-1, 1]
  std::uint16_t tracks{0};
  std::u