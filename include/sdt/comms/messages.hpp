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
/// Normalised stick input in [-1, 1]. forward/lateral are body-relative (lateral + = left).
struct CmdManual { float forward{0}, lateral{0}, vertical{0}, yaw{0}; };
struct CmdMissionUpload { Mission mission; };
struct CmdMissionControl { MissionAction action{MissionAction::Abort}; };

using Command = std::variant<CmdHeartbeat, CmdArm, CmdSetMode, CmdManual, CmdMissionUpload,
                             CmdMissionControl>;

/// A decoded command plu
  Command comman
struct Telemetry {
  TimeNs stamp{0};
  Vec3 position;
  Vec3 velocity;
  double yaw{0.0};
  bool armed{false};
  ControlMode mode{ControlMode::Hold};
  SafetyState safety{SafetyState::Nominal};
  MissionState mission_state{MissionState::Idle};
  std::uint16_t mission_index{0};
  std::uint32_t faults{0};
  std::uint16_t tracks{0};
  std::uint32_t rejected_commands{0};
};

}  // namespace sdt
