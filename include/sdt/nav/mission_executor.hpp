#pragma once
#include <cstdint>

#include "sdt/nav/mission.hpp"
#include "sdt/nav/path_follower.hpp"

namespace sdt {

enum class MissionState : std::uint8_t { Idle = 0, Running = 1, Paused = 2, Completed = 3, Aborted = 4 };

/// Deterministic mission state machine. No I/O, no allocation, no clock access.
class MissionExecutor {
 public:

