#pragma once
// Transport-independent message model (dashboard <-> vehicle).
#include <cstdint>
#include <variant>

#include "sdt/common/types.hpp"
#include "sdt/nav/mission.hpp"
#include "sdt/nav/mission_executor.hpp"
#include "sdt/safety/safety_supervisor.hpp"

namespace sdt