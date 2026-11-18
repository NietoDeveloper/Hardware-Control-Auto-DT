#pragma once
// Hardware abstraction layer = the ONLY boundary between the core and the physical world.
// Everything below is a port (interface). Vendor SDKs, autopilots, CAN/serial drivers, camera
// stacks and simulators live behind these in adapter libraries — never inside the core.
//
// Contract for all poll() methods: non-blocking, wait-free in the common case, called from
// exactly one thread. Return true iff `out` holds NEW data.
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sdt/common/types.hpp"
class IMediaSource {