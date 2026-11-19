#pragma once
#include <cstdint>

#include "sdt/common/spsc_queue.hpp"
#include "sdt/comms/codec.hpp"
#include "sdt/comms/messages.hpp"
#include "sdt/comms/transport.hpp"

namespace sdt {

using CommandQueue = SpscQueue<CommandEnvelope, 32>;

/// Dashboard-facing edge: bytes
nt64_t commands_queued{0};
 