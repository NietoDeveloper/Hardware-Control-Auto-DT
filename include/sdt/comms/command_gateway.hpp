#pragma once
#include <cstdint>

#include "sdt/common/spsc_queue.hpp"
#include "sdt/comms/codec.hpp"
#include "sdt/comms/messages.hpp"
#include "sdt/comms/transport.hpp"

namespace sdt {

using CommandQueue = SpscQueue<CommandEnvelope, 32>;

/// Dashboard-facing edge: bytes
  struct Stats {
    std::uint64_t commands_queued{0};
    std::uint64_t commands_dropped{0};  // control queue full
    std::uint64_t decode_e
  std::uint16_t tx_seq_{0};
};

}  // namespace