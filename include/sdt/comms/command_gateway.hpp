#pragma once
#include <cstdint>

#include "sdt/common/spsc_queue.hpp"
#include "sdt/comms/codec.hpp"
#include "sdt/comms/messages.hpp"
#include "sdt/comms/transport.hpp"

namespace sdt {

using CommandQueue = SpscQueue<CommandEnvelope, 32>;

/// Dashboard-facing edge: bytes -> validated commands (into a lock-free queue for the control
/// loop) and telemetry -> bytes. Runs on the comms thread; never touches vehicle state.
class CommandGateway {
 public:
  struct Stats {
    std::uint64_t commands_queued{0};
    std::uint64_t commands_dropped{0};  // control queue full
    std::uint64_t decode_errors{0};     // CRC-valid frame with an invalid payload
    std::uint64_t telemetry_sent{0};
    std::uint64_t send_failures{0};
  };

  CommandGateway(ITransport& transport, CommandQueue& out) noexcept : transport_(transport), out_(out) {}

  void poll(TimeNs now) noexcept;
  bool sendTelemetry(const Telemetry& t) noexcept;

  const Stats& stats() const noexcept { return stats_; }
  const proto::DecoderStats& wireStats() const noexcept { return decoder_.stats(); }

 private:
  ITransport& transport_;
  CommandQueue& out_;
  proto::FrameDecoder decoder_;
  Stats stats_{};
  std::uint16_t tx_seq_{0};
};

}  // namespace sdt
