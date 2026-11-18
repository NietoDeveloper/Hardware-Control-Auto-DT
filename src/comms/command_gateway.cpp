#include "sdt/comms/command_gateway.hpp"

namespace sdt {

void CommandGateway::poll(TimeNs now) noexcept {
  std::uint8_t buf[512];
  for (int pass = 0; pass < 16; ++pass) {
    const std::size_t n = transport_.receive(buf, sizeof buf);
    if (n == 0) break;
    decoder_.feed(buf, n, [&](const proto::Frame& f) {
      if (static_cast<std::uint8_t>(f.type) >= 0x80) return;  // vehicle->ground types are ignored
      CommandEnvelope env;
      env.rx_stamp = now;
      if (!proto::decodeCommand(f.type, f.payload, f.length, env.command)) {
        ++stats_.decode_errors;
        return;
      }
      if (out_.push(env)) {
        ++stats_.commands_queued;
      } else {
        ++stats_.commands_dropped;
      }
    });
  }
}

bool CommandGateway::sendTelemetry(const Telemetry& t) noexcept {
  std::uint8_t buf[proto::kMaxFrame];
  const std::size_t n = proto::encodeTelemetry(t, tx_seq_++, buf, sizeof buf);
  if (n == 0 || !transport_.send(buf, n)) {
    ++stats_.send_failures;
    return false;
  }
  ++stats_.telemetry_sent;
  return true;
}

}  // namespace sdt
