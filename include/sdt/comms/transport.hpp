#pragma once
// Byte-stream transport port. Concrete adapters (UDP, TCP, serial/LoRa radio, a WebSocket/gRPC
// bridge to the dashboard backend) implement this and nothing else in the core changes.
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <utility>

namespace sdt {

class ITransport {
 public:
  virtual ~ITransport() = default;
  /// Non-blocking. Returns the number of bytes copied into `buf` (0 = nothing available).
  virtual std::size_t receive(std::uint8_t* buf, std::size_t cap) noexcept = 0;
  /// Non-blocking. Returns false if the bytes could not be queued.
  virtual bool send(const std::uint8_t* data, std::size_t n) noexcept = 0;
};

/// In-process duplex pipe for tests and software-in-the-loop runs.
class LoopbackTransport final : public ITransport {
 public:
  Lo::sha