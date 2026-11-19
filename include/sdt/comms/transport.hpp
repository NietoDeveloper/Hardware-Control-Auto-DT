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
  virtual std