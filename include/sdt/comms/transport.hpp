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
l ~ITransport() = default;
  /// 