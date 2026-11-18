#include "sdt/comms/transport.hpp"

#include <algorithm>

namespace sdt {
namespace {
constexpr std::size_t kPipeCapacity = 1u << 20;
}

std::pair<std::unique_ptr<LoopbackTransport>, std::unique_ptr<LoopbackTransport>>
LoopbackTransport::makePair() {
  auto a = std::make_shared<Pipe>();
  auto b = std::make_shared<Pipe>();
  return {std::unique_ptr<LoopbackTransport>(new LoopbackTransport(a, b)),
          std::unique_ptr<LoopbackTransport>(new LoopbackTransport(b, a))};
}

std::size_t LoopbackTransport::receive(std::uint8_t* buf, std::size_t cap) noexcept {
  std::lock_guard<std::mutex> lock(rx_->m);
  const std::size_t n = std::min(cap, rx_->q.size());
  for (std::size_t i = 0; i < n; ++i) {
    buf[i] = rx_->q.front();
    rx_->q.pop_front();
  }
  return n;
}

bool LoopbackTransport::send(const std::uint8_t* data, std::size_t n) noexcept {
  std::lock_guard<std::mutex> lock(tx_->m);
  if (tx_->q.size() + n > kPipeCapacity) return false;
  tx_->q.insert(tx_->q.end(), data, data + n);
  return true;
}

}  // namespace sdt
