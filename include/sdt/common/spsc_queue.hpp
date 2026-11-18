#pragma once
// Wait-free single-producer / single-consumer ring buffer. Capacity is a compile-time power of two.
// push()/pop() never allocate, never block, never throw (T must be nothrow-copy-assignable).
#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace sdt {

template <class T, std::size_t N>
class SpscQueue {
  static_assert(N >= 2 && (N & (N - 1)) == 0, "capacity must be a power of two");
  static_assert(std::is_nothrow_copy_assignable_v<T>, "T must be nothrow copy assignable");

 public:
  bool push(const T& v) noexcept {
    const std::size_t h = head_.load(std::memory_order_relaxed);
    if (h - tail_.load(std::memory_order_acquire) == N) return false;  // full
    buf_[h & (N - 1)] = v;
    head_.store(h + 1, std::memory_order_release);
    return true;
  }

  bool pop(T& out) noexcept {
    const std::size_t t = tail_.load(std::memory_order_relaxed);
    if (t == head_.load(std::memory_order_acquire)) return false;  // empty
    out = buf_[t & (N - 1)];
    tail_.store(t + 1, std::memory_order_release);
    return true;
  }

  std::size_t sizeApprox() const noexcept {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }
  static constexpr std::size_t capacity() noexcept { return N; }

 private:
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
  alignas(64) std::array<T, N> buf_{};
};

}  // namespace sdt
