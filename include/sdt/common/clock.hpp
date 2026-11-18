#pragma once
#include <atomic>
#include <chrono>

#include "sdt/common/types.hpp"

namespace sdt {

/// Time is always injected. Core logic never reads a wall clock directly, which makes every
/// behaviour reproducible in software-in-the-loop tests.
class IClock {
 public:
  virtual ~IClock() = default;
  virtual TimeNs now() const noexcept = 0;
};

class SteadyClock final : public IClock {
 public:
  TimeNs now() const noexcept override {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
};

class ManualClock final : public IClock {
 public:
  explicit ManualClock(TimeNs start = 0) noexcept : t_(start) {}
  TimeNs now() const noexcept override { return t_.load(std::memory_order_acquire); }
  void advance(TimeNs dt) noexcept { t_.fetch_add(dt, std::memory_order_acq_rel); }
  void set(TimeNs t) noexcept { t_.store(t, std::memory_order_release); }

 private:
  std::atomic<TimeNs> t_;
};

}  // namespace sdt
