#pragma once
// Fixed-rate loop on absolute deadlines (no drift), with overrun accounting and optional
// real-time scheduling (SCHED_FIFO + CPU pinning on Linux; best effort, failures are logged).
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include "sdt/common/types.hpp"

namespace sdt {

struct RtConfig {
  int priority{0};  // 0 = default scheduler; 1..99 = SCHED_FIFO priority (needs CAP_SYS_NICE)
  int cpu{-1};      // -1 = no affinity
};

struct LoopStats {
  std::uint64_t cycles{0};
  std::uint64_t overruns{0};         // body finished after the next deadline
  std::int64_t max_exec_ns{0};       // worst body execution time
  std::int64_t max_lateness_ns{0};   // worst wake-up latency vs. the deadline
};

class PeriodicThread {
 public:
  PeriodicThread() = default;
  PeriodicThread(const PeriodicThread&) = delete;
  PeriodicThread& operator=(const PeriodicThread&) = delete;
  ~PeriodicThread() { stop(); }

  bool start(std::string name, TimeNs period, std::function<void()> body, RtConfig rt = {});
  void stop() noexcept;
  LoopStats stats() const noexcept;

 private:
  std::thread th_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> cycles_{0};
  std::atomic<std::uint64_t> overruns_{0};
  std::atomic<std::int64_t> max_exec_{0};
  std::atomic<std::int64_t> max_late_{0};
};

}  // namespace sdt
