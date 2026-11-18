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
