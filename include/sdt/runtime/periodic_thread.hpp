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
