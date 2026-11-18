#pragma once
// Fixed-rate loop on absolute deadlines (no drift), with overrun accounting and optional
// real-time scheduling (SCHED_FIFO + CPU pinning on Linux; best effort, failures are logged).
#include <atomic>
#include <cstdint>
