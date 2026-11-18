#include "sdt/runtime/periodic_thread.hpp"

#include <chrono>

#include "sdt/common/log.hpp"

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

namespace sdt {
namespace {
using Clock = std::chrono::steady_clock;

void applyRt(const std::string& name, const RtConfig& rt) {
#if defined(__linux__)
  pthread_setname_np(pthread_self(), name.substr(0, 15).c_str());
  if (rt.priority > 0) {
    sched_param sp{};
    sp.sched_priority = rt.priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      log(LogLevel::Warn, "SCHED_FIFO unavailable (need CAP_SYS_NICE); running at default priority");
    }
  }
  if (rt.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(static_cast<std::size_t>(rt.cpu), &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof set, &set) != 0) {
      log(LogLevel::Warn, "CPU pinning failed");
    }
  }
#else
  (void)name;
  (void)rt;
#endif
}

void raiseMax(std::atomic<std::int64_t>& a, std::int64_t v) noexcept {
  std::int64_t cur = a.load(std::memory_order_relaxed);
  while (v > cur && !a.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {
  }
}
}  // namespace

bool PeriodicThread::start(std::string name, TimeNs period, std::function<void()> body, RtConfig rt) {
  if (running_.exchange(true) || period <= 0 || !body) {
    return false;
  }
  th_ = std::thread([this, name = std::move(name), period, body = std::move(body), rt] {
    applyRt(name, rt);
    const auto step = std::chrono::nanoseconds(period);
    auto next = Clock::now() + step;
    while (running_.load(std::memory_order_acquire)) {
      const auto t0 = Clock::now();
      body();
      const auto t1 = Clock::now();
      cycles_.fetch_add(1, std::memory_order_relaxed);
      raiseMax(max_exec_, std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
      if (t1 > next) {
        overruns_.fetch_add(1, std::memory_order_relaxed);
        next = t1 + step;  // skip missed slots instead of bursting
        continue;
      }
      std::this_thread::sleep_until(next);
      raiseMax(max_late_, std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - next).count());
      next += step;
    }
  });
  return true;
}

void PeriodicThread::stop() noexcept {
  running_.store(false, std::memory_order_release);
  if (th_.joinable()) th_.join();
}

LoopStats PeriodicThread::stats() const noexcept {
  return {cycles_.load(), overruns_.load(), max_exec_.load(), max_late_.load()};
}

}  // namespace sdt
