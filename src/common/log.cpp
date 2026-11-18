#include "sdt/common/log.hpp"

#include <atomic>
#include <cstdio>
#include <mutex>

namespace sdt {
namespace {
std::atomic<int> g_level{static_cast<int>(LogLevel::Info)};
std::atomic<LogSink> g_sink{nullptr};
std::mutex g_mutex;

void defaultSink(LogLevel level, std::string_view msg) {
  static constexpr const char* kNames[] = {"DEBUG", "INFO", "WARN", "ERROR", "OFF"};
  std::lock_guard<std::mutex> lock(g_mutex);
  std::fprintf(stderr, "[sdt][%s] %.*s\n", kNames[static_cast<int>(level)],
               static_cast<int>(msg.size()), msg.data());
}
}  // namespace

void setLogLevel(LogLevel level) noexcept { g_level.store(static_cast<int>(level)); }
void setLogSink(LogSink sink) noexcept { g_sink.store(sink); }

void log(LogLevel level, std::string_view message) noexcept {
  if (static_cast<int>(level) < g_level.load()) return;
  LogSink s = g_sink.load();
  if (s != nullptr) {
    s(level, message);
  } else {
    defaultSink(level, message);
  }
}
}  // namespace sdt
