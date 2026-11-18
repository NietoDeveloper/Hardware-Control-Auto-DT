#pragma once
// Minimal leveled logger. NOT real-time safe: never call from the control loop's hot path.
// The control loop reports through counters and telemetry instead.
#include <string_view>

namespace sdt {

enum class LogLevel : int { Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4 };
using LogSink = void (*)(LogLevel, std::string_view);

void setLogLevel(LogLevel level) noexcept;
void setLogSink(LogSink sink) noexcept;  // nullptr restores the default stderr sink
void log(LogLevel level, std::string_view message) noexcept;

}  // namespace sdt
