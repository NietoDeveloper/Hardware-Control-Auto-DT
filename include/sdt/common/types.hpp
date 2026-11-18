#pragma once
// Fundamental value types. Local frame is ENU (x = East, y = North, z = Up), SI units.
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

namespace sdt {

using TimeNs = std::int64_t;  // monotonic nanoseconds

inline constexpr TimeNs kNsPerMs = 1'000'000;
inline constexpr TimeNs kNsPerSec = 1'000'000'000;
/// Sentinel for "event never happened". Far enough from INT64_MIN to survive subtraction.
inline constexpr TimeNs kNever = std::numeric_limits<TimeNs>::min() / 4;
inline constexpr double kInf = std::numeric_limits<double>::infinity();

constexpr TimeNs millis(std::int64_t ms) noexcept { return ms * kNsPerMs; }
constexpr double toSeconds(TimeNs t) noexcept { return static_cast<double>(t) * 1e-9; }
constexpr TimeNs fromSeconds(double s) noexcept { return static_cast<TimeNs>(s * 1e9); }
constexpr double clampd(double v, double lo, double hi) noexcept {
  return v < lo ? lo : (v > hi ? hi : v);
}
inline double wrapPi(double a) noexcept { return std::remainder(a, 2.0 * std::numbers::pi); }

/// Age of an event; kNever maps to "infinitely old".
constexpr TimeNs ageOf(TimeNs now, TimeNs last) noexcept {
  return last == kNever ? std::numeric_limits<TimeNs>::max() : now - last;
}

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr Vec3 operator+(const Vec3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
  constexpr Vec3 operator-(const Vec3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
  constexpr Vec3 operator*(double k) const noexcept { return {x * k, y * k, z * k}; }
  constexpr Vec3& operator+=(const Vec3& o) noexcept {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
  constexpr double dot(const Vec3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }
  double norm() const noexcept { return std::sqrt(dot(*this)); }
  double norm2d() const noexcept { return std::hypot(x, y); }
  constexpr Vec3 horizontal() const noexcept { return {x, y, 0.0}; }
  bool finite() const noexcept { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};
constexpr Vec3 operator*(double k, const Vec3& v) noexcept { return v * k; }

enum class VehicleKind : std::uint8_t { Ground = 0, Aerial = 1 };
enum class ControlMode : std::uint8_t { Hold = 0, Manual = 1, Mission = 2, ReturnHome = 3 };

/// Fused state estimate (produced by the platform's estimator, consumed read-only by the core).
struct VehicleState {
  TimeNs stamp{0};
  Vec3 position;  // m
  Vec3 velocity;  // m/s
  double yaw{0.0};       // rad, CCW from East
  double yaw_rate{0.0};  // rad/s
  bool finite() const noexcept {
    return position.finite() && velocity.finite() && std::isfinite(yaw) && std::isfinite(yaw_rate);
  }
};

/// Platform-agnostic setpoint: world-frame velocity + yaw rate. Platform adapters (HAL) translate
/// it to throttle/steering, attitude targets, motor mixing, etc.
struct VelocityCommand {
  Vec3 velocity;
  double yaw_rate{0.0};
};

}  // namespace sdt
