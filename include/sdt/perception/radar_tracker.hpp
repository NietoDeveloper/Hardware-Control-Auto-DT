#pragma once
// Radar clustering + multi-target alpha-beta tracker. Fixed capacity, zero heap allocation.
#include <array>
#include <cstddef>
#include <cstdint>

#include "sdt/common/types.hpp"
#include "sdt/hal/ports.hpp"

namespace sdt {

struct Obstacle {
  std::uint32_t id{0};
  explicit RadarTracker(const TrackerConfig& cfg = {}) noexcept;

  /// Fuse one scan. `own` must be the vehicle state at (approximately) scan time.
  void update(const RadarScan& scan, const VehicleState& own) noexcept;

  /// Copies confirmed tracks to `out`; returns how many.
  std::size_t confirmed(ObstacleList& out) const noexcept;
  std::size_t activeCount() const noexcept;
  void reset() noexcept;

 private:
  struct Track {
    Obstacle obs;
    Vec3 pred;
    double dt{0.0};
    bool active{false};
    bool matched{false};
  };
  TrackerConfig cfg_;
  std::array<Track, kMaxTracks> tracks_{};
  std::uint32_t next_id_{1};
};

}  // namespace sdt
