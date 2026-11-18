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
  Vec3 position;  // world frame
  Vec3 velocity;  // world frame
  double radius_m{1.0};
  std::uint16_t hits{0};
  std::uint1
  double min_range_m{0.3};
  double max_range_m{150.0};
  double min_snr_db{6.0};
  double cluster_m{1.5};    // detections closer than this merge into one measurement
  double gate_m{3.0};       // association gate around the predicted track position
  double alpha{0.5};        // position gain
  double beta{0.05};        // velocity gain
  std::uint16_t confirm_hits{3};
  std::uint16_t max_misses{6};  // scans a track may coast without a measurement
  double default_radius_m{1.0};
};

class RadarTracker {
 public:
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
