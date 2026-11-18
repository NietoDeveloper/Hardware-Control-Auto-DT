#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "sdt/common/types.hpp"

namespace sdt {

inline constexpr std::size_t kMaxFenceVertices = 32;

/// Polygonal (x,y) fence with an altitude band. Plain data, fixed capacity.
struct Geofence {
  bool enabled{false};
  std::uint8_t vertex_count{0};
  std::array<Vec3, kMaxFenceVertices> vertices{};  // only x,y are used
  double min_z{-1e9};
  double max_z{1e9};

  /// Signed distance to the fence boundary in metres: > 0 inside, < 0 outside.
  /// Returns +inf when disabled or malformed (fail-open is explicit and visible in config).
  double margin(const Vec3& p) const noexcept;
};

}  // namespace sdt
