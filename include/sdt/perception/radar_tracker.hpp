#pragma once
// Radar clustering + multi-target alpha-beta tracker. Fixed capacity, zero heap allocation.
#include <array>
#include <cstddef>
#include <cstdint>

#include "sdt/common/types.hpp"
#include "sdt/hal/ports.hpp"

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
