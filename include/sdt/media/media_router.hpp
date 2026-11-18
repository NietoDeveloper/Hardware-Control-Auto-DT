#pragma once
// Video/audio path, deliberately isolated from the control loop: its own thread, its own
// (lower) priority, bounded work per pass, drop-on-backpressure. A stalled encoder or a slow
// network can never delay a control cycle.
#include <array>
#include <cstdint>
#include <vector>

#include "sdt/hal/ports.hpp"

namespace sdt {

class MediaRouter {
 public:
  struct Stats {
    std::uint64_t frames_in{0};
    std::uint64_t frames_out{0};
    std::uint64_t frames_dropped{0};
    std::uint64_t keyframe_requests{0};
  };

  void addSource(IMediaSource& s) { sources_.push_back(&s); }
  void setSink(IMediaSink& sink) noexcept { sink_ = &sink; }

 private:
  std::vector<IMediaSource*> sources_;
  IMediaSink* sink_{nullptr};
  std::array<bool, 256> wait_key_{};  // per stream: dropped a frame, discard deltas until a keyframe
  Stats stats_{};
};

}  // namespace sdt
