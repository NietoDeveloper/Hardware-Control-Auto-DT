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

  void addSource(IMediaSource& s) { sources_.push_back(&s); }
