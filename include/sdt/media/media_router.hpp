#pragma once
// Video/audio path, deliberately isolated from the control loop: its own thread, its own
// (lower) priority, bounded work per pass, drop-on-backpressure. A stalled encoder or a slow
// network can never delay a control cycle.
#include <array>
#include <cstdint>
