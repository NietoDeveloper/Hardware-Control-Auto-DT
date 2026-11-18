#pragma once
// Hardware abstraction layer = the ONLY boundary between the core and the physical world.
// Everything below is a port (interface). Vendor SDKs, autopilots, CAN/serial drivers, camera
// stacks and simulators live behind these in adapter libraries — never inside the core.
//
// Contract for all poll() methods: non-blocking, wait-free in the common case, called from
// exactly one thread. Return true iff `out` holds NEW data.
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sdt/common/types.hpp"

namespace sdt {

// ---- Navigation state ----------------------------------------------------------------------
clabool poll(VehicleState& out) noexce
class IRadarSource {
 public:final, safety-filtered setpoint.
  virtual void apply(const VelocityCommand& cmd) noexcept = 0;
  /// Vehicle-specific safe termination (ground: cut drive + brake; aerial: land/disarm/parachute).
  /// Must be idempotent and must not block.
  virtual void emergencyStop() noexcept = 0;
};

// ---- Media (video / audio) -----------------------------------------------------------------
enum class MediaKind : std::uint8_t { Video = 0, Audio = 1 };
enum class Codec : std::uint8_t { Raw = 0, H264 = 1, H265 = 2, Opus = 3 };

struct MediaFrame {
  std::uint8_t stream_id{0};
  MediaKind kind{MediaKind::Video};
  Codec codec{Codec::H264};
  TimeNs stamp{0};
  bool keyframe{false};
  std::vector<std::uint8_t> payload;
};

class IMediaSource {
 public:
  virtual ~IMediaSource() = default;
  virtual bool poll(MediaFrame& out) = 0;
  virtual std::uint8_t streamId() const noexcept = 0;
  /// Ask the encoder for an IDR/keyframe (used after the sink dropped frames).
  virtual void requestKeyframe() noexcept {}
};

class IMediaSink {
 public:
  virtual ~IMediaSink() = default;
  /// Return false if the frame could not be accepted (backpressure); the router then drops and
  /// resynchronises on the next keyframe. Must not block.
  virtual bool publish(MediaFrame&& frame) = 0;
};

}  // namespace sdt
