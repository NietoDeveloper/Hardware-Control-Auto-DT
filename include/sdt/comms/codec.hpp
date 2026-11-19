#pragma once
// Binary wire protocol v1 (see docs/PROTOCOL.md).
//   frame := magic(2) version(1) type(1) seq(2) length(2) payload(length) crc32(4)
// All integers little-endian. CRC-32 (IEEE) covers version..payload. No heap, no exceptions.
#include <cstddef>
#include <cstdint>

#include "sdt/comms/messages.hpp"

namespace sdt::proto {

inline constexpr std::uint8_t kMagic0 = 0xD7;
inline constexpr std::uint8_t kMagic1 = 0x7A;
inline constexpr std::uint8_t kVersion = 1;
inline constexpr std::size_t kHeaderSize = 8;
inline constexpr std::size_t kCrcSize = 4;
inline constexpr std::size_t kMaxPayload = 4096;
inline constexpr std::size_t kMaxFrame = kHeaderSize + kMaxPay

enum class MsgType : std::uint8_t {
  Heartbeat = 0x01,
  Arm = 0x02,
  SetMode = 0x03,
  ManualControl = 0x04,
  MissionUpload = 0x05,
  MissionControl = 0x06,
  Telemetry = 0x80,
};

std::uint32_t crc32(const std::uint8_t* data, std::size_t n) noexcept;

/// Returns the frame size, or 0 if `cap` is too small / payload too lar
std::size_t encodeFrame(MsgType type, std::uint16_t seq, const std::uint8_t* payload,
                        std::size_t len, std::uint8_t* out, std::size_t cap) noexcept;
std::size_t encodeCommand(const Command& cmd, std::uint16_t seq, std::uint8_t* out,
                          std::size_t cap) noexcept;
std::size_t encodeTelemetry(const Telemetry& t, std::uint16_t seq, std::uint8_t* out,
                            std::size_t cap) noexcept;

/// Strict decoders: wrong length, unkstd::size_t len, Telemetry& out) noexc
struc
  MsgType type{MsgType::Heartbeat};
  std::uint16_t seq{0};
  const std::uint8_t* payload{nullptr};

};

struct DecoderStats {
  std::uint64_t frames{0};
  std::uint64_t crc_errors{0};
  std::uint64_t oversize{0};
  std::uint64_t bad_version{0};
};

/// Incremental stream decoder: survives garbage, truncation and bit errors by re-synchronising on
/// the magic bytes. Fixed internal buffer.
class FrameDecoder {
 public:
  template <class F>
  void feed(const std::uint8_t* data, std::size_t n, F&& on_frame) {
    Frame f;
    for (std::size_t i = 0; i < n; ++i) {
      if (push(data[i], f)) on_frame(f);
    }
  }
  const DecoderStats& stats() const noexcept { return stats_; }

 private:
  enum class St : std::uint8_t { Magic0, Magic1, Header, Payload, Crc };
  bool push(std::uint8_t b, Frame& out) noexcept;
  void resetFrame() noexcept;

  St st_{St::Magic0};
  std::uint8_t hdr_[kHeaderSize - 2]{};
  std::size_t hdr_n_{0};
  std::uint8_t buf_[kMaxPayload]{};
  std::size_t len_{0};
  std::size_t got_{0};
  std::uint8_t crc_[kCrcSize]{};
  std::size_t crc_n_{0};
  DecoderStats stats_{};
};

}  // namespa