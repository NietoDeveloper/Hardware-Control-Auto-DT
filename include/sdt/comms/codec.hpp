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
