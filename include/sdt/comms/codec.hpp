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
