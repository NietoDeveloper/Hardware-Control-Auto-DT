#pragma once
// Binary wire protocol v1 (see docs/PROTOCOL.md).
//   frame := magic(2) version(1) type(1) seq(2) length(2) payload(length) crc32(4)
// All integers little-endian. CRC-32 (IEEE) covers version..payload. No heap, no exceptions.
#include <cstddef>
#include <cstdint>

