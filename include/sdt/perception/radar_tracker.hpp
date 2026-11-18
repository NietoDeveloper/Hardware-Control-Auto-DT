#pragma once
// Radar clustering + multi-target alpha-beta tracker. Fixed capacity, zero heap allocation.
#include <array>
#include <cstddef>
#include <cstdint>

#include "sdt/common/types.hpp"
#include "sdt/hal/ports.hpp"
