#pragma once

#include <cstddef>
#include <cstdint>

namespace drawn_pattern {
// Numeric values are persisted in the existing 32-bit slot format word.
enum class SampleFormat : uint8_t { Q8 = 1, Q16 = 2, RunsQ16 = 3 };
constexpr size_t sparseSamples = 1024;
}  // namespace drawn_pattern
