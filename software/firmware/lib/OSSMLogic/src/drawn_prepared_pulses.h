#pragma once

#include <cstdint>

namespace drawn_pattern {
// Only already prepared motor targets cross this boundary. The end phase of
// each row stays in the software FIFO until its individual acceptance.
struct PreparedPulses {
    static constexpr unsigned capacity = 2;
    struct Target { int32_t position; uint16_t crossing; };
    Target targets[capacity]{};
    unsigned count = 0;
};
}
