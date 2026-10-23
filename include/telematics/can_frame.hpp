#pragma once
// Classic CAN 2.0 frame (11-bit or 29-bit ID, up to 8 data bytes).
#include <array>
#include <cstdint>

namespace telematics {

struct CanFrame {
    uint32_t id = 0;
    uint8_t dlc = 0;                  // data length code (0..8)
    std::array<uint8_t, 8> data{};    // payload, zero-initialised
    uint64_t timestamp_ms = 0;        // receive time
};

}  // namespace telematics
