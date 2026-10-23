#pragma once
// OBD-II (SAE J1979) Mode 01 request building and response decoding over CAN (ISO 15765-4).
#include <optional>
#include <string>

#include "telematics/can_frame.hpp"

namespace telematics::obd2 {

constexpr uint32_t kFunctionalRequestId = 0x7DF;  // broadcast to all ECUs
constexpr uint32_t kResponseIdMin = 0x7E8;        // ECU #1 response
constexpr uint32_t kResponseIdMax = 0x7EF;        // ECU #8 response
constexpr uint8_t kModeCurrentData = 0x01;
constexpr uint8_t kPositiveResponseOffset = 0x40;

enum class Pid : uint8_t {
    EngineLoad = 0x04,
    CoolantTemp = 0x05,
    EngineRpm = 0x0C,
    VehicleSpeed = 0x0D,
    IntakeAirTemp = 0x0F,
    ThrottlePosition = 0x11,
    FuelLevel = 0x2F,
    BatteryVoltage = 0x42,  // control module voltage
};

struct PidReading {
    Pid pid;
    double value;
    const char* unit;
};

// Build a single-frame ISO-TP Mode 01 request for the given PID.
CanFrame buildRequest(Pid pid, uint64_t timestamp_ms = 0);

// Decode a single-frame Mode 01 positive response. Returns nullopt for
// frames that are not OBD responses, negative responses, unknown PIDs,
// or responses that are too short.
std::optional<PidReading> decodeResponse(const CanFrame& frame);

std::string pidName(Pid pid);

}  // namespace telematics::obd2
