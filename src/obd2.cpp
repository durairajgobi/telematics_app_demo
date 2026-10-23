#include "telematics/obd2.hpp"

namespace telematics::obd2 {

CanFrame buildRequest(Pid pid, uint64_t timestamp_ms) {
    CanFrame f;
    f.id = kFunctionalRequestId;
    f.dlc = 8;
    f.data = {0x02, kModeCurrentData, static_cast<uint8_t>(pid), 0x55, 0x55, 0x55, 0x55, 0x55};
    f.timestamp_ms = timestamp_ms;
    return f;
}

std::optional<PidReading> decodeResponse(const CanFrame& f) {
    if (f.id < kResponseIdMin || f.id > kResponseIdMax) return std::nullopt;
    if (f.dlc < 3) return std::nullopt;

    const uint8_t len = f.data[0] & 0x0F;  // ISO-TP single-frame length
    if ((f.data[0] >> 4) != 0 || len < 2 || len > 7 || len + 1 > f.dlc) return std::nullopt;
    if (f.data[1] != kModeCurrentData + kPositiveResponseOffset) return std::nullopt;

    const auto pid = static_cast<Pid>(f.data[2]);
    const uint8_t payload = len - 2;  // bytes after mode + pid
    const double A = f.data[3];
    const double B = f.data[4];

    auto need = [&](uint8_t n) { return payload >= n; };

    switch (pid) {
        case Pid::EngineLoad:
            if (!need(1)) break;
            return PidReading{pid, A * 100.0 / 255.0, "%"};
        case Pid::CoolantTemp:
            if (!need(1)) break;
            return PidReading{pid, A - 40.0, "degC"};
        case Pid::EngineRpm:
            if (!need(2)) break;
            return PidReading{pid, (256.0 * A + B) / 4.0, "rpm"};
        case Pid::VehicleSpeed:
            if (!need(1)) break;
            return PidReading{pid, A, "km/h"};
        case Pid::IntakeAirTemp:
            if (!need(1)) break;
            return PidReading{pid, A - 40.0, "degC"};
        case Pid::ThrottlePosition:
            if (!need(1)) break;
            return PidReading{pid, A * 100.0 / 255.0, "%"};
        case Pid::FuelLevel:
            if (!need(1)) break;
            return PidReading{pid, A * 100.0 / 255.0, "%"};
        case Pid::BatteryVoltage:
            if (!need(2)) break;
            return PidReading{pid, (256.0 * A + B) / 1000.0, "V"};
    }
    return std::nullopt;
}

std::string pidName(Pid pid) {
    switch (pid) {
        case Pid::EngineLoad: return "engine_load";
        case Pid::CoolantTemp: return "coolant_temp";
        case Pid::EngineRpm: return "engine_rpm";
        case Pid::VehicleSpeed: return "vehicle_speed";
        case Pid::IntakeAirTemp: return "intake_air_temp";
        case Pid::ThrottlePosition: return "throttle_position";
        case Pid::FuelLevel: return "fuel_level";
        case Pid::BatteryVoltage: return "battery_voltage";
    }
    return "unknown";
}

}  // namespace telematics::obd2
