#pragma once
// Snapshot of the vehicle, fused from OBD-II and GPS, plus JSON serialisation.
#include <cstdint>
#include <string>

#include "telematics/gps.hpp"

namespace telematics {

struct VehicleState {
    uint64_t timestamp_ms = 0;
    double speed_kmh = 0.0;
    double rpm = 0.0;
    double coolant_c = 0.0;
    double throttle_pct = 0.0;
    double fuel_pct = 0.0;
    double engine_load_pct = 0.0;
    double battery_v = 0.0;
    gps::GpsFix gps;
};

enum class EventType {
    HarshBraking,
    HarshAcceleration,
    Overspeed,
    ExcessiveIdle,
    EngineOverheat,
    LowFuel,
    LowBattery,
};

struct DrivingEvent {
    EventType type;
    uint64_t timestamp_ms;
    double value;  // the measurement that triggered it
    double latitude;
    double longitude;
};

std::string eventName(EventType t);

std::string toJson(const VehicleState& s, const std::string& vin);
std::string toJson(const DrivingEvent& e, const std::string& vin);

}  // namespace telematics
