#pragma once
// Rule-based driver-behaviour and vehicle-health event detection.
#include <optional>
#include <vector>

#include "telematics/vehicle_state.hpp"

namespace telematics {

struct EventThresholds {
    double harsh_brake_mps2 = -3.5;   // deceleration (negative)
    double harsh_accel_mps2 = 3.0;    // acceleration
    double speed_limit_kmh = 100.0;
    double idle_rpm_min = 400.0;      // engine running
    double idle_speed_max_kmh = 2.0;  // vehicle stationary
    uint64_t idle_max_ms = 5 * 60 * 1000;
    double overheat_c = 110.0;
    double low_fuel_pct = 10.0;
    double low_battery_v = 11.8;
    uint64_t min_dt_ms = 200;         // ignore samples closer than this for accel
};

class EventDetector {
public:
    explicit EventDetector(EventThresholds t = {}) : t_(t) {}

    // Feed a new state; returns any events raised by this sample.
    // Level-type events (overspeed, overheat, ...) fire once on entry and
    // re-arm only after the condition clears, so they do not spam.
    std::vector<DrivingEvent> process(const VehicleState& s);

    void reset();

private:
    void latch(bool condition, bool& active, EventType type, double value,
               const VehicleState& s, std::vector<DrivingEvent>& out);

    EventThresholds t_;
    std::optional<VehicleState> prev_;
    std::optional<uint64_t> idle_since_ms_;
    bool idle_reported_ = false;
    bool braking_ = false;
    bool accelerating_ = false;
    bool overspeed_ = false;
    bool overheat_ = false;
    bool low_fuel_ = false;
    bool low_battery_ = false;
};

}  // namespace telematics
