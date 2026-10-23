#include "telematics/event_detector.hpp"

namespace telematics {

void EventDetector::reset() {
    prev_.reset();
    idle_since_ms_.reset();
    idle_reported_ = braking_ = accelerating_ = overspeed_ = overheat_ = low_fuel_ = low_battery_ = false;
}

void EventDetector::latch(bool condition, bool& active, EventType type, double value,
                          const VehicleState& s, std::vector<DrivingEvent>& out) {
    if (condition && !active) {
        out.push_back({type, s.timestamp_ms, value, s.gps.latitude, s.gps.longitude});
    }
    active = condition;
}

std::vector<DrivingEvent> EventDetector::process(const VehicleState& s) {
    std::vector<DrivingEvent> out;

    // Longitudinal acceleration from consecutive speed samples.
    if (prev_ && s.timestamp_ms > prev_->timestamp_ms) {
        const uint64_t dt_ms = s.timestamp_ms - prev_->timestamp_ms;
        if (dt_ms >= t_.min_dt_ms) {
            const double dv_mps = (s.speed_kmh - prev_->speed_kmh) / 3.6;
            const double accel = dv_mps / (static_cast<double>(dt_ms) / 1000.0);
            // One event per manoeuvre: a 3-second emergency stop is one harsh brake, not three.
            latch(accel <= t_.harsh_brake_mps2, braking_, EventType::HarshBraking, accel, s, out);
            latch(accel >= t_.harsh_accel_mps2, accelerating_, EventType::HarshAcceleration, accel, s, out);
        }
    }

    latch(s.speed_kmh > t_.speed_limit_kmh, overspeed_, EventType::Overspeed, s.speed_kmh, s, out);
    latch(s.coolant_c > t_.overheat_c, overheat_, EventType::EngineOverheat, s.coolant_c, s, out);
    latch(s.fuel_pct > 0.0 && s.fuel_pct < t_.low_fuel_pct, low_fuel_, EventType::LowFuel, s.fuel_pct, s, out);
    latch(s.battery_v > 0.0 && s.battery_v < t_.low_battery_v, low_battery_, EventType::LowBattery, s.battery_v, s, out);

    // Excessive idling: engine on, vehicle stationary, for longer than idle_max_ms.
    const bool idling = s.rpm >= t_.idle_rpm_min && s.speed_kmh <= t_.idle_speed_max_kmh;
    if (idling) {
        if (!idle_since_ms_) idle_since_ms_ = s.timestamp_ms;
        const uint64_t idle_ms = s.timestamp_ms - *idle_since_ms_;
        if (!idle_reported_ && idle_ms >= t_.idle_max_ms) {
            out.push_back({EventType::ExcessiveIdle, s.timestamp_ms, idle_ms / 1000.0, s.gps.latitude, s.gps.longitude});
            idle_reported_ = true;
        }
    } else {
        idle_since_ms_.reset();
        idle_reported_ = false;
    }

    prev_ = s;
    return out;
}

}  // namespace telematics
