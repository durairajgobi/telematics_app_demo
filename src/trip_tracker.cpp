#include "telematics/trip_tracker.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include "telematics/gps.hpp"

namespace telematics {
namespace {
constexpr double kEngineOnRpm = 300.0;
constexpr double kMovingKmh = 2.0;
constexpr double kMaxGpsJumpM = 500.0;  // reject GPS teleports between samples
}  // namespace

std::optional<TripSummary> TripTracker::update(const VehicleState& s) {
    const bool engine_on = s.rpm >= kEngineOnRpm;

    if (!active_) {
        if (!engine_on) return std::nullopt;
        active_ = true;
        current_ = TripSummary{};
        current_.start_ms = current_.end_ms = s.timestamp_ms;
        start_fuel_ = last_fuel_ = s.fuel_pct;
        moving_time_s_ = 0.0;
        engine_off_since_.reset();
        last_ = s;
        return std::nullopt;
    }

    if (last_ && s.timestamp_ms > last_->timestamp_ms) {
        const double dt_s = (s.timestamp_ms - last_->timestamp_ms) / 1000.0;

        // Prefer GPS distance; fall back to integrating wheel speed.
        double step_km = 0.0;
        if (s.gps.valid && last_->gps.valid) {
            const double m = gps::haversineMeters(last_->gps.latitude, last_->gps.longitude,
                                                  s.gps.latitude, s.gps.longitude);
            if (m < kMaxGpsJumpM) step_km = m / 1000.0;
        } else {
            step_km = (s.speed_kmh + last_->speed_kmh) / 2.0 * dt_s / 3600.0;
        }
        current_.distance_km += step_km;

        if (s.speed_kmh > kMovingKmh) moving_time_s_ += dt_s;
        else if (engine_on) current_.idle_time_s += dt_s;
    }

    current_.max_speed_kmh = std::max(current_.max_speed_kmh, s.speed_kmh);
    // Fuel senders read unreliably with the engine off, so sample only while running.
    if (engine_on && s.fuel_pct > 0.0) last_fuel_ = s.fuel_pct;
    last_ = s;

    if (engine_on) {
        engine_off_since_.reset();
        current_.end_ms = s.timestamp_ms;
        return std::nullopt;
    }

    if (!engine_off_since_) engine_off_since_ = s.timestamp_ms;
    if (s.timestamp_ms - *engine_off_since_ >= end_after_off_ms_) return finish();
    return std::nullopt;
}

std::optional<TripSummary> TripTracker::finish() {
    if (!active_) return std::nullopt;
    active_ = false;
    current_.fuel_used_pct = std::max(0.0, start_fuel_ - last_fuel_);
    current_.avg_speed_kmh = moving_time_s_ > 0 ? current_.distance_km / (moving_time_s_ / 3600.0) : 0.0;
    last_.reset();
    return current_;
}

std::string toJson(const TripSummary& t, const std::string& vin) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(2);
    o << "{\"type\":\"trip\",\"vin\":\"" << vin << "\""
      << ",\"start\":" << t.start_ms << ",\"end\":" << t.end_ms
      << ",\"duration_s\":" << t.durationSeconds()
      << ",\"distance_km\":" << t.distance_km
      << ",\"max_speed_kmh\":" << t.max_speed_kmh
      << ",\"avg_speed_kmh\":" << t.avg_speed_kmh
      << ",\"idle_s\":" << t.idle_time_s
      << ",\"fuel_used_pct\":" << t.fuel_used_pct
      << ",\"harsh_events\":" << t.harsh_events << "}";
    return o.str();
}

}  // namespace telematics
