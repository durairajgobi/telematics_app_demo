#pragma once
// Trip segmentation (ignition-on driving periods) and per-trip statistics.
#include <optional>
#include <string>

#include "telematics/vehicle_state.hpp"

namespace telematics {

struct TripSummary {
    uint64_t start_ms = 0;
    uint64_t end_ms = 0;
    double distance_km = 0.0;
    double max_speed_kmh = 0.0;
    double avg_speed_kmh = 0.0;   // distance / moving time
    double idle_time_s = 0.0;
    double fuel_used_pct = 0.0;
    int harsh_events = 0;

    double durationSeconds() const { return (end_ms - start_ms) / 1000.0; }
};

std::string toJson(const TripSummary& t, const std::string& vin);

class TripTracker {
public:
    // A trip starts when the engine is running and ends after the engine
    // has been off for `end_after_off_ms` (tolerates brief stalls).
    explicit TripTracker(uint64_t end_after_off_ms = 60'000) : end_after_off_ms_(end_after_off_ms) {}

    // Returns a summary when a trip completes on this sample.
    std::optional<TripSummary> update(const VehicleState& s);
    void recordHarshEvent() { if (active_) ++current_.harsh_events; }

    bool inTrip() const { return active_; }
    const TripSummary& current() const { return current_; }

    // Forces the active trip closed (e.g. on shutdown).
    std::optional<TripSummary> finish();

private:
    uint64_t end_after_off_ms_;
    bool active_ = false;
    TripSummary current_;
    std::optional<VehicleState> last_;
    std::optional<uint64_t> engine_off_since_;
    double start_fuel_ = 0.0;
    double last_fuel_ = 0.0;
    double moving_time_s_ = 0.0;
};

}  // namespace telematics
