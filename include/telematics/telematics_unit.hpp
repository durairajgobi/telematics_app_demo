#pragma once
// Telematics Control Unit (TCU): polls OBD-II PIDs, fuses GPS, detects
// events, tracks trips and publishes everything through store-and-forward.
#include <functional>
#include <string>
#include <vector>

#include "telematics/can_bus.hpp"
#include "telematics/event_detector.hpp"
#include "telematics/obd2.hpp"
#include "telematics/store_forward.hpp"
#include "telematics/trip_tracker.hpp"
#include "telematics/vehicle_state.hpp"

namespace telematics {

struct TcuConfig {
    std::string vin = "MA3EWDE1S00000000";
    uint64_t telemetry_interval_ms = 10'000;  // periodic position/status report
    std::chrono::milliseconds pid_timeout{50};
    std::vector<obd2::Pid> pids = {
        obd2::Pid::VehicleSpeed, obd2::Pid::EngineRpm,   obd2::Pid::CoolantTemp,
        obd2::Pid::ThrottlePosition, obd2::Pid::FuelLevel, obd2::Pid::EngineLoad,
        obd2::Pid::BatteryVoltage,
    };
    EventThresholds thresholds;
};

class TelematicsUnit {
public:
    TelematicsUnit(ICanBus& bus, StoreAndForward& uplink, TcuConfig cfg = {});

    // Poll all configured PIDs and refresh the cached vehicle state.
    void pollObd(uint64_t now_ms);
    // Feed one raw NMEA sentence from the GNSS receiver.
    void onNmea(const std::string& sentence);
    // Run detection, trip tracking and reporting for the current state.
    void tick(uint64_t now_ms);
    // Close any open trip and report it (call on shutdown).
    void shutdown();

    const VehicleState& state() const { return state_; }
    int eventsRaised() const { return events_raised_; }
    int tripsCompleted() const { return trips_completed_; }

    // Optional hook for local logging / display.
    std::function<void(const std::string& kind, const std::string& json)> on_publish;

private:
    void publish(const std::string& kind, std::string json, Priority p);

    ICanBus& bus_;
    StoreAndForward& uplink_;
    TcuConfig cfg_;
    VehicleState state_;
    EventDetector detector_;
    TripTracker trips_;
    uint64_t last_report_ms_ = 0;
    bool reported_once_ = false;
    int events_raised_ = 0;
    int trips_completed_ = 0;
};

}  // namespace telematics
