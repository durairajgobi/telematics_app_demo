#include "telematics/telematics_unit.hpp"

namespace telematics {

TelematicsUnit::TelematicsUnit(ICanBus& bus, StoreAndForward& uplink, TcuConfig cfg)
    : bus_(bus), uplink_(uplink), cfg_(std::move(cfg)), detector_(cfg_.thresholds) {}

void TelematicsUnit::pollObd(uint64_t now_ms) {
    bool any_response = false;
    for (auto pid : cfg_.pids) {
        if (!bus_.write(obd2::buildRequest(pid, now_ms))) continue;

        // Wait for the matching response, skipping unrelated frames.
        for (int i = 0; i < 4; ++i) {
            auto frame = bus_.read(cfg_.pid_timeout);
            if (!frame) break;
            auto r = obd2::decodeResponse(*frame);
            if (!r || r->pid != pid) continue;
            any_response = true;
            switch (r->pid) {
                case obd2::Pid::VehicleSpeed: state_.speed_kmh = r->value; break;
                case obd2::Pid::EngineRpm: state_.rpm = r->value; break;
                case obd2::Pid::CoolantTemp: state_.coolant_c = r->value; break;
                case obd2::Pid::ThrottlePosition: state_.throttle_pct = r->value; break;
                case obd2::Pid::FuelLevel: state_.fuel_pct = r->value; break;
                case obd2::Pid::EngineLoad: state_.engine_load_pct = r->value; break;
                case obd2::Pid::BatteryVoltage: state_.battery_v = r->value; break;
                default: break;
            }
            break;
        }
    }
    // No ECU answered: ignition off. Zero engine values so trips can end.
    if (!any_response) {
        state_.speed_kmh = 0;
        state_.rpm = 0;
        state_.throttle_pct = 0;
    }
    state_.timestamp_ms = now_ms;
}

void TelematicsUnit::onNmea(const std::string& sentence) { gps::parseNmea(sentence, state_.gps); }

void TelematicsUnit::tick(uint64_t now_ms) {
    state_.timestamp_ms = now_ms;

    for (const auto& ev : detector_.process(state_)) {
        ++events_raised_;
        if (ev.type == EventType::HarshBraking || ev.type == EventType::HarshAcceleration) {
            trips_.recordHarshEvent();
        }
        publish("event", toJson(ev, cfg_.vin), Priority::High);
    }

    if (auto trip = trips_.update(state_)) {
        ++trips_completed_;
        publish("trip", toJson(*trip, cfg_.vin), Priority::High);
    }

    // Report every interval while driving; much less often while parked.
    const uint64_t interval = trips_.inTrip() ? cfg_.telemetry_interval_ms : cfg_.telemetry_interval_ms * 30;
    if (!reported_once_ || now_ms - last_report_ms_ >= interval) {
        publish("telemetry", toJson(state_, cfg_.vin), Priority::Normal);
        last_report_ms_ = now_ms;
        reported_once_ = true;
    }
}

void TelematicsUnit::shutdown() {
    if (auto trip = trips_.finish()) {
        ++trips_completed_;
        publish("trip", toJson(*trip, cfg_.vin), Priority::High);
    }
}

void TelematicsUnit::publish(const std::string& kind, std::string json, Priority p) {
    if (on_publish) on_publish(kind, json);
    uplink_.enqueue(std::move(json), p);
}

}  // namespace telematics
