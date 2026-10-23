#include "telematics/vehicle_state.hpp"

#include <iomanip>
#include <sstream>

namespace telematics {
namespace {

std::string escape(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (char c : in) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) continue;  // drop control chars
                out += c;
        }
    }
    return out;
}

}  // namespace

std::string eventName(EventType t) {
    switch (t) {
        case EventType::HarshBraking: return "harsh_braking";
        case EventType::HarshAcceleration: return "harsh_acceleration";
        case EventType::Overspeed: return "overspeed";
        case EventType::ExcessiveIdle: return "excessive_idle";
        case EventType::EngineOverheat: return "engine_overheat";
        case EventType::LowFuel: return "low_fuel";
        case EventType::LowBattery: return "low_battery";
    }
    return "unknown";
}

std::string toJson(const VehicleState& s, const std::string& vin) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(2);
    o << "{\"type\":\"telemetry\",\"vin\":\"" << escape(vin) << "\""
      << ",\"ts\":" << s.timestamp_ms
      << ",\"speed_kmh\":" << s.speed_kmh
      << ",\"rpm\":" << s.rpm
      << ",\"coolant_c\":" << s.coolant_c
      << ",\"throttle_pct\":" << s.throttle_pct
      << ",\"fuel_pct\":" << s.fuel_pct
      << ",\"load_pct\":" << s.engine_load_pct
      << ",\"battery_v\":" << s.battery_v;
    if (s.gps.valid) {
        o << std::setprecision(6)
          << ",\"gps\":{\"lat\":" << s.gps.latitude << ",\"lon\":" << s.gps.longitude
          << std::setprecision(1) << ",\"alt_m\":" << s.gps.altitude_m
          << ",\"sats\":" << s.gps.satellites << ",\"hdop\":" << s.gps.hdop << "}";
    } else {
        o << ",\"gps\":null";
    }
    o << "}";
    return o.str();
}

std::string toJson(const DrivingEvent& e, const std::string& vin) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(2);
    o << "{\"type\":\"event\",\"vin\":\"" << escape(vin) << "\""
      << ",\"event\":\"" << eventName(e.type) << "\""
      << ",\"ts\":" << e.timestamp_ms
      << ",\"value\":" << e.value
      << std::setprecision(6) << ",\"lat\":" << e.latitude << ",\"lon\":" << e.longitude << "}";
    return o.str();
}

}  // namespace telematics
