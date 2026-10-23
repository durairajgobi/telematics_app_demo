#include "telematics/gps.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace telematics::gps {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6371008.8;
constexpr double kKnotsToKmh = 1.852;

std::vector<std::string_view> split(std::string_view s, char delim) {
    std::vector<std::string_view> out;
    size_t start = 0;
    for (size_t i = 0; i <= s.size(); ++i) {
        if (i == s.size() || s[i] == delim) {
            out.push_back(s.substr(start, i - start));
            start = i + 1;
        }
    }
    return out;
}

double toDouble(std::string_view s) {
    if (s.empty()) return 0.0;
    std::string tmp(s);
    return std::strtod(tmp.c_str(), nullptr);
}

// NMEA "ddmm.mmmm" / "dddmm.mmmm" -> decimal degrees.
std::optional<double> nmeaToDegrees(std::string_view value, std::string_view hemi) {
    if (value.empty() || hemi.empty()) return std::nullopt;
    const double raw = toDouble(value);
    const double degrees = std::floor(raw / 100.0);
    const double minutes = raw - degrees * 100.0;
    double dec = degrees + minutes / 60.0;
    if (hemi == "S" || hemi == "W") dec = -dec;
    return dec;
}

int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

}  // namespace

bool verifyChecksum(std::string_view s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) s.remove_suffix(1);
    if (s.size() < 4 || s[0] != '$') return false;
    const size_t star = s.rfind('*');
    if (star == std::string_view::npos || star + 3 != s.size()) return false;

    uint8_t sum = 0;
    for (size_t i = 1; i < star; ++i) sum ^= static_cast<uint8_t>(s[i]);

    const int hi = hexVal(s[star + 1]);
    const int lo = hexVal(s[star + 2]);
    if (hi < 0 || lo < 0) return false;
    return sum == static_cast<uint8_t>(hi * 16 + lo);
}

bool parseNmea(std::string_view s, GpsFix& fix) {
    if (!verifyChecksum(s)) return false;
    const size_t star = s.rfind('*');
    const auto f = split(s.substr(1, star - 1), ',');
    if (f.empty() || f[0].size() < 5) return false;

    const std::string_view type = f[0].substr(2);  // skip talker id (GP, GN, ...)

    if (type == "RMC") {
        // $GPRMC,time,status,lat,N,lon,E,speed_kn,course,date,...
        if (f.size() < 10) return false;
        if (f[2] != "A") {  // V = void / no fix
            fix.valid = false;
            return true;
        }
        auto lat = nmeaToDegrees(f[3], f[4]);
        auto lon = nmeaToDegrees(f[5], f[6]);
        if (!lat || !lon) return false;
        fix.utc_time = std::string(f[1]);
        fix.latitude = *lat;
        fix.longitude = *lon;
        fix.speed_kmh = toDouble(f[7]) * kKnotsToKmh;
        fix.course_deg = toDouble(f[8]);
        fix.valid = true;
        return true;
    }

    if (type == "GGA") {
        // $GPGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,...
        if (f.size() < 10) return false;
        if (toDouble(f[6]) == 0.0) {  // quality 0 = no fix
            fix.valid = false;
            return true;
        }
        auto lat = nmeaToDegrees(f[2], f[3]);
        auto lon = nmeaToDegrees(f[4], f[5]);
        if (!lat || !lon) return false;
        fix.utc_time = std::string(f[1]);
        fix.latitude = *lat;
        fix.longitude = *lon;
        fix.satellites = static_cast<int>(toDouble(f[7]));
        fix.hdop = toDouble(f[8]);
        fix.altitude_m = toDouble(f[9]);
        fix.valid = true;
        return true;
    }

    return false;  // unsupported sentence type
}

double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
    auto rad = [](double d) { return d * kPi / 180.0; };
    const double dLat = rad(lat2 - lat1);
    const double dLon = rad(lon2 - lon1);
    const double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
                     std::cos(rad(lat1)) * std::cos(rad(lat2)) * std::sin(dLon / 2) * std::sin(dLon / 2);
    return 2.0 * kEarthRadiusM * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

}  // namespace telematics::gps
