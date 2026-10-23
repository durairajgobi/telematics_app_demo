#pragma once
// NMEA 0183 parsing ($xxRMC, $xxGGA) and geodesic helpers.
#include <optional>
#include <string>
#include <string_view>

namespace telematics::gps {

struct GpsFix {
    double latitude = 0.0;    // decimal degrees, +N
    double longitude = 0.0;   // decimal degrees, +E
    double speed_kmh = 0.0;   // from RMC
    double course_deg = 0.0;  // from RMC
    double altitude_m = 0.0;  // from GGA
    int satellites = 0;       // from GGA
    double hdop = 0.0;        // from GGA
    std::string utc_time;     // hhmmss.ss
    bool valid = false;
};

// Validates the "*hh" checksum. Sentences without a checksum are rejected.
bool verifyChecksum(std::string_view sentence);

// Parses an RMC or GGA sentence into `fix`, updating only the fields that
// sentence carries. Returns false for invalid/unsupported sentences.
bool parseNmea(std::string_view sentence, GpsFix& fix);

// Great-circle distance in metres (haversine, mean Earth radius).
double haversineMeters(double lat1, double lon1, double lat2, double lon2);

}  // namespace telematics::gps
