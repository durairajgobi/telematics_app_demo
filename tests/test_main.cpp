// Dependency-free unit tests. Run: ./tcu_tests
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include "telematics/event_detector.hpp"
#include "telematics/gps.hpp"
#include "telematics/obd2.hpp"
#include "telematics/store_forward.hpp"
#include "telematics/trip_tracker.hpp"

using namespace telematics;

static int g_failures = 0, g_checks = 0;

#define CHECK(cond)                                                                    \
    do {                                                                               \
        ++g_checks;                                                                    \
        if (!(cond)) {                                                                 \
            ++g_failures;                                                              \
            std::cerr << "  FAIL " << __FILE__ << ":" << __LINE__ << "  " #cond "\n";  \
        }                                                                              \
    } while (0)
#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((a) - (b)) <= (eps))

static CanFrame response(std::initializer_list<uint8_t> bytes, uint32_t id = 0x7E8) {
    CanFrame f;
    f.id = id;
    f.dlc = 8;
    size_t i = 0;
    for (auto b : bytes) f.data[i++] = b;
    return f;
}

static void testObd2() {
    auto req = obd2::buildRequest(obd2::Pid::EngineRpm);
    CHECK(req.id == 0x7DF);
    CHECK(req.data[0] == 0x02 && req.data[1] == 0x01 && req.data[2] == 0x0C);

    auto rpm = obd2::decodeResponse(response({0x04, 0x41, 0x0C, 0x1A, 0xF8}));
    CHECK(rpm && rpm->pid == obd2::Pid::EngineRpm);
    CHECK_NEAR(rpm->value, 1726.0, 1e-9);  // (0x1A*256 + 0xF8)/4

    auto spd = obd2::decodeResponse(response({0x03, 0x41, 0x0D, 0x64}));
    CHECK(spd && spd->value == 100.0);

    auto coolant = obd2::decodeResponse(response({0x03, 0x41, 0x05, 0x7B}));
    CHECK(coolant && coolant->value == 83.0);

    auto fuel = obd2::decodeResponse(response({0x03, 0x41, 0x2F, 0xFF}));
    CHECK(fuel && std::fabs(fuel->value - 100.0) < 1e-9);

    auto volts = obd2::decodeResponse(response({0x04, 0x41, 0x42, 0x37, 0x14}));
    CHECK(volts && std::fabs(volts->value - 14.1) < 1e-9);

    CHECK(!obd2::decodeResponse(response({0x03, 0x41, 0x0D, 0x64}, 0x123)));   // wrong CAN id
    CHECK(!obd2::decodeResponse(response({0x03, 0x7F, 0x01, 0x12})));          // negative response
    CHECK(!obd2::decodeResponse(response({0x03, 0x41, 0x0C, 0x1A})));          // RPM needs 2 bytes
    CHECK(!obd2::decodeResponse(response({0x10, 0x14, 0x49, 0x02})));          // multi-frame, unsupported
}

static void testGps() {
    const std::string rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    CHECK(gps::verifyChecksum(rmc));
    CHECK(!gps::verifyChecksum("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6B"));
    CHECK(!gps::verifyChecksum("$GPRMC,123519,A"));

    gps::GpsFix fix;
    CHECK(gps::parseNmea(rmc, fix));
    CHECK(fix.valid);
    CHECK_NEAR(fix.latitude, 48.1173, 1e-4);
    CHECK_NEAR(fix.longitude, 11.516667, 1e-5);
    CHECK_NEAR(fix.speed_kmh, 22.4 * 1.852, 1e-6);
    CHECK_NEAR(fix.course_deg, 84.4, 1e-9);

    const std::string gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    CHECK(gps::parseNmea(gga, fix));
    CHECK(fix.satellites == 8);
    CHECK_NEAR(fix.altitude_m, 545.4, 1e-9);
    CHECK_NEAR(fix.hdop, 0.9, 1e-9);

    // Southern / western hemispheres are negative.
    gps::GpsFix sw;
    const std::string body = "GPRMC,000000,A,3352.000,S,15112.000,W,0.0,0.0,010126,,,A";
    uint8_t sum = 0;
    for (char c : body) sum ^= static_cast<uint8_t>(c);
    char cs[4];
    std::snprintf(cs, sizeof(cs), "%02X", sum);
    CHECK(gps::parseNmea("$" + body + "*" + cs, sw));
    CHECK(sw.latitude < 0 && sw.longitude < 0);

    // London -> Paris ~343.5 km
    CHECK_NEAR(gps::haversineMeters(51.5074, -0.1278, 48.8566, 2.3522) / 1000.0, 343.5, 1.0);
}

static VehicleState st(uint64_t ms, double kmh, double rpm = 1500) {
    VehicleState s;
    s.timestamp_ms = ms;
    s.speed_kmh = kmh;
    s.rpm = rpm;
    s.coolant_c = 90;
    s.fuel_pct = 50;
    s.battery_v = 14;
    return s;
}

static int count(const std::vector<DrivingEvent>& v, EventType t) {
    int n = 0;
    for (auto& e : v) n += e.type == t;
    return n;
}

static void testEvents() {
    EventDetector d;
    d.process(st(0, 60));
    auto ev = d.process(st(1000, 40));  // -20 km/h in 1 s = -5.56 m/s²
    CHECK(count(ev, EventType::HarshBraking) == 1);

    ev = d.process(st(2000, 52));  // +12 km/h/s = 3.33 m/s²
    CHECK(count(ev, EventType::HarshAcceleration) == 1);

    ev = d.process(st(3000, 55));  // gentle
    CHECK(ev.empty());

    // A sustained emergency stop is a single event.
    EventDetector stop;
    int brakes = 0;
    double v = 90;
    stop.process(st(0, v));
    for (uint64_t t = 1000; t <= 4000; t += 1000) brakes += count(stop.process(st(t, v -= 25)), EventType::HarshBraking);
    CHECK(brakes == 1);

    // Overspeed fires once per excursion.
    CHECK(count(d.process(st(4000, 105)), EventType::Overspeed) == 1);
    CHECK(count(d.process(st(5000, 106)), EventType::Overspeed) == 0);
    CHECK(count(d.process(st(6000, 98)), EventType::Overspeed) == 0);
    CHECK(count(d.process(st(7000, 101)), EventType::Overspeed) == 1);

    // Idle: 0 km/h with engine running for > 5 min.
    EventDetector idle;
    int idles = 0;
    for (uint64_t t = 0; t <= 7 * 60'000; t += 1000) idles += count(idle.process(st(t, 0, 800)), EventType::ExcessiveIdle);
    CHECK(idles == 1);

    // Health alerts.
    EventDetector h;
    auto hot = st(0, 50);
    hot.coolant_c = 115;
    hot.fuel_pct = 8;
    hot.battery_v = 11.2;
    ev = h.process(hot);
    CHECK(count(ev, EventType::EngineOverheat) == 1);
    CHECK(count(ev, EventType::LowFuel) == 1);
    CHECK(count(ev, EventType::LowBattery) == 1);
}

static void testTrips() {
    TripTracker tt(10'000);
    CHECK(!tt.update(st(0, 0, 0)));  // engine off: no trip
    CHECK(!tt.inTrip());

    // 10 minutes at a steady 60 km/h with no GPS -> 10 km by integration.
    for (uint64_t t = 1000; t <= 601'000; t += 1000) {
        auto s = st(t, 60);
        s.fuel_pct = 50.0 - (t / 601'000.0) * 2.0;
        CHECK(!tt.update(s));
    }
    CHECK(tt.inTrip());

    // Engine off; trip ends after 10 s.
    std::optional<TripSummary> done;
    for (uint64_t t = 602'000; t <= 615'000 && !done; t += 1000) done = tt.update(st(t, 0, 0));
    CHECK(done.has_value());
    if (done) {
        CHECK_NEAR(done->distance_km, 10.0, 0.05);
        CHECK_NEAR(done->max_speed_kmh, 60.0, 1e-9);
        CHECK_NEAR(done->avg_speed_kmh, 60.0, 0.5);
        CHECK_NEAR(done->fuel_used_pct, 2.0, 0.01);
        CHECK(done->durationSeconds() == 600.0);
    }
    CHECK(!tt.inTrip());
}

class FakeTransport : public ITransport {
public:
    bool up = true;
    std::vector<std::string> received;
    bool send(const std::vector<Message>& b) override {
        if (!up) return false;
        for (auto& m : b) received.push_back(m.payload);
        return true;
    }
};

static void testStoreAndForward() {
    FakeTransport tx;
    StoreAndForward sf(tx, 4, 10);

    tx.up = false;
    sf.enqueue("n1");
    sf.enqueue("n2");
    sf.enqueue("alert", Priority::High);
    CHECK(!sf.flushOnce());
    CHECK(sf.pending() == 3);  // nothing lost while offline

    sf.enqueue("n3");
    sf.enqueue("n4");  // over capacity: evicts oldest Normal ("n1")
    CHECK(sf.pending() == 4);
    CHECK(sf.stats().dropped == 1);

    tx.up = true;
    CHECK(sf.flushOnce());
    CHECK(sf.pending() == 0);
    CHECK(tx.received.size() == 4);
    if (tx.received.size() == 4) {
        CHECK(tx.received[0] == "alert");  // alerts jump the queue
        CHECK(tx.received[1] == "n2");
        CHECK(tx.received[3] == "n4");
    }

    // Background thread delivers everything.
    FakeTransport tx2;
    StoreAndForward sf2(tx2, 1000, 7, std::chrono::milliseconds(1), std::chrono::milliseconds(5));
    sf2.start();
    for (int i = 0; i < 100; ++i) sf2.enqueue("m" + std::to_string(i));
    for (int i = 0; i < 200 && sf2.pending() > 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    sf2.stop();
    CHECK(tx2.received.size() == 100);
    CHECK(sf2.stats().sent == 100);
}

int main() {
    struct { const char* name; void (*fn)(); } tests[] = {
        {"obd2", testObd2}, {"gps", testGps}, {"events", testEvents},
        {"trips", testTrips}, {"store_and_forward", testStoreAndForward},
    };
    for (auto& t : tests) {
        const int before = g_failures;
        t.fn();
        std::cout << (g_failures == before ? "[ OK ] " : "[FAIL] ") << t.name << "\n";
    }
    std::cout << g_checks - g_failures << "/" << g_checks << " checks passed\n";
    return g_failures == 0 ? 0 : 1;
}
