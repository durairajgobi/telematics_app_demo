// Telematics Control Unit demo.
//
//   ./tcu_app                     simulated 13-minute drive (no hardware needed)
//   ./tcu_app --can vcan0 [--nmea /dev/ttyACM0] [--seconds 60]
//                                 live mode: real OBD-II over SocketCAN, NMEA from a GNSS serial port
//
// Uplink (default: write telemetry.jsonl locally):
//   --uplink mqtts://broker.example.com:8883 --ca root-ca.pem --cert device.crt --key device.key
//   --uplink https://api.example.com/v1/ingest --token <bearer> [--ca root-ca.pem]
#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "telematics/net/transports.hpp"
#include "telematics/telematics_unit.hpp"

using namespace telematics;

namespace {

// Offline stand-in for a cloud uplink: appends each message to a JSONL file.
class FileTransport : public ITransport {
public:
    explicit FileTransport(const std::string& path) : out_(path) {}
    bool send(const std::vector<Message>& batch) override {
        for (const auto& m : batch) out_ << m.payload << '\n';
        out_.flush();
        return true;
    }

private:
    std::ofstream out_;
};

// Wraps any transport and simulates loss of cellular coverage when
// `connected` is false, so store-and-forward buffering can be observed.
class CoverageSimulator : public ITransport {
public:
    explicit CoverageSimulator(ITransport& inner) : inner_(inner) {}
    bool send(const std::vector<Message>& batch) override {
        if (!connected) return false;
        const bool ok = inner_.send(batch);
        if (ok) ++batches;
        return ok;
    }
    std::atomic<bool> connected{true};
    std::atomic<int> batches{0};

private:
    ITransport& inner_;
};

struct Options {
    std::string can_if, nmea, uplink, ca, cert, key, token;
    std::string vin = TcuConfig{}.vin;
    int seconds = 60;
};

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::unique_ptr<ITransport> makeTransport(const Options& o) {
    if (o.uplink.empty()) return std::make_unique<FileTransport>("telemetry.jsonl");

    const net::Url url = net::Url::parse(o.uplink);
    std::shared_ptr<net::TlsContext> tls;
    if (url.secure()) {
        net::TlsConfig cfg;
        if (!o.ca.empty()) cfg.ca_pem = readFile(o.ca);
        if (!o.cert.empty()) cfg.cert_pem = readFile(o.cert);
        if (!o.key.empty()) cfg.key_pem = readFile(o.key);
        tls = net::TlsContext::client(cfg);
    }
    auto log = [](const std::string& e) { std::cerr << "  uplink error: " << e << "\n"; };

    if (url.scheme == "mqtt" || url.scheme == "mqtts") {
        net::MqttClientConfig cfg;
        cfg.host = url.host;
        cfg.port = url.port;
        cfg.tls = tls;
        cfg.connect.client_id = o.vin;
        if (!o.token.empty()) {  // token-style auth (e.g. Azure SAS, JWT) in the password field
            cfg.connect.username = o.vin;
            cfg.connect.password = o.token;
        }
        auto t = std::make_unique<net::MqttTransport>(cfg, "vehicles/" + o.vin);
        t->on_error = log;
        return t;
    }
    auto t = std::make_unique<net::HttpsTransport>(o.uplink, tls, o.token);
    t->on_error = log;
    return t;
}

std::string withChecksum(const std::string& body) {
    uint8_t sum = 0;
    for (char c : body) sum ^= static_cast<uint8_t>(c);
    char buf[8];
    std::snprintf(buf, sizeof(buf), "*%02X", sum);
    return "$" + body + buf;
}

std::string toNmeaCoord(double deg, bool is_lat) {
    const char hemi = is_lat ? (deg >= 0 ? 'N' : 'S') : (deg >= 0 ? 'E' : 'W');
    deg = std::fabs(deg);
    const int d = static_cast<int>(deg);
    const double m = (deg - d) * 60.0;
    char buf[32];
    std::snprintf(buf, sizeof(buf), is_lat ? "%02d%07.4f,%c" : "%03d%07.4f,%c", d, m, hemi);
    return buf;
}

// Synthesises the RMC + GGA pair a GNSS receiver would emit each second.
std::pair<std::string, std::string> makeNmea(double t_s, double lat, double lon, double kmh, double course) {
    const int secs = static_cast<int>(t_s);
    char time[32];
    std::snprintf(time, sizeof(time), "%02d%02d%02d.00", 9 + secs / 3600, (secs / 60) % 60, secs % 60);
    char tail[64];

    std::snprintf(tail, sizeof(tail), ",%.1f,%.1f,151026,,,A", kmh / 1.852, course);
    const std::string rmc = withChecksum("GPRMC," + std::string(time) + ",A," + toNmeaCoord(lat, true) + "," +
                                         toNmeaCoord(lon, false) + tail);
    const std::string gga = withChecksum("GPGGA," + std::string(time) + "," + toNmeaCoord(lat, true) + "," +
                                         toNmeaCoord(lon, false) + ",1,09,0.9,920.0,M,-86.0,M,,");
    return {rmc, gga};
}

void printPublish(const std::string& kind, const std::string& json) {
    if (kind != "telemetry") std::cout << "  [" << kind << "] " << json << "\n";
}

int runSimulation(ITransport& uplink_transport, const std::string& vin) {
    std::cout << "Simulated drive: urban -> highway (overspeed) -> emergency stop -> long idle -> park\n\n";

    SimulatedVehicleBus bus;
    CoverageSimulator transport(uplink_transport);
    StoreAndForward uplink(transport, 5000, 25, std::chrono::milliseconds(5), std::chrono::milliseconds(40));
    TcuConfig cfg;
    cfg.vin = vin;
    TelematicsUnit tcu(bus, uplink, cfg);
    tcu.on_publish = printPublish;
    uplink.start();

    double lat = 12.9716, lon = 77.5946;  // start position
    double prev_dist = 0.0;
    constexpr double kPi = 3.14159265358979323846;

    for (int sec = 0; sec <= 900; ++sec) {
        bus.step(1.0);
        const uint64_t now_ms = static_cast<uint64_t>(bus.timeSeconds() * 1000);

        // Dead-reckon a GPS position from the vehicle model.
        const double moved = bus.distanceMeters() - prev_dist;
        prev_dist = bus.distanceMeters();
        lat += moved * std::cos(bus.headingDeg() * kPi / 180) / 111'320.0;
        lon += moved * std::sin(bus.headingDeg() * kPi / 180) / (111'320.0 * std::cos(lat * kPi / 180));

        // Cellular dead zone between t=150s and t=260s: data must be buffered.
        transport.connected = !(sec >= 150 && sec < 260);
        if (sec == 150) std::cout << "  ~~ entering cellular dead zone (uplink offline) ~~\n";
        if (sec == 260) std::cout << "  ~~ coverage restored, " << uplink.pending() << " messages buffered ~~\n";

        auto [rmc, gga] = makeNmea(bus.timeSeconds(), lat, lon, bus.speedKmh(), bus.headingDeg());
        tcu.onNmea(rmc);
        tcu.onNmea(gga);
        tcu.pollObd(now_ms);
        tcu.tick(now_ms);

        // Give the uplink thread a moment so the dead-zone retry/backoff is visible.
        std::this_thread::sleep_for(std::chrono::microseconds(300));
    }

    tcu.shutdown();
    uplink.stop(/*drain=*/true);

    const auto s = uplink.stats();
    std::cout << "\nSummary\n"
              << "  events raised     : " << tcu.eventsRaised() << "\n"
              << "  trips completed   : " << tcu.tripsCompleted() << "\n"
              << "  messages enqueued : " << s.enqueued << "\n"
              << "  messages delivered: " << s.sent << " in " << transport.batches << " batches\n"
              << "  failed attempts   : " << s.failed_attempts << " (retried with back-off)\n"
              << "  dropped           : " << s.dropped << "\n"
              << "  output            : uplink (telemetry.jsonl when no --uplink is given)\n";
    return s.sent == s.enqueued ? 0 : 1;
}

#ifdef __linux__
int runLive(const std::string& ifname, const std::string& nmea_path, int seconds, ITransport& transport,
            const std::string& vin) {
    std::unique_ptr<SocketCanBus> bus;
    try {
        bus = std::make_unique<SocketCanBus>(ifname);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }
    StoreAndForward uplink(transport);
    TcuConfig cfg;
    cfg.vin = vin;
    TelematicsUnit tcu(*bus, uplink, cfg);
    tcu.on_publish = [](const std::string& kind, const std::string& json) {
        std::cout << "[" << kind << "] " << json << "\n";
    };
    uplink.start();

    std::ifstream gnss;
    if (!nmea_path.empty()) gnss.open(nmea_path);

    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
        const auto now_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
        std::string line;
        while (gnss.is_open() && gnss.rdbuf()->in_avail() > 0 && std::getline(gnss, line)) tcu.onNmea(line);
        tcu.pollObd(now_ms);
        tcu.tick(now_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    tcu.shutdown();
    uplink.stop(true);
    return 0;
}
#endif

}  // namespace

int main(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(a + " needs a value");
            return argv[++i];
        };
        try {
            if (a == "--can") o.can_if = value();
            else if (a == "--nmea") o.nmea = value();
            else if (a == "--seconds") o.seconds = std::stoi(value());
            else if (a == "--uplink") o.uplink = value();
            else if (a == "--ca") o.ca = value();
            else if (a == "--cert") o.cert = value();
            else if (a == "--key") o.key = value();
            else if (a == "--token") o.token = value();
            else if (a == "--vin") o.vin = value();
            else throw std::runtime_error("unknown option " + a);
        } catch (const std::exception& e) {
            std::cerr << "error: " << e.what() << "\n"
                      << "usage: " << argv[0]
                      << " [--can IFACE [--nmea DEVICE] [--seconds N]]\n"
                         "       [--uplink mqtts://host:8883|https://host/path] [--ca FILE] [--cert FILE] [--key FILE]\n"
                         "       [--token TOKEN] [--vin VIN]\n";
            return 2;
        }
    }

    std::unique_ptr<ITransport> transport;
    try {
        transport = makeTransport(o);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }

    if (o.can_if.empty()) return runSimulation(*transport, o.vin);
#ifdef __linux__
    return runLive(o.can_if, o.nmea, o.seconds, *transport, o.vin);
#else
    std::cerr << "SocketCAN live mode is only available on Linux\n";
    return 2;
#endif
}
