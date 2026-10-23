#include "telematics/can_bus.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "telematics/obd2.hpp"

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace telematics {

// ---------------------------------------------------------------- SocketCAN
#ifdef __linux__
SocketCanBus::SocketCanBus(const std::string& ifname) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error("socket(PF_CAN) failed: " + std::string(std::strerror(errno)));

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(fd_);
        throw std::runtime_error("unknown CAN interface '" + ifname + "'");
    }

    // Only accept OBD-II response IDs 0x7E8-0x7EF.
    can_filter filter{};
    filter.can_id = obd2::kResponseIdMin;
    filter.can_mask = 0x7F8;
    ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd_);
        throw std::runtime_error("bind to '" + ifname + "' failed: " + std::strerror(errno));
    }
}

SocketCanBus::~SocketCanBus() {
    if (fd_ >= 0) ::close(fd_);
}

bool SocketCanBus::write(const CanFrame& f) {
    can_frame raw{};
    raw.can_id = f.id > 0x7FF ? (f.id | CAN_EFF_FLAG) : f.id;
    raw.can_dlc = std::min<uint8_t>(f.dlc, 8);
    std::memcpy(raw.data, f.data.data(), raw.can_dlc);
    return ::write(fd_, &raw, sizeof(raw)) == static_cast<ssize_t>(sizeof(raw));
}

std::optional<CanFrame> SocketCanBus::read(std::chrono::milliseconds timeout) {
    pollfd pfd{fd_, POLLIN, 0};
    if (::poll(&pfd, 1, static_cast<int>(timeout.count())) <= 0) return std::nullopt;

    can_frame raw{};
    if (::read(fd_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw))) return std::nullopt;

    CanFrame f;
    f.id = (raw.can_id & CAN_EFF_FLAG) ? (raw.can_id & CAN_EFF_MASK) : (raw.can_id & CAN_SFF_MASK);
    f.dlc = std::min<uint8_t>(raw.can_dlc, 8);
    std::memcpy(f.data.data(), raw.data, f.dlc);
    f.timestamp_ms = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                               std::chrono::steady_clock::now().time_since_epoch())
                                               .count());
    return f;
}
#endif

// ------------------------------------------------------------ Simulated ECU
SimulatedVehicleBus::SimulatedVehicleBus(uint32_t seed) : rng_(seed) {}

// Drive cycle (seconds): park → start → urban → highway (with overspeed)
// → hard stop → long idle → short drive → engine off.
double SimulatedVehicleBus::targetSpeed() const {
    const double t = t_s_;
    if (t < 5) return 0;
    if (t < 60) return 45;
    if (t < 120) return 30;
    if (t < 200) return 95;
    if (t < 240) return 118;   // overspeed
    if (t < 300) return 90;
    if (t < 303) return 0;     // emergency stop
    if (t < 680) return 0;     // idling at a stop
    if (t < 760) return 50;
    return 0;
}

void SimulatedVehicleBus::step(double dt_s) {
    t_s_ += dt_s;
    engine_on_ = t_s_ >= 3.0 && t_s_ < 775.0;

    const double target = engine_on_ ? targetSpeed() : 0.0;
    std::normal_distribution<double> noise(0.0, 0.4);

    // Acceleration limits (km/h per second): normal ~2.5 m/s², emergency stop ~8 m/s².
    const bool emergency = t_s_ >= 300 && t_s_ < 303;
    const double max_up = 9.0, max_down = emergency ? 29.0 : 9.0;
    const double diff = target - speed_kmh_;
    const double dv = std::clamp(diff, -max_down * dt_s, max_up * dt_s);
    speed_kmh_ = std::max(0.0, speed_kmh_ + dv + (target > 0 ? noise(rng_) * dt_s : 0.0));

    throttle_pct_ = engine_on_ ? std::clamp(15.0 + dv / dt_s * 6.0 + speed_kmh_ * 0.15, 0.0, 100.0) : 0.0;

    if (!engine_on_) {
        rpm_ = 0;
    } else if (speed_kmh_ < 2.0) {
        rpm_ = 780 + noise(rng_) * 20;
    } else {
        const double gear_ratio = speed_kmh_ < 20 ? 110 : speed_kmh_ < 40 ? 70 : speed_kmh_ < 60 ? 48 : 32;
        rpm_ = std::clamp(speed_kmh_ * gear_ratio * 0.9, 900.0, 6000.0);
    }

    // Coolant warms towards 90 °C when running, cools towards ambient when off.
    const double coolant_target = engine_on_ ? 90.0 : 25.0;
    coolant_c_ += (coolant_target - coolant_c_) * (engine_on_ ? 0.02 : 0.005) * dt_s;

    if (engine_on_) fuel_pct_ = std::max(0.0, fuel_pct_ - (0.0004 + throttle_pct_ * 0.00003) * dt_s);

    const double moved = speed_kmh_ / 3.6 * dt_s;
    distance_m_ += moved;
    if (speed_kmh_ > 5) heading_deg_ = std::fmod(heading_deg_ + noise(rng_) * 0.5 * dt_s + 360.0, 360.0);
}

bool SimulatedVehicleBus::write(const CanFrame& req) {
    if (req.id != obd2::kFunctionalRequestId || req.data[1] != obd2::kModeCurrentData) return false;
    if (!engine_on_) return true;  // ECU asleep: no response (realistic timeout)

    const auto pid = static_cast<obd2::Pid>(req.data[2]);
    CanFrame r;
    r.id = 0x7E8;
    r.dlc = 8;
    r.timestamp_ms = static_cast<uint64_t>(t_s_ * 1000.0);
    r.data = {0x03, 0x41, req.data[2], 0x00, 0x00, 0x55, 0x55, 0x55};

    auto pct = [](double p) { return static_cast<uint8_t>(std::clamp(p * 255.0 / 100.0 + 0.5, 0.0, 255.0)); };

    switch (pid) {
        case obd2::Pid::EngineLoad: r.data[3] = pct(throttle_pct_ * 0.8 + 10); break;
        case obd2::Pid::CoolantTemp: r.data[3] = static_cast<uint8_t>(std::clamp(coolant_c_ + 40.0, 0.0, 255.0)); break;
        case obd2::Pid::EngineRpm: {
            const auto raw = static_cast<uint16_t>(std::clamp(rpm_ * 4.0, 0.0, 65535.0));
            r.data[0] = 0x04;
            r.data[3] = raw >> 8;
            r.data[4] = raw & 0xFF;
            break;
        }
        case obd2::Pid::VehicleSpeed: r.data[3] = static_cast<uint8_t>(std::clamp(speed_kmh_ + 0.5, 0.0, 255.0)); break;
        case obd2::Pid::IntakeAirTemp: r.data[3] = 30 + 40; break;
        case obd2::Pid::ThrottlePosition: r.data[3] = pct(throttle_pct_); break;
        case obd2::Pid::FuelLevel: r.data[3] = pct(fuel_pct_); break;
        case obd2::Pid::BatteryVoltage: {
            const auto mv = static_cast<uint16_t>(engine_on_ ? 14100 : 12400);
            r.data[0] = 0x04;
            r.data[3] = mv >> 8;
            r.data[4] = mv & 0xFF;
            break;
        }
        default:
            // Negative response: 7F 01 12 (sub-function not supported)
            r.data = {0x03, 0x7F, 0x01, 0x12, 0x55, 0x55, 0x55, 0x55};
    }
    rx_.push_back(r);
    return true;
}

std::optional<CanFrame> SimulatedVehicleBus::read(std::chrono::milliseconds) {
    if (rx_.empty()) return std::nullopt;
    CanFrame f = rx_.front();
    rx_.pop_front();
    return f;
}

}  // namespace telematics
