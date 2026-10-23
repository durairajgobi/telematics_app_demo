#pragma once
// CAN bus abstraction with a Linux SocketCAN implementation and a simulated
// vehicle ECU for development without hardware.
#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <random>
#include <string>

#include "telematics/can_frame.hpp"

namespace telematics {

class ICanBus {
public:
    virtual ~ICanBus() = default;
    virtual bool write(const CanFrame& frame) = 0;
    // Waits up to `timeout` for a frame.
    virtual std::optional<CanFrame> read(std::chrono::milliseconds timeout) = 0;
};

#ifdef __linux__
// Real CAN interface, e.g. "can0" (hardware) or "vcan0" (virtual).
// Setup a virtual bus for testing:
//   sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
class SocketCanBus : public ICanBus {
public:
    explicit SocketCanBus(const std::string& ifname);  // throws std::runtime_error
    ~SocketCanBus() override;
    SocketCanBus(const SocketCanBus&) = delete;
    SocketCanBus& operator=(const SocketCanBus&) = delete;

    bool write(const CanFrame& frame) override;
    std::optional<CanFrame> read(std::chrono::milliseconds timeout) override;

private:
    int fd_ = -1;
};
#endif

// Simulated engine ECU: answers OBD-II Mode 01 requests from a physical
// drive-cycle model (accelerate, cruise, overspeed, hard stop, idle, park).
class SimulatedVehicleBus : public ICanBus {
public:
    explicit SimulatedVehicleBus(uint32_t seed = 42);

    bool write(const CanFrame& frame) override;
    std::optional<CanFrame> read(std::chrono::milliseconds timeout) override;

    // Advance the vehicle model by dt (simulated time).
    void step(double dt_s);
    double timeSeconds() const { return t_s_; }
    double speedKmh() const { return speed_kmh_; }
    bool engineOn() const { return engine_on_; }
    double headingDeg() const { return heading_deg_; }
    double distanceMeters() const { return distance_m_; }

private:
    double targetSpeed() const;

    std::deque<CanFrame> rx_;
    std::mt19937 rng_;
    double t_s_ = 0.0;
    double speed_kmh_ = 0.0;
    double rpm_ = 0.0;
    double coolant_c_ = 25.0;
    double throttle_pct_ = 0.0;
    double fuel_pct_ = 62.0;
    double heading_deg_ = 45.0;
    double distance_m_ = 0.0;
    bool engine_on_ = false;
};

}  // namespace telematics
