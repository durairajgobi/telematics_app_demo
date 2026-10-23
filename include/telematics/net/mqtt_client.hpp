#pragma once
// Synchronous MQTT 3.1.1 client over TCP or TLS.
//
// Supports QoS 0 and QoS 1 publish and subscribe, retained messages, last
// will, keep-alive (PINGREQ/PINGRESP) and dead-connection detection. QoS 2 is
// deliberately not implemented: telemetry pipelines use QoS 1 (at-least-once)
// plus idempotent processing on the server.
//
// Thread-safe: all public calls are serialised by an internal mutex.
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "telematics/net/mqtt_codec.hpp"
#include "telematics/net/tls.hpp"

namespace telematics::net {

struct MqttClientConfig {
    std::string host;
    uint16_t port = 8883;                 // 1883 plain, 8883 TLS
    std::shared_ptr<TlsContext> tls;      // nullptr => plain TCP (testing only!)
    mqtt::ConnectOptions connect;
    milliseconds connect_timeout{10'000};
    milliseconds ack_timeout{10'000};     // CONNACK / PUBACK / SUBACK / PINGRESP
};

class MqttClient {
public:
    using MessageHandler = std::function<void(const mqtt::PublishMsg&)>;

    explicit MqttClient(MqttClientConfig cfg);
    ~MqttClient();

    // Opens TCP (+TLS), sends CONNECT, waits for CONNACK. Throws NetError on failure.
    void connect();
    void disconnect();  // sends DISCONNECT (broker discards the last will)
    bool connected() const;
    bool sessionPresent() const { return session_present_; }

    // QoS 0: fire-and-forget. QoS 1: returns once the broker's PUBACK arrives.
    // Any failure closes the connection and throws, so the caller can reconnect.
    void publish(const std::string& topic, std::string_view payload, uint8_t qos = 1, bool retain = false);

    // Returns the granted QoS. Throws if the broker rejects the subscription.
    uint8_t subscribe(const std::string& filter, uint8_t qos = 1);
    void unsubscribe(const std::string& filter);

    // Services incoming messages and keep-alive for up to `timeout`.
    // Call this regularly (e.g. every second) when not publishing.
    void loop(milliseconds timeout);

    void onMessage(MessageHandler h);

private:
    void send(const mqtt::Bytes& bytes);
    std::optional<mqtt::Packet> readPacket(Clock::time_point deadline);
    mqtt::Packet waitFor(mqtt::PacketType type, uint16_t id, Clock::time_point deadline);
    void dispatch(const mqtt::Packet& p);
    void keepAlive();
    uint16_t nextPacketId();
    void dropConnection();
    void requireConnected() const;

    MqttClientConfig cfg_;
    std::unique_ptr<IStream> stream_;
    mqtt::PacketReader reader_;
    MessageHandler handler_;
    mutable std::recursive_mutex mu_;
    uint16_t last_id_ = 0;
    bool session_present_ = false;
    Clock::time_point last_tx_{};
    Clock::time_point ping_sent_{};
    bool ping_outstanding_ = false;
};

}  // namespace telematics::net
