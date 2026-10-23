#include "telematics/net/mqtt_client.hpp"

#include <algorithm>

namespace telematics::net {

using namespace mqtt;

MqttClient::MqttClient(MqttClientConfig cfg) : cfg_(std::move(cfg)) {}

MqttClient::~MqttClient() {
    try {
        disconnect();
    } catch (...) {
    }
}

bool MqttClient::connected() const {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    return stream_ && stream_->isOpen();
}

void MqttClient::onMessage(MessageHandler h) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    handler_ = std::move(h);
}

void MqttClient::requireConnected() const {
    if (!stream_ || !stream_->isOpen()) throw NetError("MQTT client is not connected");
}

void MqttClient::dropConnection() {
    if (stream_) stream_->close();
    stream_.reset();
    reader_.reset();
    ping_outstanding_ = false;
}

void MqttClient::connect() {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    dropConnection();
    const auto deadline = Clock::now() + cfg_.connect_timeout;

    TcpSocket tcp = TcpSocket::connect(cfg_.host, cfg_.port, cfg_.connect_timeout);
    if (cfg_.tls) {
        stream_ = TlsStream::connect(cfg_.tls, std::move(tcp), cfg_.host, remaining(deadline));
    } else {
        stream_ = std::make_unique<TcpSocket>(std::move(tcp));
    }

    try {
        send(encodeConnect(cfg_.connect));
        auto p = readPacket(deadline);
        if (!p) throw TimeoutError("timed out waiting for CONNACK");
        if (p->type != PacketType::Connack) throw ProtocolError("broker sent a packet before CONNACK");
        const Connack ack = parseConnack(*p);
        if (ack.return_code != 0) {
            throw NetError("MQTT broker refused connection: " + connackReason(ack.return_code));
        }
        session_present_ = ack.session_present;
    } catch (...) {
        dropConnection();
        throw;
    }
}

void MqttClient::disconnect() {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (stream_ && stream_->isOpen()) {
        try {
            send(encodeDisconnect());
            // Graceful close: let the broker close first. Closing a socket that
            // still has unread bytes (e.g. TLS 1.3 session tickets) makes the
            // kernel send a TCP RST, which can discard our DISCONNECT, so the
            // broker would wrongly publish the "offline" last will.
            uint8_t buf[512];
            const auto deadline = Clock::now() + milliseconds(1000);
            while (stream_ && stream_->readSome(buf, sizeof(buf), remaining(deadline)) > 0) {
            }
        } catch (const std::exception&) {
            // timeout, reset or already gone: nothing more to tell the broker
        }
    }
    dropConnection();
}

void MqttClient::send(const Bytes& bytes) {
    requireConnected();
    try {
        stream_->writeAll(bytes.data(), bytes.size(), cfg_.ack_timeout);
    } catch (...) {
        dropConnection();
        throw;
    }
    last_tx_ = Clock::now();
}

std::optional<Packet> MqttClient::readPacket(Clock::time_point deadline) {
    uint8_t buf[4096];
    for (;;) {
        if (auto p = reader_.next()) return p;
        requireConnected();
        size_t n = 0;
        try {
            n = stream_->readSome(buf, sizeof(buf), remaining(deadline));
        } catch (const TimeoutError&) {
            return std::nullopt;
        } catch (...) {
            dropConnection();
            throw;
        }
        if (n == 0) {
            dropConnection();
            throw NetError("MQTT broker closed the connection");
        }
        reader_.feed(buf, n);
    }
}

void MqttClient::dispatch(const Packet& p) {
    switch (p.type) {
        case PacketType::Publish: {
            PublishMsg m = parsePublish(p);
            if (m.qos == 2) throw ProtocolError("QoS 2 delivery is not supported (subscribe with QoS <= 1)");
            if (m.qos == 1) send(encodePuback(m.packet_id));
            if (handler_) handler_(m);
            break;
        }
        case PacketType::Pingresp:
            ping_outstanding_ = false;
            break;
        default:
            break;  // late or duplicate acks are harmless
    }
}

Packet MqttClient::waitFor(PacketType type, uint16_t id, Clock::time_point deadline) {
    for (;;) {
        auto p = readPacket(deadline);
        if (!p) {
            dropConnection();  // the session state is unknown; force a reconnect
            throw TimeoutError("timed out waiting for MQTT acknowledgement");
        }
        if (p->type == type && parsePacketId(*p) == id) return *p;
        dispatch(*p);
    }
}

uint16_t MqttClient::nextPacketId() {
    if (++last_id_ == 0) last_id_ = 1;  // 0 is not a valid packet id
    return last_id_;
}

void MqttClient::publish(const std::string& topic, std::string_view payload, uint8_t qos, bool retain) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (qos > 1) throw ProtocolError("QoS 2 publish is not supported");
    requireConnected();
    keepAlive();

    PublishMsg m;
    m.topic = topic;
    m.payload.assign(payload.begin(), payload.end());
    m.qos = qos;
    m.retain = retain;
    if (qos == 1) m.packet_id = nextPacketId();
    send(encodePublish(m));
    if (qos == 1) waitFor(PacketType::Puback, m.packet_id, Clock::now() + cfg_.ack_timeout);
}

uint8_t MqttClient::subscribe(const std::string& filter, uint8_t qos) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    requireConnected();
    const uint16_t id = nextPacketId();
    send(encodeSubscribe(id, {{filter, std::min<uint8_t>(qos, 1)}}));
    const Packet ack = waitFor(PacketType::Suback, id, Clock::now() + cfg_.ack_timeout);
    const auto codes = parseSubackCodes(ack);
    if (codes.empty() || codes[0] == 0x80) throw NetError("MQTT broker rejected subscription to '" + filter + "'");
    return codes[0];
}

void MqttClient::unsubscribe(const std::string& filter) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    requireConnected();
    const uint16_t id = nextPacketId();
    send(encodeUnsubscribe(id, {filter}));
    waitFor(PacketType::Unsuback, id, Clock::now() + cfg_.ack_timeout);
}

void MqttClient::keepAlive() {
    const auto ka = std::chrono::seconds(cfg_.connect.keep_alive_s);
    if (ka.count() == 0) return;
    const auto now = Clock::now();
    if (ping_outstanding_ && now - ping_sent_ > cfg_.ack_timeout) {
        dropConnection();
        throw TimeoutError("MQTT keep-alive: no PINGRESP from broker");
    }
    // Ping a little before the deadline so the broker never sees us as idle.
    if (!ping_outstanding_ && now - last_tx_ >= ka * 3 / 4) {
        send(encodePingreq());
        ping_outstanding_ = true;
        ping_sent_ = now;
    }
}

void MqttClient::loop(milliseconds timeout) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    requireConnected();
    const auto deadline = Clock::now() + timeout;
    keepAlive();
    while (auto p = readPacket(deadline)) dispatch(*p);
    keepAlive();
}

}  // namespace telematics::net
