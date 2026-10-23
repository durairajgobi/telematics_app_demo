#pragma once
// MQTT 3.1.1 (OASIS standard) packet encoding and decoding.
// Pure functions with no I/O, so they are easy to unit-test and reuse.
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "telematics/net/stream.hpp"

namespace telematics::net::mqtt {

using Bytes = std::vector<uint8_t>;

enum class PacketType : uint8_t {
    Connect = 1, Connack, Publish, Puback, Pubrec, Pubrel, Pubcomp,
    Subscribe, Suback, Unsubscribe, Unsuback, Pingreq, Pingresp, Disconnect,
};

constexpr uint32_t kMaxRemainingLength = 268'435'455;  // 4-byte variable-length integer limit

struct Will {
    std::string topic;
    std::string payload;
    uint8_t qos = 1;
    bool retain = true;
};

struct ConnectOptions {
    std::string client_id;                 // e.g. the VIN or device serial
    std::optional<std::string> username;
    std::optional<std::string> password;   // also used for SAS / JWT tokens
    uint16_t keep_alive_s = 60;
    bool clean_session = true;
    std::optional<Will> will;              // "last will" published by the broker if we vanish
};

struct PublishMsg {
    std::string topic;
    std::string payload;
    uint8_t qos = 0;
    bool retain = false;
    bool dup = false;
    uint16_t packet_id = 0;  // required for QoS > 0
};

struct Packet {
    PacketType type;
    uint8_t flags;  // low nibble of the fixed header
    Bytes body;     // variable header + payload
};

struct Connack {
    bool session_present;
    uint8_t return_code;  // 0 = accepted
};

// Variable-length "remaining length" field.
void appendRemainingLength(Bytes& out, uint32_t value);
// Returns {value, bytes used}; nullopt if more bytes are needed. Throws ProtocolError if malformed.
std::optional<std::pair<uint32_t, size_t>> decodeRemainingLength(const uint8_t* data, size_t len);

// Client -> broker
Bytes encodeConnect(const ConnectOptions& opts);
Bytes encodePublish(const PublishMsg& msg);
Bytes encodePuback(uint16_t packet_id);
Bytes encodeSubscribe(uint16_t packet_id, const std::vector<std::pair<std::string, uint8_t>>& filters);
Bytes encodeUnsubscribe(uint16_t packet_id, const std::vector<std::string>& filters);
Bytes encodePingreq();
Bytes encodeDisconnect();

// Broker -> client
Connack parseConnack(const Packet& p);
PublishMsg parsePublish(const Packet& p);
uint16_t parsePacketId(const Packet& p);  // PUBACK, SUBACK, UNSUBACK ...
std::vector<uint8_t> parseSubackCodes(const Packet& p);  // 0x80 = failure
std::string connackReason(uint8_t return_code);

bool isValidTopicName(std::string_view topic);     // for PUBLISH: no wildcards
bool isValidTopicFilter(std::string_view filter);  // for SUBSCRIBE: + and # rules

// Incremental framer: feed raw bytes from the stream, pop complete packets.
class PacketReader {
public:
    explicit PacketReader(size_t max_packet_size = 1 << 20) : max_(max_packet_size) {}
    void feed(const uint8_t* data, size_t len);
    std::optional<Packet> next();  // throws ProtocolError on malformed input
    void reset() { buf_.clear(); }

private:
    Bytes buf_;
    size_t max_;
};

}  // namespace telematics::net::mqtt
