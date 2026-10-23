#include "telematics/net/mqtt_codec.hpp"

namespace telematics::net::mqtt {
namespace {

void putU16(Bytes& b, uint16_t v) {
    b.push_back(static_cast<uint8_t>(v >> 8));
    b.push_back(static_cast<uint8_t>(v & 0xFF));
}

void putString(Bytes& b, std::string_view s) {
    if (s.size() > 0xFFFF) throw ProtocolError("MQTT string longer than 65535 bytes");
    putU16(b, static_cast<uint16_t>(s.size()));
    b.insert(b.end(), s.begin(), s.end());
}

Bytes frame(uint8_t first_byte, const Bytes& body) {
    if (body.size() > kMaxRemainingLength) throw ProtocolError("MQTT packet too large");
    Bytes out;
    out.reserve(body.size() + 5);
    out.push_back(first_byte);
    appendRemainingLength(out, static_cast<uint32_t>(body.size()));
    out.insert(out.end(), body.begin(), body.end());
    return out;
}

// Bounds-checked cursor over a packet body.
struct Cursor {
    const Bytes& b;
    size_t pos = 0;

    uint16_t u16() {
        if (pos + 2 > b.size()) throw ProtocolError("MQTT packet truncated");
        const uint16_t v = static_cast<uint16_t>(b[pos] << 8 | b[pos + 1]);
        pos += 2;
        return v;
    }
    std::string str() {
        const uint16_t n = u16();
        if (pos + n > b.size()) throw ProtocolError("MQTT string exceeds packet");
        std::string s(b.begin() + static_cast<std::ptrdiff_t>(pos), b.begin() + static_cast<std::ptrdiff_t>(pos + n));
        pos += n;
        return s;
    }
};

void checkQos(uint8_t qos) {
    if (qos > 2) throw ProtocolError("invalid MQTT QoS " + std::to_string(qos));
}

}  // namespace

void appendRemainingLength(Bytes& out, uint32_t value) {
    if (value > kMaxRemainingLength) throw ProtocolError("remaining length out of range");
    do {
        uint8_t byte = value % 128;
        value /= 128;
        if (value > 0) byte |= 0x80;
        out.push_back(byte);
    } while (value > 0);
}

std::optional<std::pair<uint32_t, size_t>> decodeRemainingLength(const uint8_t* data, size_t len) {
    uint32_t value = 0, multiplier = 1;
    for (size_t i = 0; i < 4; ++i) {
        if (i >= len) return std::nullopt;
        value += (data[i] & 0x7F) * multiplier;
        if ((data[i] & 0x80) == 0) return std::make_pair(value, i + 1);
        multiplier *= 128;
    }
    throw ProtocolError("malformed remaining length (more than 4 bytes)");
}

bool isValidTopicName(std::string_view t) {
    if (t.empty() || t.size() > 0xFFFF) return false;
    return t.find_first_of("+#") == std::string_view::npos && t.find('\0') == std::string_view::npos;
}

bool isValidTopicFilter(std::string_view f) {
    if (f.empty() || f.size() > 0xFFFF || f.find('\0') != std::string_view::npos) return false;
    size_t start = 0;
    for (;;) {
        const size_t slash = f.find('/', start);
        const std::string_view level = f.substr(start, slash == std::string_view::npos ? f.npos : slash - start);
        if (level.find_first_of("+#") != std::string_view::npos && level.size() != 1) return false;  // "a+" invalid
        if (level == "#" && slash != std::string_view::npos) return false;  // '#' must be last
        if (slash == std::string_view::npos) return true;
        start = slash + 1;
    }
}

Bytes encodeConnect(const ConnectOptions& o) {
    if (o.password && !o.username) throw ProtocolError("MQTT 3.1.1: password requires a username");
    if (o.client_id.empty() && !o.clean_session) throw ProtocolError("empty client id requires clean session");

    Bytes body;
    putString(body, "MQTT");
    body.push_back(0x04);  // protocol level 4 = 3.1.1

    uint8_t flags = 0;
    if (o.clean_session) flags |= 0x02;
    if (o.will) {
        if (!isValidTopicName(o.will->topic)) throw ProtocolError("invalid will topic");
        checkQos(o.will->qos);
        flags |= 0x04 | static_cast<uint8_t>(o.will->qos << 3);
        if (o.will->retain) flags |= 0x20;
    }
    if (o.password) flags |= 0x40;
    if (o.username) flags |= 0x80;
    body.push_back(flags);
    putU16(body, o.keep_alive_s);

    putString(body, o.client_id);
    if (o.will) {
        putString(body, o.will->topic);
        putString(body, o.will->payload);  // binary data uses the same 2-byte length prefix
    }
    if (o.username) putString(body, *o.username);
    if (o.password) putString(body, *o.password);
    return frame(0x10, body);
}

Bytes encodePublish(const PublishMsg& m) {
    if (!isValidTopicName(m.topic)) throw ProtocolError("invalid MQTT topic name '" + m.topic + "'");
    checkQos(m.qos);
    if (m.qos > 0 && m.packet_id == 0) throw ProtocolError("QoS>0 PUBLISH needs a non-zero packet id");

    Bytes body;
    putString(body, m.topic);
    if (m.qos > 0) putU16(body, m.packet_id);
    body.insert(body.end(), m.payload.begin(), m.payload.end());

    const uint8_t first = static_cast<uint8_t>(0x30 | (m.dup ? 0x08 : 0) | (m.qos << 1) | (m.retain ? 1 : 0));
    return frame(first, body);
}

Bytes encodePuback(uint16_t id) {
    Bytes body;
    putU16(body, id);
    return frame(0x40, body);
}

Bytes encodeSubscribe(uint16_t id, const std::vector<std::pair<std::string, uint8_t>>& filters) {
    if (filters.empty()) throw ProtocolError("SUBSCRIBE needs at least one filter");
    Bytes body;
    putU16(body, id);
    for (const auto& [filter, qos] : filters) {
        if (!isValidTopicFilter(filter)) throw ProtocolError("invalid MQTT topic filter '" + filter + "'");
        checkQos(qos);
        putString(body, filter);
        body.push_back(qos);
    }
    return frame(0x82, body);  // SUBSCRIBE fixed-header flags must be 0b0010
}

Bytes encodeUnsubscribe(uint16_t id, const std::vector<std::string>& filters) {
    if (filters.empty()) throw ProtocolError("UNSUBSCRIBE needs at least one filter");
    Bytes body;
    putU16(body, id);
    for (const auto& f : filters) putString(body, f);
    return frame(0xA2, body);
}

Bytes encodePingreq() { return {0xC0, 0x00}; }
Bytes encodeDisconnect() { return {0xE0, 0x00}; }

Connack parseConnack(const Packet& p) {
    if (p.type != PacketType::Connack || p.body.size() != 2) throw ProtocolError("malformed CONNACK");
    return {static_cast<bool>(p.body[0] & 0x01), p.body[1]};
}

PublishMsg parsePublish(const Packet& p) {
    if (p.type != PacketType::Publish) throw ProtocolError("not a PUBLISH packet");
    PublishMsg m;
    m.dup = p.flags & 0x08;
    m.qos = (p.flags >> 1) & 0x03;
    m.retain = p.flags & 0x01;
    if (m.qos == 3) throw ProtocolError("PUBLISH with QoS 3");
    Cursor c{p.body};
    m.topic = c.str();
    if (m.qos > 0) m.packet_id = c.u16();
    m.payload.assign(p.body.begin() + static_cast<std::ptrdiff_t>(c.pos), p.body.end());
    return m;
}

uint16_t parsePacketId(const Packet& p) {
    Cursor c{p.body};
    return c.u16();
}

std::vector<uint8_t> parseSubackCodes(const Packet& p) {
    if (p.type != PacketType::Suback || p.body.size() < 3) throw ProtocolError("malformed SUBACK");
    return {p.body.begin() + 2, p.body.end()};
}

std::string connackReason(uint8_t rc) {
    switch (rc) {
        case 0: return "accepted";
        case 1: return "unacceptable protocol version";
        case 2: return "client identifier rejected";
        case 3: return "server unavailable";
        case 4: return "bad user name or password";
        case 5: return "not authorized";
        default: return "unknown return code " + std::to_string(rc);
    }
}

void PacketReader::feed(const uint8_t* data, size_t len) { buf_.insert(buf_.end(), data, data + len); }

std::optional<Packet> PacketReader::next() {
    if (buf_.size() < 2) return std::nullopt;
    const auto rl = decodeRemainingLength(buf_.data() + 1, buf_.size() - 1);
    if (!rl) return std::nullopt;
    const auto [length, rl_bytes] = *rl;
    if (length > max_) throw ProtocolError("incoming MQTT packet exceeds size limit");
    const size_t header = 1 + rl_bytes;
    if (buf_.size() < header + length) return std::nullopt;

    const uint8_t first = buf_[0];
    const uint8_t type_val = first >> 4;
    if (type_val < 1 || type_val > 14) throw ProtocolError("invalid MQTT packet type 0");

    Packet p{static_cast<PacketType>(type_val), static_cast<uint8_t>(first & 0x0F),
             Bytes(buf_.begin() + static_cast<std::ptrdiff_t>(header),
                   buf_.begin() + static_cast<std::ptrdiff_t>(header + length))};
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(header + length));

    // Reserved flag bits must match the spec (section 2.2.2).
    const bool needs_0010 = p.type == PacketType::Pubrel || p.type == PacketType::Subscribe ||
                            p.type == PacketType::Unsubscribe;
    if (p.type != PacketType::Publish && p.flags != (needs_0010 ? 0x02 : 0x00)) {
        throw ProtocolError("invalid fixed-header flags");
    }
    return p;
}

}  // namespace telematics::net::mqtt
