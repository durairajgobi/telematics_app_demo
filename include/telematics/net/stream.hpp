#pragma once
// Byte-stream abstraction shared by plain TCP and TLS connections, so the
// MQTT and HTTP clients work unchanged over either.
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>

namespace telematics::net {

using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

struct NetError : std::runtime_error {
    using std::runtime_error::runtime_error;
};
struct TimeoutError : NetError {
    using NetError::NetError;
};
struct ProtocolError : NetError {
    using NetError::NetError;
};

class IStream {
public:
    virtual ~IStream() = default;

    // Writes every byte or throws (NetError / TimeoutError).
    virtual void writeAll(const void* data, size_t len, milliseconds timeout) = 0;

    // Returns >0 bytes read, or 0 when the peer closed the connection cleanly.
    // Throws TimeoutError if nothing arrives in time, NetError on failure.
    virtual size_t readSome(void* buf, size_t len, milliseconds timeout) = 0;

    virtual void close() = 0;
    virtual bool isOpen() const = 0;
};

inline void writeString(IStream& s, std::string_view data, milliseconds timeout) {
    s.writeAll(data.data(), data.size(), timeout);
}

// Reads exactly `len` bytes within `timeout`; throws on EOF or timeout.
void readExact(IStream& s, void* buf, size_t len, milliseconds timeout);

// Milliseconds left until `deadline` (0 if passed).
inline milliseconds remaining(Clock::time_point deadline) {
    const auto left = std::chrono::duration_cast<milliseconds>(deadline - Clock::now());
    return left.count() > 0 ? left : milliseconds(0);
}

}  // namespace telematics::net
