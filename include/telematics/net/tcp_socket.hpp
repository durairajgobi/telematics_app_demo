#pragma once
// TCP/IP client and listener sockets (POSIX: Linux, macOS, QNX).
// Non-blocking under the hood so every operation honours a timeout —
// essential on cellular links where a connect can otherwise hang for minutes.
#include <cstdint>
#include <string>

#include "telematics/net/stream.hpp"

namespace telematics::net {

// Waits until `fd` is ready for `events` (POLLIN/POLLOUT). False on timeout.
bool waitReady(int fd, short events, Clock::time_point deadline);

class TcpSocket : public IStream {
public:
    TcpSocket() = default;
    explicit TcpSocket(int fd, std::string peer = {});
    ~TcpSocket() override;

    TcpSocket(TcpSocket&& other) noexcept;
    TcpSocket& operator=(TcpSocket&& other) noexcept;
    TcpSocket(const TcpSocket&) = delete;
    TcpSocket& operator=(const TcpSocket&) = delete;

    // Resolves `host` (DNS, IPv4 and IPv6) and connects to the first address
    // that answers. Enables TCP_NODELAY and TCP keep-alive.
    static TcpSocket connect(const std::string& host, uint16_t port, milliseconds timeout);

    void writeAll(const void* data, size_t len, milliseconds timeout) override;
    size_t readSome(void* buf, size_t len, milliseconds timeout) override;
    void close() override;
    bool isOpen() const override { return fd_ >= 0; }

    int fd() const { return fd_; }
    const std::string& peer() const { return peer_; }

private:
    void configure();

    int fd_ = -1;
    std::string peer_;
};

class TcpListener {
public:
    // Binds and listens. Port 0 picks a free port (see port()).
    explicit TcpListener(const std::string& bind_addr = "127.0.0.1", uint16_t port = 0, int backlog = 16);
    ~TcpListener();
    TcpListener(const TcpListener&) = delete;
    TcpListener& operator=(const TcpListener&) = delete;

    TcpSocket accept(milliseconds timeout);  // throws TimeoutError
    uint16_t port() const { return port_; }
    void close();

private:
    int fd_ = -1;
    uint16_t port_ = 0;
};

}  // namespace telematics::net
