#include "telematics/net/tcp_socket.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>

namespace telematics::net {
namespace {

std::string errnoText(int err) { return std::strerror(err); }

void setNonBlocking(int fd) {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        throw NetError("fcntl(O_NONBLOCK) failed: " + errnoText(errno));
    }
}

#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;  // never raise SIGPIPE on a dead peer
#else
constexpr int kSendFlags = 0;  // macOS: SO_NOSIGPIPE is set in configure()
#endif

}  // namespace

bool waitReady(int fd, short events, Clock::time_point deadline) {
    for (;;) {
        pollfd p{fd, events, 0};
        const int rc = ::poll(&p, 1, static_cast<int>(remaining(deadline).count()));
        if (rc > 0) return true;  // ready, or error/hang-up the next call will report
        if (rc == 0) return false;
        if (errno != EINTR) throw NetError("poll failed: " + errnoText(errno));
    }
}

void readExact(IStream& s, void* buf, size_t len, milliseconds timeout) {
    const auto deadline = Clock::now() + timeout;
    auto* p = static_cast<uint8_t*>(buf);
    while (len > 0) {
        const size_t n = s.readSome(p, len, remaining(deadline));
        if (n == 0) throw NetError("connection closed by peer");
        p += n;
        len -= n;
    }
}

// ------------------------------------------------------------------ TcpSocket

TcpSocket::TcpSocket(int fd, std::string peer) : fd_(fd), peer_(std::move(peer)) {}

TcpSocket::~TcpSocket() { close(); }

TcpSocket::TcpSocket(TcpSocket&& o) noexcept : fd_(o.fd_), peer_(std::move(o.peer_)) { o.fd_ = -1; }

TcpSocket& TcpSocket::operator=(TcpSocket&& o) noexcept {
    if (this != &o) {
        close();
        fd_ = o.fd_;
        peer_ = std::move(o.peer_);
        o.fd_ = -1;
    }
    return *this;
}

void TcpSocket::configure() {
    setNonBlocking(fd_);
    int one = 1;
    ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));  // small telemetry packets: no Nagle delay
    ::setsockopt(fd_, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
#ifdef SO_NOSIGPIPE
    ::setsockopt(fd_, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
#ifdef TCP_KEEPIDLE
    // Detect a silently dead cellular link in ~60 s instead of the 2 h default.
    int idle = 30, interval = 10, count = 3;
    ::setsockopt(fd_, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    ::setsockopt(fd_, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
    ::setsockopt(fd_, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
#endif
}

TcpSocket TcpSocket::connect(const std::string& host, uint16_t port, milliseconds timeout) {
    const auto deadline = Clock::now() + timeout;
    const std::string where = host + ":" + std::to_string(port);

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;  // IPv4 and IPv6
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    // Note: getaddrinfo itself is blocking; production TCUs often use c-ares or a resolver thread.
    const int rc = ::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res);
    if (rc != 0) throw NetError("DNS lookup for '" + host + "' failed: " + ::gai_strerror(rc));
    std::unique_ptr<addrinfo, decltype(&::freeaddrinfo)> guard(res, ::freeaddrinfo);

    std::string last_error = "no addresses";
    bool timed_out = false;
    for (addrinfo* ai = res; ai; ai = ai->ai_next) {
        int fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) {
            last_error = errnoText(errno);
            continue;
        }
        TcpSocket sock(fd, where);
        sock.configure();

        if (::connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) return sock;
        if (errno != EINPROGRESS) {
            last_error = errnoText(errno);
            continue;
        }
        if (!waitReady(fd, POLLOUT, deadline)) {
            timed_out = true;
            last_error = "timed out";
            break;
        }
        int err = 0;
        socklen_t len = sizeof(err);
        ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err == 0) return sock;
        last_error = errnoText(err);
    }
    const std::string msg = "TCP connect to " + where + " failed: " + last_error;
    if (timed_out) throw TimeoutError(msg);
    throw NetError(msg);
}

void TcpSocket::writeAll(const void* data, size_t len, milliseconds timeout) {
    if (fd_ < 0) throw NetError("write on closed socket");
    const auto deadline = Clock::now() + timeout;
    const auto* p = static_cast<const uint8_t*>(data);
    while (len > 0) {
        const ssize_t n = ::send(fd_, p, len, kSendFlags);
        if (n > 0) {
            p += n;
            len -= static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (!waitReady(fd_, POLLOUT, deadline)) throw TimeoutError("TCP write timed out to " + peer_);
            continue;
        }
        throw NetError("TCP write to " + peer_ + " failed: " + errnoText(errno));
    }
}

size_t TcpSocket::readSome(void* buf, size_t len, milliseconds timeout) {
    if (fd_ < 0) throw NetError("read on closed socket");
    const auto deadline = Clock::now() + timeout;
    for (;;) {
        const ssize_t n = ::recv(fd_, buf, len, 0);
        if (n >= 0) return static_cast<size_t>(n);  // 0 = orderly shutdown
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            if (!waitReady(fd_, POLLIN, deadline)) throw TimeoutError("TCP read timed out from " + peer_);
            continue;
        }
        throw NetError("TCP read from " + peer_ + " failed: " + errnoText(errno));
    }
}

void TcpSocket::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

// ---------------------------------------------------------------- TcpListener

TcpListener::TcpListener(const std::string& bind_addr, uint16_t port, int backlog) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, bind_addr.c_str(), &addr.sin_addr) != 1) {
        throw NetError("invalid IPv4 bind address '" + bind_addr + "'");
    }
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) throw NetError("socket failed: " + errnoText(errno));
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 || ::listen(fd_, backlog) < 0) {
        const std::string err = errnoText(errno);
        close();
        throw NetError("bind/listen on " + bind_addr + " failed: " + err);
    }
    setNonBlocking(fd_);
    socklen_t len = sizeof(addr);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
    port_ = ntohs(addr.sin_port);
}

TcpListener::~TcpListener() { close(); }

TcpSocket TcpListener::accept(milliseconds timeout) {
    const auto deadline = Clock::now() + timeout;
    for (;;) {
        sockaddr_in peer{};
        socklen_t len = sizeof(peer);
        const int fd = ::accept(fd_, reinterpret_cast<sockaddr*>(&peer), &len);
        if (fd >= 0) {
            char ip[INET_ADDRSTRLEN] = {};
            ::inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
            TcpSocket s(fd, std::string(ip) + ":" + std::to_string(ntohs(peer.sin_port)));
            setNonBlocking(fd);
            return s;
        }
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            if (!waitReady(fd_, POLLIN, deadline)) throw TimeoutError("accept timed out");
            continue;
        }
        throw NetError("accept failed: " + errnoText(errno));
    }
}

void TcpListener::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

}  // namespace telematics::net
