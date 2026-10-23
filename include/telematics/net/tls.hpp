#pragma once
// TLS 1.2/1.3 over TcpSocket using OpenSSL 1.1.1+ / 3.x.
//
// Client features: CA verification (system store, file or in-memory PEM),
// SNI, hostname / IP verification, mutual TLS (client certificate — required
// by AWS IoT Core, Azure IoT Hub and most OEM backends), ALPN.
#include <memory>
#include <string>
#include <vector>

#include "telematics/net/tcp_socket.hpp"

struct ssl_ctx_st;  // SSL_CTX
struct ssl_st;      // SSL

namespace telematics::net {

struct TlsConfig {
    // Trust anchors. If none are given and use_system_ca is true, the OS store is used.
    std::string ca_file;
    std::string ca_pem;
    bool use_system_ca = true;

    // Own certificate + private key (client cert for mTLS, or server cert).
    std::string cert_file, key_file;
    std::string cert_pem, key_pem;

    bool verify_peer = true;          // never disable in production
    std::vector<std::string> alpn;    // e.g. {"x-amzn-mqtt-ca"} for AWS IoT MQTT on port 443
};

class TlsContext {
public:
    static std::shared_ptr<TlsContext> client(const TlsConfig& cfg);
    // require_client_cert: reject clients without a certificate signed by cfg's CA (mTLS).
    static std::shared_ptr<TlsContext> server(const TlsConfig& cfg, bool require_client_cert = false);

    ~TlsContext();
    TlsContext(const TlsContext&) = delete;
    TlsContext& operator=(const TlsContext&) = delete;

    ssl_ctx_st* native() const { return ctx_; }
    bool verifyPeer() const { return verify_peer_; }

private:
    TlsContext(ssl_ctx_st* ctx, bool verify) : ctx_(ctx), verify_peer_(verify) {}
    ssl_ctx_st* ctx_;
    bool verify_peer_;
};

class TlsStream : public IStream {
public:
    // Performs the client handshake. `server_name` is used for SNI and for
    // certificate hostname (or IP address) verification.
    static std::unique_ptr<TlsStream> connect(std::shared_ptr<TlsContext> ctx, TcpSocket socket,
                                              const std::string& server_name, milliseconds timeout);
    // Performs the server handshake on an accepted socket.
    static std::unique_ptr<TlsStream> accept(std::shared_ptr<TlsContext> ctx, TcpSocket socket,
                                             milliseconds timeout);

    ~TlsStream() override;

    void writeAll(const void* data, size_t len, milliseconds timeout) override;
    size_t readSome(void* buf, size_t len, milliseconds timeout) override;
    void close() override;
    bool isOpen() const override { return ssl_ != nullptr; }

    std::string protocol() const;     // "TLSv1.3"
    std::string cipher() const;
    std::string peerSubject() const;  // peer certificate subject, empty if none

private:
    TlsStream(std::shared_ptr<TlsContext> ctx, TcpSocket socket);
    void handshake(bool client, Clock::time_point deadline);
    void waitIo(int ssl_error, Clock::time_point deadline, const char* op);

    std::shared_ptr<TlsContext> ctx_;
    TcpSocket socket_;
    ssl_st* ssl_ = nullptr;
};

}  // namespace telematics::net
