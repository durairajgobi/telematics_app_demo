#include "telematics/net/tls.hpp"

#include <arpa/inet.h>
#include <openssl/err.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/x509v3.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>

#include <algorithm>
#include <climits>

#if OPENSSL_VERSION_NUMBER < 0x30000000L
#define SSL_get1_peer_certificate SSL_get_peer_certificate  // OpenSSL 1.1.1
#endif

namespace telematics::net {
namespace {

// Drains OpenSSL's thread-local error queue into a readable message.
std::string opensslErrors() {
    std::string out;
    while (const unsigned long e = ERR_get_error()) {
        char buf[256];
        ERR_error_string_n(e, buf, sizeof(buf));
        if (!out.empty()) out += "; ";
        out += buf;
    }
    return out.empty() ? "unknown OpenSSL error" : out;
}

[[noreturn]] void fail(const std::string& what) { throw NetError(what + ": " + opensslErrors()); }

using BioPtr = std::unique_ptr<BIO, decltype(&BIO_free)>;
BioPtr memBio(const std::string& pem) {
    BioPtr bio(BIO_new_mem_buf(pem.data(), static_cast<int>(pem.size())), BIO_free);
    if (!bio) fail("BIO_new_mem_buf");
    return bio;
}

void loadTrust(SSL_CTX* ctx, const TlsConfig& cfg) {
    if (!cfg.ca_file.empty() && SSL_CTX_load_verify_locations(ctx, cfg.ca_file.c_str(), nullptr) != 1) {
        fail("loading CA file '" + cfg.ca_file + "'");
    }
    if (!cfg.ca_pem.empty()) {
        X509_STORE* store = SSL_CTX_get_cert_store(ctx);
        auto bio = memBio(cfg.ca_pem);
        int added = 0;
        while (X509* cert = PEM_read_bio_X509(bio.get(), nullptr, nullptr, nullptr)) {
            X509_STORE_add_cert(store, cert);
            X509_free(cert);
            ++added;
        }
        ERR_clear_error();  // end-of-data "error" after the last certificate
        if (added == 0) throw NetError("ca_pem contains no certificates");
    }
    if (cfg.ca_file.empty() && cfg.ca_pem.empty() && cfg.use_system_ca) {
        if (SSL_CTX_set_default_verify_paths(ctx) != 1) fail("loading system CA store");
    }
}

void loadIdentity(SSL_CTX* ctx, const TlsConfig& cfg) {
    if (!cfg.cert_file.empty()) {
        if (SSL_CTX_use_certificate_chain_file(ctx, cfg.cert_file.c_str()) != 1) {
            fail("loading certificate '" + cfg.cert_file + "'");
        }
    } else if (!cfg.cert_pem.empty()) {
        auto bio = memBio(cfg.cert_pem);
        X509* leaf = PEM_read_bio_X509(bio.get(), nullptr, nullptr, nullptr);
        if (!leaf) fail("parsing cert_pem");
        const int ok = SSL_CTX_use_certificate(ctx, leaf);
        X509_free(leaf);
        if (ok != 1) fail("SSL_CTX_use_certificate");
        while (X509* extra = PEM_read_bio_X509(bio.get(), nullptr, nullptr, nullptr)) {
            SSL_CTX_add0_chain_cert(ctx, extra);  // takes ownership
        }
        ERR_clear_error();
    }

    if (!cfg.key_file.empty()) {
        if (SSL_CTX_use_PrivateKey_file(ctx, cfg.key_file.c_str(), SSL_FILETYPE_PEM) != 1) {
            fail("loading private key '" + cfg.key_file + "'");
        }
    } else if (!cfg.key_pem.empty()) {
        auto bio = memBio(cfg.key_pem);
        EVP_PKEY* key = PEM_read_bio_PrivateKey(bio.get(), nullptr, nullptr, nullptr);
        if (!key) fail("parsing key_pem");
        const int ok = SSL_CTX_use_PrivateKey(ctx, key);
        EVP_PKEY_free(key);
        if (ok != 1) fail("SSL_CTX_use_PrivateKey");
    }

    const bool has_cert = !cfg.cert_file.empty() || !cfg.cert_pem.empty();
    const bool has_key = !cfg.key_file.empty() || !cfg.key_pem.empty();
    if (has_cert != has_key) throw NetError("TLS certificate and private key must be supplied together");
    if (has_cert && SSL_CTX_check_private_key(ctx) != 1) fail("private key does not match certificate");
}

SSL_CTX* newContext(const SSL_METHOD* method) {
    SSL_CTX* ctx = SSL_CTX_new(method);
    if (!ctx) fail("SSL_CTX_new");
    SSL_CTX_set_min_proto_version(ctx, TLS1_2_VERSION);  // TLS 1.0/1.1 are deprecated (RFC 8996)
    SSL_CTX_set_options(ctx, SSL_OP_NO_COMPRESSION | SSL_OP_NO_RENEGOTIATION);
    SSL_CTX_set_mode(ctx, SSL_MODE_ACCEPT_MOVING_WRITE_BUFFER);
    return ctx;
}

bool isIpAddress(const std::string& host) {
    unsigned char buf[sizeof(in6_addr)];
    return inet_pton(AF_INET, host.c_str(), buf) == 1 || inet_pton(AF_INET6, host.c_str(), buf) == 1;
}

// OpenSSL writes to the socket with write(), which raises SIGPIPE (killing the
// process by default) if the peer has gone. Block SIGPIPE on this thread for
// the duration of an OpenSSL call and discard any instance it generated, so the
// failure surfaces as an ordinary error. macOS uses SO_NOSIGPIPE instead.
class SigpipeGuard {
public:
#ifdef __linux__
    SigpipeGuard() {
        sigemptyset(&pipe_);
        sigaddset(&pipe_, SIGPIPE);
        sigset_t pending;
        sigpending(&pending);
        already_pending_ = sigismember(&pending, SIGPIPE) == 1;
        pthread_sigmask(SIG_BLOCK, &pipe_, &old_);
    }
    ~SigpipeGuard() {
        if (!already_pending_) {
            const timespec zero{0, 0};
            while (sigtimedwait(&pipe_, nullptr, &zero) > 0) {
            }
        }
        pthread_sigmask(SIG_SETMASK, &old_, nullptr);
    }

private:
    sigset_t pipe_, old_;
    bool already_pending_ = false;
#endif
};

}  // namespace

// ----------------------------------------------------------------- TlsContext

std::shared_ptr<TlsContext> TlsContext::client(const TlsConfig& cfg) {
    SSL_CTX* ctx = newContext(TLS_client_method());
    try {
        if (cfg.verify_peer) {
            loadTrust(ctx, cfg);
            SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, nullptr);
        } else {
            SSL_CTX_set_verify(ctx, SSL_VERIFY_NONE, nullptr);
        }
        loadIdentity(ctx, cfg);
        if (!cfg.alpn.empty()) {
            std::vector<unsigned char> wire;  // length-prefixed protocol list
            for (const auto& p : cfg.alpn) {
                if (p.empty() || p.size() > 255) throw NetError("invalid ALPN protocol '" + p + "'");
                wire.push_back(static_cast<unsigned char>(p.size()));
                wire.insert(wire.end(), p.begin(), p.end());
            }
            if (SSL_CTX_set_alpn_protos(ctx, wire.data(), static_cast<unsigned>(wire.size())) != 0) {
                fail("SSL_CTX_set_alpn_protos");
            }
        }
    } catch (...) {
        SSL_CTX_free(ctx);
        throw;
    }
    return std::shared_ptr<TlsContext>(new TlsContext(ctx, cfg.verify_peer));
}

std::shared_ptr<TlsContext> TlsContext::server(const TlsConfig& cfg, bool require_client_cert) {
    SSL_CTX* ctx = newContext(TLS_server_method());
    try {
        loadIdentity(ctx, cfg);
        if (require_client_cert) {
            loadTrust(ctx, cfg);
            SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT, nullptr);
        }
    } catch (...) {
        SSL_CTX_free(ctx);
        throw;
    }
    return std::shared_ptr<TlsContext>(new TlsContext(ctx, require_client_cert));
}

TlsContext::~TlsContext() { SSL_CTX_free(ctx_); }

// ------------------------------------------------------------------ TlsStream

TlsStream::TlsStream(std::shared_ptr<TlsContext> ctx, TcpSocket socket)
    : ctx_(std::move(ctx)), socket_(std::move(socket)) {
    ssl_ = SSL_new(ctx_->native());
    if (!ssl_) fail("SSL_new");
    if (SSL_set_fd(ssl_, socket_.fd()) != 1) {
        SSL_free(ssl_);
        ssl_ = nullptr;
        fail("SSL_set_fd");
    }
}

TlsStream::~TlsStream() { close(); }

std::unique_ptr<TlsStream> TlsStream::connect(std::shared_ptr<TlsContext> ctx, TcpSocket socket,
                                              const std::string& server_name, milliseconds timeout) {
    const auto deadline = Clock::now() + timeout;
    std::unique_ptr<TlsStream> s(new TlsStream(std::move(ctx), std::move(socket)));
    SSL* ssl = s->ssl_;

    const bool ip = isIpAddress(server_name);
    if (!ip && SSL_set_tlsext_host_name(ssl, server_name.c_str()) != 1) fail("setting SNI");

    if (s->ctx_->verifyPeer()) {
        X509_VERIFY_PARAM* param = SSL_get0_param(ssl);
        X509_VERIFY_PARAM_set_hostflags(param, X509_CHECK_FLAG_NO_PARTIAL_WILDCARDS);
        const int ok = ip ? X509_VERIFY_PARAM_set1_ip_asc(param, server_name.c_str())
                          : X509_VERIFY_PARAM_set1_host(param, server_name.c_str(), 0);
        if (ok != 1) fail("setting expected peer name");
    }
    s->handshake(true, deadline);
    return s;
}

std::unique_ptr<TlsStream> TlsStream::accept(std::shared_ptr<TlsContext> ctx, TcpSocket socket,
                                             milliseconds timeout) {
    const auto deadline = Clock::now() + timeout;
    std::unique_ptr<TlsStream> s(new TlsStream(std::move(ctx), std::move(socket)));
    s->handshake(false, deadline);
    return s;
}

void TlsStream::waitIo(int ssl_error, Clock::time_point deadline, const char* op) {
    const short events = ssl_error == SSL_ERROR_WANT_WRITE ? POLLOUT : POLLIN;
    if (!waitReady(socket_.fd(), events, deadline)) {
        throw TimeoutError(std::string("TLS ") + op + " timed out with " + socket_.peer());
    }
}

void TlsStream::handshake(bool client, Clock::time_point deadline) {
    SigpipeGuard guard;
    for (;;) {
        ERR_clear_error();
        const int rc = client ? SSL_connect(ssl_) : SSL_accept(ssl_);
        if (rc == 1) return;
        const int err = SSL_get_error(ssl_, rc);
        if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE) {
            waitIo(err, deadline, "handshake");
            continue;
        }
        std::string msg = "TLS handshake with " + socket_.peer() + " failed";
        const long verify = SSL_get_verify_result(ssl_);
        if (verify != X509_V_OK) {
            msg += ": certificate verification failed (";
            msg += X509_verify_cert_error_string(verify);
            msg += ")";
            ERR_clear_error();
        } else {
            msg += ": " + opensslErrors();
        }
        close();
        throw NetError(msg);
    }
}

void TlsStream::writeAll(const void* data, size_t len, milliseconds timeout) {
    if (!ssl_) throw NetError("write on closed TLS stream");
    SigpipeGuard guard;
    const auto deadline = Clock::now() + timeout;
    const auto* p = static_cast<const uint8_t*>(data);
    while (len > 0) {
        ERR_clear_error();
        const int chunk = static_cast<int>(std::min<size_t>(len, INT_MAX));
        const int n = SSL_write(ssl_, p, chunk);
        if (n > 0) {
            p += n;
            len -= static_cast<size_t>(n);
            continue;
        }
        const int err = SSL_get_error(ssl_, n);
        if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE) {
            waitIo(err, deadline, "write");
            continue;
        }
        throw NetError("TLS write to " + socket_.peer() + " failed: " + opensslErrors());
    }
}

size_t TlsStream::readSome(void* buf, size_t len, milliseconds timeout) {
    if (!ssl_) throw NetError("read on closed TLS stream");
    SigpipeGuard guard;  // reads can send alerts / key updates
    const auto deadline = Clock::now() + timeout;
    for (;;) {
        ERR_clear_error();
        const int n = SSL_read(ssl_, buf, static_cast<int>(std::min<size_t>(len, INT_MAX)));
        if (n > 0) return static_cast<size_t>(n);
        const int err = SSL_get_error(ssl_, n);
        if (err == SSL_ERROR_ZERO_RETURN) return 0;  // peer sent close_notify
        if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE) {
            waitIo(err, deadline, "read");
            continue;
        }
        // A TCP close without close_notify could be a truncation attack, so it is an error.
        throw NetError("TLS read from " + socket_.peer() + " failed: " + opensslErrors());
    }
}

void TlsStream::close() {
    if (ssl_) {
        SigpipeGuard guard;
        ERR_clear_error();
        SSL_shutdown(ssl_);  // best effort: send close_notify, don't wait for the reply
        SSL_free(ssl_);
        ssl_ = nullptr;
        ERR_clear_error();
    }
    socket_.close();
}

std::string TlsStream::protocol() const { return ssl_ ? SSL_get_version(ssl_) : ""; }

std::string TlsStream::cipher() const { return ssl_ ? SSL_get_cipher_name(ssl_) : ""; }

std::string TlsStream::peerSubject() const {
    if (!ssl_) return {};
    X509* cert = SSL_get1_peer_certificate(ssl_);
    if (!cert) return {};
    char buf[512];
    X509_NAME_oneline(X509_get_subject_name(cert), buf, sizeof(buf));
    X509_free(cert);
    return buf;
}

}  // namespace telematics::net
