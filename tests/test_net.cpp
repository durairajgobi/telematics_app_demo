// Network stack tests: MQTT codec, URL/chunked parsing, and end-to-end runs
// of the MQTT and HTTPS clients against in-process servers over real TLS.
// Certificates are generated on the fly, so no files or internet are needed.
#include <openssl/pem.h>
#include <openssl/x509v3.h>

#include <atomic>
#include <cstring>
#include <iostream>
#include <thread>

#include "telematics/net/transports.hpp"

using namespace telematics;
using namespace telematics::net;
using namespace std::chrono_literals;

static int g_failures = 0, g_checks = 0;
#define CHECK(cond)                                                                   \
    do {                                                                              \
        ++g_checks;                                                                   \
        if (!(cond)) {                                                                \
            ++g_failures;                                                             \
            std::cerr << "  FAIL " << __FILE__ << ":" << __LINE__ << "  " #cond "\n"; \
        }                                                                             \
    } while (0)

template <typename E, typename F>
static bool throws(F&& f, std::string* msg = nullptr) {
    try {
        f();
    } catch (const E& e) {
        if (msg) *msg = e.what();
        return true;
    } catch (...) {
    }
    return false;
}

// ------------------------------------------------------------ test certificates

struct Pki {
    std::string cert_pem, key_pem;
};

static Pki makeSelfSigned(const char* cn, const char* san) {
    EVP_PKEY* key = EVP_EC_gen("P-256");
    X509* x = X509_new();
    X509_set_version(x, 2);
    static long serial = 1;
    ASN1_INTEGER_set(X509_get_serialNumber(x), serial++);
    X509_gmtime_adj(X509_getm_notBefore(x), -60);
    X509_gmtime_adj(X509_getm_notAfter(x), 3600);
    X509_set_pubkey(x, key);
    X509_NAME* name = X509_get_subject_name(x);
    X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC, reinterpret_cast<const unsigned char*>(cn), -1, -1, 0);
    X509_set_issuer_name(x, name);
    X509V3_CTX ctx;
    X509V3_set_ctx_nodb(&ctx);
    X509V3_set_ctx(&ctx, x, x, nullptr, nullptr, 0);
    X509_EXTENSION* ext = X509V3_EXT_conf_nid(nullptr, &ctx, NID_subject_alt_name, san);
    X509_add_ext(x, ext, -1);
    X509_EXTENSION_free(ext);
    X509_sign(x, key, EVP_sha256());

    auto toPem = [](auto writer) {
        BIO* bio = BIO_new(BIO_s_mem());
        writer(bio);
        char* data = nullptr;
        const long len = BIO_get_mem_data(bio, &data);
        std::string s(data, static_cast<size_t>(len));
        BIO_free(bio);
        return s;
    };
    Pki p;
    p.cert_pem = toPem([&](BIO* b) { PEM_write_bio_X509(b, x); });
    p.key_pem = toPem([&](BIO* b) { PEM_write_bio_PrivateKey(b, key, nullptr, nullptr, 0, nullptr, nullptr); });
    X509_free(x);
    EVP_PKEY_free(key);
    return p;
}

static const Pki& serverPki() {
    static Pki p = makeSelfSigned("localhost", "DNS:localhost,IP:127.0.0.1");
    return p;
}
static const Pki& clientPki() {
    static Pki p = makeSelfSigned("tcu-MA3EWDE1S00000000", "DNS:tcu.local");
    return p;
}

static std::shared_ptr<TlsContext> trustingClient(bool with_client_cert = false) {
    TlsConfig c;
    c.ca_pem = serverPki().cert_pem;
    if (with_client_cert) {
        c.cert_pem = clientPki().cert_pem;
        c.key_pem = clientPki().key_pem;
    }
    return TlsContext::client(c);
}

static std::shared_ptr<TlsContext> serverCtx(bool require_client_cert = false) {
    TlsConfig s;
    s.cert_pem = serverPki().cert_pem;
    s.key_pem = serverPki().key_pem;
    if (require_client_cert) s.ca_pem = clientPki().cert_pem;
    return TlsContext::server(s, require_client_cert);
}

// ------------------------------------------------------------ unit tests

static void testRemainingLength() {
    const std::pair<uint32_t, mqtt::Bytes> cases[] = {
        {0, {0x00}}, {127, {0x7F}}, {128, {0x80, 0x01}}, {16383, {0xFF, 0x7F}},
        {16384, {0x80, 0x80, 0x01}}, {2097151, {0xFF, 0xFF, 0x7F}},
        {2097152, {0x80, 0x80, 0x80, 0x01}}, {268435455, {0xFF, 0xFF, 0xFF, 0x7F}},
    };
    for (const auto& [value, bytes] : cases) {
        mqtt::Bytes out;
        mqtt::appendRemainingLength(out, value);
        CHECK(out == bytes);
        auto d = mqtt::decodeRemainingLength(bytes.data(), bytes.size());
        CHECK(d && d->first == value && d->second == bytes.size());
    }
    const uint8_t partial[] = {0x80, 0x80};
    CHECK(!mqtt::decodeRemainingLength(partial, 2));
    const uint8_t bad[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x01};
    CHECK(throws<ProtocolError>([&] { mqtt::decodeRemainingLength(bad, 5); }));
}

static void testMqttCodec() {
    mqtt::ConnectOptions o;
    o.client_id = "tcu";
    o.keep_alive_s = 60;
    const mqtt::Bytes expect_connect = {0x10, 0x0F, 0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04, 0x02,
                                        0x00, 0x3C, 0x00, 0x03, 't', 'c', 'u'};
    CHECK(mqtt::encodeConnect(o) == expect_connect);

    o.username = "u";
    o.password = "p";
    o.will = mqtt::Will{"v/status", "offline", 1, true};
    const auto c2 = mqtt::encodeConnect(o);
    CHECK(c2[9] == (0x80 | 0x40 | 0x20 | 0x08 | 0x04 | 0x02));  // user, pass, will retain, will qos1, will, clean

    mqtt::ConnectOptions bad;
    bad.client_id = "x";
    bad.password = "nouser";
    CHECK(throws<ProtocolError>([&] { mqtt::encodeConnect(bad); }));

    mqtt::PublishMsg m{"a/b", "hi", 1, false, false, 10};
    const mqtt::Bytes expect_pub = {0x32, 0x09, 0x00, 0x03, 'a', '/', 'b', 0x00, 0x0A, 'h', 'i'};
    CHECK(mqtt::encodePublish(m) == expect_pub);
    m.topic = "a/+";
    CHECK(throws<ProtocolError>([&] { mqtt::encodePublish(m); }));

    CHECK(mqtt::isValidTopicFilter("vehicles/+/cmd/#"));
    CHECK(mqtt::isValidTopicFilter("#"));
    CHECK(!mqtt::isValidTopicFilter("a/#/b"));
    CHECK(!mqtt::isValidTopicFilter("a+/b"));

    // Byte-at-a-time framing of two back-to-back packets.
    mqtt::PacketReader r;
    mqtt::Bytes stream = expect_pub;
    const mqtt::Bytes ping = {0xD0, 0x00};
    stream.insert(stream.end(), ping.begin(), ping.end());
    std::vector<mqtt::Packet> got;
    for (uint8_t b : stream) {
        r.feed(&b, 1);
        while (auto p = r.next()) got.push_back(*p);
    }
    CHECK(got.size() == 2);
    if (got.size() == 2) {
        const auto pm = mqtt::parsePublish(got[0]);
        CHECK(pm.topic == "a/b" && pm.payload == "hi" && pm.qos == 1 && pm.packet_id == 10);
        CHECK(got[1].type == mqtt::PacketType::Pingresp);
    }

    mqtt::PacketReader r2;
    const uint8_t bad_flags[] = {0x41, 0x02, 0x00, 0x01};  // PUBACK with reserved flag set
    r2.feed(bad_flags, sizeof(bad_flags));
    CHECK(throws<ProtocolError>([&] { r2.next(); }));
}

static void testUrlAndChunked() {
    auto u = Url::parse("https://api.example.com/v1/telemetry?fleet=7#frag");
    CHECK(u.scheme == "https" && u.host == "api.example.com" && u.port == 443);
    CHECK(u.target == "/v1/telemetry?fleet=7" && u.hostHeader() == "api.example.com");
    u = Url::parse("mqtts://broker.local:8884");
    CHECK(u.port == 8884 && u.target == "/" && u.secure());
    u = Url::parse("http://[::1]:8080/x");
    CHECK(u.host == "::1" && u.port == 8080 && u.hostHeader() == "[::1]:8080");
    CHECK(throws<NetError>([] { Url::parse("ftp://x"); }));
    CHECK(throws<NetError>([] { Url::parse("https://host:99999/"); }));
    CHECK(throws<NetError>([] { Url::parse("https://evil\r\nX: y/"); }));

    std::string out;
    size_t used = 0;
    const std::string body = "4\r\nWiki\r\n6;ext=1\r\npedia \r\nE\r\nin \r\n\r\nchunks.\r\n0\r\nX-Trailer: 1\r\n\r\nNEXT";
    CHECK(decodeChunked(body, out, used) == ChunkedStatus::Complete);
    CHECK(out == "Wikipedia in \r\n\r\nchunks." && body.substr(used) == "NEXT");
    CHECK(decodeChunked("4\r\nWi", out, used) == ChunkedStatus::NeedMore);
    CHECK(throws<ProtocolError>([&] { decodeChunked("zz\r\n", out, used); }));
}

// ------------------------------------------------------------ fake MQTT broker

struct FakeBroker {
    TcpListener listener{"127.0.0.1", 0};
    std::shared_ptr<TlsContext> tls;
    std::thread th;
    std::atomic<bool> stop{false};
    std::vector<mqtt::PublishMsg> published;  // read after join()
    std::string client_id, error;
    bool will_seen = false, disconnect_seen = false;
    int pings = 0, connections = 0;
    uint8_t refuse_code = 0;

    explicit FakeBroker(std::shared_ptr<TlsContext> ctx, int max_connections = 1) : tls(std::move(ctx)) {
        // Started here, not in the initializer list, so every member exists first.
        th = std::thread([this, max_connections] {
            for (int i = 0; i < max_connections && !stop; ++i) {
                try {
                    serve();
                } catch (const std::exception& e) {
                    error = e.what();
                }
            }
        });
    }
    ~FakeBroker() { join(); }
    void join() {
        if (th.joinable()) th.join();
    }

    void serve() {
        auto sock = listener.accept(5s);
        std::unique_ptr<IStream> s;
        if (tls) s = TlsStream::accept(tls, std::move(sock), 5s);
        else s = std::make_unique<TcpSocket>(std::move(sock));
        ++connections;

        mqtt::PacketReader reader;
        auto send = [&](const mqtt::Bytes& b) { s->writeAll(b.data(), b.size(), 2s); };
        uint8_t buf[4096];
        for (;;) {
            size_t n = 0;
            try {
                n = s->readSome(buf, sizeof(buf), 3s);
            } catch (const NetError&) {
                return;  // client dropped
            }
            if (n == 0) return;
            reader.feed(buf, n);
            while (auto p = reader.next()) {
                switch (p->type) {
                    case mqtt::PacketType::Connect: {
                        // body: "MQTT" (6) + level + flags + keepalive (4) + client id
                        will_seen = (p->body[7] & 0x04) != 0;
                        const size_t id_len = p->body[10] << 8 | p->body[11];
                        client_id.assign(p->body.begin() + 12, p->body.begin() + 12 + static_cast<long>(id_len));
                        send({0x20, 0x02, 0x00, refuse_code});
                        if (refuse_code) return;
                        break;
                    }
                    case mqtt::PacketType::Publish: {
                        auto m = mqtt::parsePublish(*p);
                        published.push_back(m);
                        if (m.qos == 1) send(mqtt::encodePuback(m.packet_id));
                        break;
                    }
                    case mqtt::PacketType::Subscribe: {
                        const uint16_t id = mqtt::parsePacketId(*p);
                        send({0x90, 0x03, static_cast<uint8_t>(id >> 8), static_cast<uint8_t>(id & 0xFF), 0x01});
                        // Immediately deliver a QoS1 "command" to the subscriber.
                        send(mqtt::encodePublish({"vehicles/VIN1/cmd", R"({"cmd":"locate"})", 1, false, false, 77}));
                        break;
                    }
                    case mqtt::PacketType::Pingreq:
                        ++pings;
                        send({0xD0, 0x00});
                        break;
                    case mqtt::PacketType::Disconnect:
                        disconnect_seen = true;
                        return;
                    default:
                        break;
                }
            }
        }
    }
};

static MqttClientConfig mqttCfg(uint16_t port, std::shared_ptr<TlsContext> tls) {
    MqttClientConfig c;
    c.host = "localhost";
    c.port = port;
    c.tls = std::move(tls);
    c.connect.client_id = "VIN1";
    c.connect.keep_alive_s = 1;  // short so the test exercises PINGREQ
    c.ack_timeout = 2s;
    c.connect_timeout = 3s;
    return c;
}

static void testTcpLoopback() {
    TcpListener l;
    std::thread server([&] {
        auto s = l.accept(2s);
        char buf[16];
        const size_t n = s.readSome(buf, sizeof(buf), 2s);
        s.writeAll(buf, n, 2s);  // echo
    });
    auto c = TcpSocket::connect("127.0.0.1", l.port(), 2s);
    writeString(c, "ping", 1s);
    char buf[4];
    readExact(c, buf, 4, 2s);
    CHECK(std::memcmp(buf, "ping", 4) == 0);
    server.join();

    CHECK(throws<TimeoutError>([&] { char b; c.readSome(&b, 1, 50ms); }) ||
          c.readSome(buf, 1, 50ms) == 0);  // peer closed after echo
    CHECK(throws<NetError>([] { TcpSocket::connect("no-such-host.invalid", 80, 2s); }));
}

static void testMqttOverTls() {
    FakeBroker broker(serverCtx());
    MqttClient client(mqttCfg(broker.listener.port(), trustingClient()));

    std::vector<mqtt::PublishMsg> inbox;
    client.onMessage([&](const mqtt::PublishMsg& m) { inbox.push_back(m); });

    client.connect();
    CHECK(client.connected());
    client.publish("vehicles/VIN1/telemetry", R"({"speed":42})", 1);
    client.publish("vehicles/VIN1/telemetry", "qos0", 0);
    CHECK(client.subscribe("vehicles/VIN1/cmd", 1) == 1);
    client.loop(1500ms);  // keep-alive of 1 s must trigger a PINGREQ
    client.disconnect();
    broker.join();

    CHECK(broker.error.empty());
    CHECK(broker.client_id == "VIN1");
    CHECK(broker.published.size() == 2);
    if (broker.published.size() == 2) {
        CHECK(broker.published[0].payload == R"({"speed":42})" && broker.published[0].qos == 1);
        CHECK(broker.published[1].qos == 0);
    }
    CHECK(inbox.size() == 1 && inbox[0].topic == "vehicles/VIN1/cmd");
    CHECK(broker.pings >= 1);
    CHECK(broker.disconnect_seen);
}

static void testMqttRefused() {
    FakeBroker broker(serverCtx());
    broker.refuse_code = 5;
    MqttClient client(mqttCfg(broker.listener.port(), trustingClient()));
    std::string msg;
    CHECK(throws<NetError>([&] { client.connect(); }, &msg));
    CHECK(msg.find("not authorized") != std::string::npos);
    CHECK(!client.connected());
}

static void testTlsVerification() {
    // 1) Client does not trust the server's certificate.
    {
        FakeBroker broker(serverCtx());
        TlsConfig untrusting;
        untrusting.ca_pem = clientPki().cert_pem;  // wrong CA
        MqttClient c(mqttCfg(broker.listener.port(), TlsContext::client(untrusting)));
        std::string msg;
        CHECK(throws<NetError>([&] { c.connect(); }, &msg));
        CHECK(msg.find("certificate verification failed") != std::string::npos);
    }
    // 2) Trusted CA, but the name does not match the certificate.
    {
        TcpListener l;
        std::thread srv([&] {
            try {
                TlsStream::accept(serverCtx(), l.accept(2s), 2s);
            } catch (...) {
            }
        });
        auto tcp = TcpSocket::connect("127.0.0.1", l.port(), 2s);
        std::string msg;
        CHECK(throws<NetError>([&] { TlsStream::connect(trustingClient(), std::move(tcp), "evil.example", 2s); }, &msg));
        CHECK(msg.find("hostname mismatch") != std::string::npos);
        srv.join();
    }
    // 3) Connecting by IP address checks the IP SAN.
    {
        TcpListener l;
        std::string proto;
        std::thread srv([&] {
            try {
                auto s = TlsStream::accept(serverCtx(), l.accept(2s), 2s);
                proto = s->protocol();
            } catch (...) {
            }
        });
        auto tcp = TcpSocket::connect("127.0.0.1", l.port(), 2s);
        auto s = TlsStream::connect(trustingClient(), std::move(tcp), "127.0.0.1", 2s);
        CHECK(s->protocol() == "TLSv1.3");
        CHECK(s->peerSubject().find("CN=localhost") != std::string::npos);
        srv.join();
    }
}

static void testMutualTls() {
    // Server requires a client certificate (as AWS IoT Core does).
    {
        FakeBroker broker(serverCtx(/*require_client_cert=*/true));
        MqttClient c(mqttCfg(broker.listener.port(), trustingClient(/*with_client_cert=*/true)));
        c.connect();
        c.publish("vehicles/VIN1/telemetry", "{}", 1);
        c.disconnect();
        broker.join();
        CHECK(broker.error.empty() && broker.published.size() == 1);
    }
    {
        FakeBroker broker(serverCtx(true));
        MqttClient c(mqttCfg(broker.listener.port(), trustingClient(false)));
        // TLS 1.3 reports the missing certificate after the handshake, on first use.
        bool failed = throws<NetError>([&] {
            c.connect();
            c.publish("vehicles/VIN1/telemetry", "{}", 1);
        });
        broker.join();
        CHECK(failed);
        CHECK(!broker.error.empty());  // server rejected the handshake
        CHECK(broker.published.empty());
    }
}

static void testMqttTransportReconnects() {
    FakeBroker broker(serverCtx(), /*max_connections=*/2);
    net::MqttTransport transport(mqttCfg(broker.listener.port(), trustingClient()), "vehicles/VIN1");

    std::vector<Message> batch = {{R"({"type":"telemetry","speed":1})", Priority::Normal, 1},
                                  {R"({"type":"event","event":"harsh_braking"})", Priority::High, 2}};
    CHECK(transport.send(batch));
    transport.client().disconnect();  // simulate a dropped link; next send reconnects
    CHECK(transport.send({{R"({"type":"trip"})", Priority::High, 3}}));
    transport.client().disconnect();
    broker.join();

    CHECK(broker.connections == 2);
    CHECK(broker.will_seen);
    std::vector<std::string> topics;
    for (auto& m : broker.published) topics.push_back(m.topic + "=" + (m.retain ? "R:" : "") + m.payload.substr(0, 12));
    CHECK(topics.size() == 5);  // online, telemetry, event, online, trip
    if (topics.size() == 5) {
        CHECK(topics[0] == "vehicles/VIN1/status=R:online");
        CHECK(broker.published[1].topic == "vehicles/VIN1/telemetry");
        CHECK(broker.published[2].topic == "vehicles/VIN1/event");
        CHECK(broker.published[4].topic == "vehicles/VIN1/trip");
    }

    // Unreachable broker => send() reports failure so store-and-forward retries.
    TcpListener closed;
    const uint16_t dead_port = closed.port();
    closed.close();
    net::MqttTransport dead(mqttCfg(dead_port, trustingClient()), "vehicles/VIN1");
    std::string err;
    dead.on_error = [&](const std::string& e) { err = e; };
    CHECK(!dead.send(batch));
    CHECK(err.find("connect") != std::string::npos);
}

// ------------------------------------------------------------ fake HTTPS server

struct FakeHttps {
    TcpListener listener{"127.0.0.1", 0};
    std::thread th;
    std::string request_head, request_body, error;
    int status;
    bool chunked;

    FakeHttps(int status_code, bool use_chunked) : status(status_code), chunked(use_chunked) {
        th = std::thread([this] {
            try {
                auto s = TlsStream::accept(serverCtx(), listener.accept(5s), 5s);
                std::string buf;
                char tmp[4096];
                size_t end;
                while ((end = buf.find("\r\n\r\n")) == std::string::npos) {
                    const size_t n = s->readSome(tmp, sizeof(tmp), 3s);
                    if (n == 0) return;
                    buf.append(tmp, n);
                }
                request_head = buf.substr(0, end);
                buf.erase(0, end + 4);
                const auto cl = request_head.find("Content-Length: ");
                const size_t len = cl == std::string::npos ? 0 : std::stoul(request_head.substr(cl + 16));
                while (buf.size() < len) buf.append(tmp, s->readSome(tmp, sizeof(tmp), 3s));
                request_body = buf.substr(0, len);

                const std::string reason = status == 200 ? "OK" : status == 503 ? "Service Unavailable" : "Bad Request";
                std::string resp = "HTTP/1.1 100 Continue\r\n\r\n";  // interim response must be skipped
                resp += "HTTP/1.1 " + std::to_string(status) + " " + reason + "\r\nContent-Type: application/json\r\n";
                if (chunked) {
                    resp += "Transfer-Encoding: chunked\r\n\r\n9\r\n{\"accepte\r\n6\r\nd\":2}\n\r\n0\r\n\r\n";
                } else {
                    resp += "Content-Length: 11\r\n\r\n{\"err\":\"x\"}";
                }
                writeString(*s, resp, 2s);
                s->close();
            } catch (const std::exception& e) {
                error = e.what();
            }
        });
    }
    ~FakeHttps() { join(); }
    void join() {
        if (th.joinable()) th.join();
    }
    std::string url() const { return "https://localhost:" + std::to_string(listener.port()) + "/v1/ingest?fleet=7"; }
};

static void testHttpsClient() {
    FakeHttps srv(200, true);
    HttpClient http(trustingClient(), 5s);
    const auto r = http.post(srv.url(), R"([{"a":1}])", "application/json", {{"Authorization", "Bearer abc"}});
    srv.join();
    CHECK(srv.error.empty());
    CHECK(r.status == 200 && r.ok());
    CHECK(r.body == "{\"accepted\":2}\n");
    CHECK(r.header("Content-Type") == std::optional<std::string>("application/json"));
    CHECK(srv.request_head.rfind("POST /v1/ingest?fleet=7 HTTP/1.1\r\n", 0) == 0);
    CHECK(srv.request_head.find("Host: localhost:" + std::to_string(srv.listener.port())) != std::string::npos);
    CHECK(srv.request_head.find("Authorization: Bearer abc") != std::string::npos);
    CHECK(srv.request_body == R"([{"a":1}])");

    CHECK(throws<NetError>([&] { http.get("https://localhost:1/", {{"X-Evil", "a\r\nInjected: 1"}}); }));
}

static void testHttpsTransport() {
    std::vector<Message> batch = {{R"({"type":"telemetry","n":1})", Priority::Normal, 1},
                                  {R"({"type":"event","n":2})", Priority::High, 2}};
    {
        FakeHttps srv(200, true);
        HttpsTransport t(srv.url(), trustingClient(), "secret-token", 5s);
        CHECK(t.send(batch));
        srv.join();
        CHECK(srv.request_body == R"([{"type":"telemetry","n":1},{"type":"event","n":2}])");
        CHECK(srv.request_head.find("Authorization: Bearer secret-token") != std::string::npos);
        CHECK(srv.request_head.find("X-Batch-Size: 2") != std::string::npos);
    }
    {
        FakeHttps srv(503, false);  // transient: keep the data and retry
        HttpsTransport t(srv.url(), trustingClient(), "", 5s);
        CHECK(!t.send(batch));
        CHECK(t.rejected() == 0);
    }
    {
        FakeHttps srv(400, false);  // permanent: drop so the queue is not blocked
        HttpsTransport t(srv.url(), trustingClient(), "", 5s);
        CHECK(t.send(batch));
        CHECK(t.rejected() == 2);
    }
    {
        // End to end through StoreAndForward.
        FakeHttps srv(200, true);
        HttpsTransport t(srv.url(), trustingClient(), "", 5s);
        StoreAndForward uplink(t, 100, 10);
        uplink.enqueue(R"({"type":"telemetry"})");
        uplink.enqueue(R"({"type":"event"})", Priority::High);
        CHECK(uplink.flushOnce());
        srv.join();
        CHECK(uplink.pending() == 0);
        CHECK(srv.request_body == R"([{"type":"event"},{"type":"telemetry"}])");  // alert first
    }
}

int main() {
    struct {
        const char* name;
        void (*fn)();
    } tests[] = {
        {"mqtt_remaining_length", testRemainingLength},
        {"mqtt_codec", testMqttCodec},
        {"url_and_chunked", testUrlAndChunked},
        {"tcp_loopback", testTcpLoopback},
        {"mqtt_over_tls", testMqttOverTls},
        {"mqtt_connack_refused", testMqttRefused},
        {"tls_verification", testTlsVerification},
        {"mutual_tls", testMutualTls},
        {"mqtt_transport_reconnect", testMqttTransportReconnects},
        {"https_client", testHttpsClient},
        {"https_transport", testHttpsTransport},
    };
    for (auto& t : tests) {
        const int before = g_failures;
        try {
            t.fn();
        } catch (const std::exception& e) {
            ++g_failures;
            std::cerr << "  EXCEPTION " << e.what() << "\n";
        }
        std::cout << (g_failures == before ? "[ OK ] " : "[FAIL] ") << t.name << "\n";
    }
    std::cout << g_checks - g_failures << "/" << g_checks << " checks passed\n";
    return g_failures == 0 ? 0 : 1;
}
