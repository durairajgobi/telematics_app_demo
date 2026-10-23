#pragma once
// Minimal HTTP/1.1 client over TCP or TLS (HTTPS).
// One request per connection (Connection: close); supports Content-Length,
// chunked and read-until-close response bodies, with size limits.
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "telematics/net/tls.hpp"

namespace telematics::net {

using Headers = std::vector<std::pair<std::string, std::string>>;

struct Url {
    std::string scheme;  // "http" or "https" (also "mqtt"/"mqtts" for config parsing)
    std::string host;    // without brackets for IPv6
    uint16_t port = 0;
    std::string target;  // path + query, always starts with '/'

    static Url parse(const std::string& url);  // throws NetError
    bool secure() const { return scheme == "https" || scheme == "mqtts"; }
    std::string hostHeader() const;
};

struct HttpResponse {
    int status = 0;
    std::string reason;
    Headers headers;  // names lower-cased
    std::string body;

    std::optional<std::string> header(const std::string& name) const;
    bool ok() const { return status >= 200 && status < 300; }
};

class HttpClient {
public:
    // `tls` is used for https:// URLs; if null, a context trusting the
    // system CA store is created on first use.
    explicit HttpClient(std::shared_ptr<TlsContext> tls = nullptr, milliseconds timeout = milliseconds(15'000));

    HttpResponse request(const std::string& method, const std::string& url, const Headers& headers = {},
                         const std::string& body = {});
    HttpResponse get(const std::string& url, const Headers& headers = {}) { return request("GET", url, headers); }
    HttpResponse post(const std::string& url, const std::string& body,
                      const std::string& content_type = "application/json", Headers headers = {});

    size_t max_body_bytes = 16 * 1024 * 1024;

private:
    std::shared_ptr<TlsContext> tls_;
    milliseconds timeout_;
};

enum class ChunkedStatus { NeedMore, Complete };
// Decodes a complete or partial chunked body. On Complete, `out` holds the
// payload and `consumed` the number of input bytes used. Throws ProtocolError.
ChunkedStatus decodeChunked(std::string_view in, std::string& out, size_t& consumed);

}  // namespace telematics::net
