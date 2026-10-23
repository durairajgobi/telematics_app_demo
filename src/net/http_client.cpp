#include "telematics/net/http_client.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace telematics::net {
namespace {

constexpr size_t kMaxHeaderBytes = 64 * 1024;

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

std::string trim(std::string_view s) {
    const auto b = s.find_first_not_of(" \t");
    if (b == std::string_view::npos) return {};
    const auto e = s.find_last_not_of(" \t");
    return std::string(s.substr(b, e - b + 1));
}

// Rejects CR/LF so callers cannot inject extra headers or split requests.
void checkHeaderSafe(const std::string& s, const char* what) {
    if (s.find_first_of("\r\n") != std::string::npos || s.find('\0') != std::string::npos) {
        throw NetError(std::string("illegal character in HTTP ") + what);
    }
}

uint16_t defaultPort(const std::string& scheme) {
    if (scheme == "http") return 80;
    if (scheme == "https") return 443;
    if (scheme == "mqtt") return 1883;
    if (scheme == "mqtts") return 8883;
    throw NetError("unsupported URL scheme '" + scheme + "'");
}

}  // namespace

// ------------------------------------------------------------------------ Url

Url Url::parse(const std::string& url) {
    checkHeaderSafe(url, "URL");
    Url u;
    const auto sep = url.find("://");
    if (sep == std::string::npos) throw NetError("URL has no scheme: " + url);
    u.scheme = lower(url.substr(0, sep));
    const uint16_t def = defaultPort(u.scheme);

    const auto auth_start = sep + 3;
    const auto path_start = url.find_first_of("/?#", auth_start);
    std::string authority = url.substr(auth_start, path_start - auth_start);
    if (authority.find('@') != std::string::npos) throw NetError("credentials in URLs are not supported");

    std::string port_str;
    if (!authority.empty() && authority[0] == '[') {  // [IPv6]:port
        const auto close = authority.find(']');
        if (close == std::string::npos) throw NetError("bad IPv6 literal in URL");
        u.host = authority.substr(1, close - 1);
        if (close + 1 < authority.size()) {
            if (authority[close + 1] != ':') throw NetError("bad URL authority");
            port_str = authority.substr(close + 2);
        }
    } else {
        const auto colon = authority.rfind(':');
        u.host = authority.substr(0, colon);
        if (colon != std::string::npos) port_str = authority.substr(colon + 1);
    }
    if (u.host.empty()) throw NetError("URL has no host: " + url);

    if (port_str.empty()) {
        u.port = def;
    } else {
        if (port_str.find_first_not_of("0123456789") != std::string::npos || port_str.size() > 5) {
            throw NetError("bad port in URL: " + url);
        }
        const long p = std::strtol(port_str.c_str(), nullptr, 10);
        if (p < 1 || p > 65535) throw NetError("port out of range in URL: " + url);
        u.port = static_cast<uint16_t>(p);
    }

    u.target = path_start == std::string::npos ? "/" : url.substr(path_start);
    if (const auto hash = u.target.find('#'); hash != std::string::npos) u.target.erase(hash);  // never sent
    if (u.target.empty() || u.target[0] != '/') u.target.insert(0, "/");
    return u;
}

std::string Url::hostHeader() const {
    const std::string h = host.find(':') != std::string::npos ? "[" + host + "]" : host;
    return port == defaultPort(scheme) ? h : h + ":" + std::to_string(port);
}

std::optional<std::string> HttpResponse::header(const std::string& name) const {
    const std::string key = lower(name);
    for (const auto& [k, v] : headers) {
        if (k == key) return v;
    }
    return std::nullopt;
}

// ------------------------------------------------------------ chunked bodies

ChunkedStatus decodeChunked(std::string_view in, std::string& out, size_t& consumed) {
    out.clear();
    size_t pos = 0;
    for (;;) {
        const auto eol = in.find("\r\n", pos);
        if (eol == std::string_view::npos) {
            if (in.size() - pos > 1024) throw ProtocolError("chunk size line too long");
            return ChunkedStatus::NeedMore;
        }
        std::string_view line = in.substr(pos, eol - pos);
        if (const auto semi = line.find(';'); semi != std::string_view::npos) line = line.substr(0, semi);  // extensions
        const std::string size_hex = trim(line);
        if (size_hex.empty() || size_hex.size() > 8 || size_hex.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos) {
            throw ProtocolError("invalid chunk size '" + size_hex + "'");
        }
        const size_t size = std::strtoul(size_hex.c_str(), nullptr, 16);
        pos = eol + 2;

        if (size == 0) {  // last chunk, then optional trailers ending with an empty line
            for (;;) {
                const auto t_eol = in.find("\r\n", pos);
                if (t_eol == std::string_view::npos) return ChunkedStatus::NeedMore;
                const bool empty = t_eol == pos;
                pos = t_eol + 2;
                if (empty) {
                    consumed = pos;
                    return ChunkedStatus::Complete;
                }
            }
        }
        if (in.size() < pos + size + 2) return ChunkedStatus::NeedMore;
        if (in.substr(pos + size, 2) != "\r\n") throw ProtocolError("chunk not terminated by CRLF");
        out.append(in.substr(pos, size));
        pos += size + 2;
    }
}

// ----------------------------------------------------------------- HttpClient

HttpClient::HttpClient(std::shared_ptr<TlsContext> tls, milliseconds timeout) : tls_(std::move(tls)), timeout_(timeout) {}

HttpResponse HttpClient::post(const std::string& url, const std::string& body, const std::string& content_type,
                              Headers headers) {
    headers.emplace_back("Content-Type", content_type);
    return request("POST", url, headers, body);
}

HttpResponse HttpClient::request(const std::string& method, const std::string& url_str, const Headers& headers,
                                 const std::string& body) {
    const Url url = Url::parse(url_str);
    if (url.scheme != "http" && url.scheme != "https") throw NetError("not an HTTP URL: " + url_str);
    if (method.empty() || method.find_first_of(" \r\n") != std::string::npos) throw NetError("bad HTTP method");

    const auto deadline = Clock::now() + timeout_;

    // ---- connect
    TcpSocket tcp = TcpSocket::connect(url.host, url.port, timeout_);
    std::unique_ptr<IStream> stream;
    if (url.secure()) {
        if (!tls_) tls_ = TlsContext::client(TlsConfig{});
        stream = TlsStream::connect(tls_, std::move(tcp), url.host, remaining(deadline));
    } else {
        stream = std::make_unique<TcpSocket>(std::move(tcp));
    }

    // ---- request
    std::string req = method + " " + url.target + " HTTP/1.1\r\n";
    req += "Host: " + url.hostHeader() + "\r\n";
    bool has_ua = false;
    for (const auto& [k, v] : headers) {
        checkHeaderSafe(k, "header name");
        checkHeaderSafe(v, "header value");
        const std::string lk = lower(k);
        if (lk == "host" || lk == "content-length" || lk == "connection") continue;  // managed here
        has_ua |= lk == "user-agent";
        req += k + ": " + v + "\r\n";
    }
    if (!has_ua) req += "User-Agent: telematics-tcu/1.0\r\n";
    if (!body.empty() || method == "POST" || method == "PUT" || method == "PATCH") {
        req += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    }
    req += "Connection: close\r\n\r\n";
    writeString(*stream, req, remaining(deadline));
    if (!body.empty()) writeString(*stream, body, remaining(deadline));

    // ---- response head
    std::string buf;
    char chunk[8192];
    auto readMore = [&]() -> bool {
        const size_t n = stream->readSome(chunk, sizeof(chunk), remaining(deadline));
        buf.append(chunk, n);
        return n > 0;
    };

    HttpResponse resp;
    size_t head_end;
    for (;;) {  // loop skips interim 1xx responses
        while ((head_end = buf.find("\r\n\r\n")) == std::string::npos) {
            if (buf.size() > kMaxHeaderBytes) throw ProtocolError("HTTP response headers too large");
            if (!readMore()) throw NetError("connection closed before HTTP response headers");
        }
        const std::string head = buf.substr(0, head_end);
        buf.erase(0, head_end + 4);

        size_t line_end = head.find("\r\n");
        const std::string status_line = head.substr(0, line_end);
        if (status_line.compare(0, 7, "HTTP/1.") != 0 || status_line.size() < 12) {
            throw ProtocolError("bad HTTP status line: " + status_line);
        }
        resp = HttpResponse{};
        resp.status = std::atoi(status_line.substr(9, 3).c_str());
        resp.reason = status_line.size() > 13 ? status_line.substr(13) : "";

        while (line_end != std::string::npos) {
            const size_t start = line_end + 2;
            line_end = head.find("\r\n", start);
            const std::string line = head.substr(start, line_end == std::string::npos ? std::string::npos : line_end - start);
            const auto colon = line.find(':');
            if (colon == std::string::npos) continue;
            resp.headers.emplace_back(lower(trim(line.substr(0, colon))), trim(line.substr(colon + 1)));
        }
        if (resp.status >= 200 || resp.status == 101) break;
    }

    // ---- response body
    const bool no_body = method == "HEAD" || resp.status == 204 || resp.status == 304;
    const auto te = resp.header("transfer-encoding");
    const auto cl = resp.header("content-length");

    if (no_body) {
        // nothing to read
    } else if (te && lower(*te).find("chunked") != std::string::npos) {
        size_t consumed = 0;
        while (decodeChunked(buf, resp.body, consumed) == ChunkedStatus::NeedMore) {
            if (buf.size() > max_body_bytes) throw ProtocolError("HTTP response body too large");
            if (!readMore()) throw NetError("connection closed inside chunked body");
        }
    } else if (cl) {
        if (cl->empty() || cl->find_first_not_of("0123456789") != std::string::npos) {
            throw ProtocolError("invalid Content-Length");
        }
        const size_t len = std::strtoull(cl->c_str(), nullptr, 10);
        if (len > max_body_bytes) throw ProtocolError("HTTP response body too large");
        while (buf.size() < len) {
            if (!readMore()) throw NetError("connection closed before full HTTP body");
        }
        resp.body = buf.substr(0, len);
    } else {
        while (readMore()) {
            if (buf.size() > max_body_bytes) throw ProtocolError("HTTP response body too large");
        }
        resp.body = std::move(buf);
    }
    stream->close();
    return resp;
}

}  // namespace telematics::net
