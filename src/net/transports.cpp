#include "telematics/net/transports.hpp"

namespace telematics::net {

// ------------------------------------------------------------- MqttTransport

namespace {
MqttClientConfig withStatusWill(MqttClientConfig cfg, const std::string& prefix) {
    if (!cfg.connect.will) cfg.connect.will = mqtt::Will{prefix + "/status", "offline", 1, true};
    return cfg;
}
}  // namespace

MqttTransport::MqttTransport(MqttClientConfig cfg, std::string topic_prefix)
    : prefix_(std::move(topic_prefix)), client_(withStatusWill(std::move(cfg), prefix_)) {}

void MqttTransport::ensureConnected() {
    if (client_.connected()) return;
    client_.connect();
    client_.publish(prefix_ + "/status", "online", 1, /*retain=*/true);
}

std::string MqttTransport::topicFor(const std::string& payload) const {
    static const std::string key = "\"type\":\"";
    const auto start = payload.find(key);
    if (start != std::string::npos) {
        const auto b = start + key.size();
        const auto e = payload.find('"', b);
        if (e != std::string::npos && e > b) {
            const std::string type = payload.substr(b, e - b);
            if (mqtt::isValidTopicName(type) && type.find('/') == std::string::npos) return prefix_ + "/" + type;
        }
    }
    return prefix_ + "/telemetry";
}

bool MqttTransport::send(const std::vector<Message>& batch) {
    try {
        ensureConnected();
        // At-least-once: if the connection fails mid-batch, the whole batch is
        // retried, so the backend should de-duplicate (e.g. on vin + ts + type).
        for (const auto& m : batch) client_.publish(topicFor(m.payload), m.payload, 1, false);
        return true;
    } catch (const std::exception& e) {
        if (on_error) on_error(std::string("MQTT: ") + e.what());
        client_.disconnect();
        return false;
    }
}

// ------------------------------------------------------------ HttpsTransport

HttpsTransport::HttpsTransport(std::string url, std::shared_ptr<TlsContext> tls, std::string bearer_token,
                               milliseconds timeout)
    : url_(std::move(url)), token_(std::move(bearer_token)), http_(std::move(tls), timeout) {
    Url::parse(url_);  // validate early
}

bool HttpsTransport::send(const std::vector<Message>& batch) {
    std::string body = "[";
    for (size_t i = 0; i < batch.size(); ++i) {
        if (i) body += ',';
        body += batch[i].payload;
    }
    body += ']';

    Headers headers;
    if (!token_.empty()) headers.emplace_back("Authorization", "Bearer " + token_);
    headers.emplace_back("X-Batch-Size", std::to_string(batch.size()));

    try {
        const HttpResponse r = http_.post(url_, body, "application/json", headers);
        if (r.ok()) return true;
        const bool retryable = r.status == 408 || r.status == 429 || r.status >= 500;
        if (on_error) on_error("HTTPS: server returned " + std::to_string(r.status) + " " + r.reason);
        if (retryable) return false;
        rejected_ += batch.size();
        return true;  // permanent rejection: drop rather than block the queue forever
    } catch (const std::exception& e) {
        if (on_error) on_error(std::string("HTTPS: ") + e.what());
        return false;
    }
}

}  // namespace telematics::net
