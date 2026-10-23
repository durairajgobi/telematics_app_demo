#pragma once
// ITransport implementations that connect the store-and-forward uplink to a
// cloud backend over MQTT (TLS) or HTTPS.
#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "telematics/net/http_client.hpp"
#include "telematics/net/mqtt_client.hpp"
#include "telematics/store_forward.hpp"

namespace telematics::net {

// Publishes each message with QoS 1 to "<topic_prefix>/<type>", where type is
// the "type" field of the JSON payload (telemetry, event, trip).
// Maintains "<topic_prefix>/status" = online/offline using a retained last will,
// so the backend knows when a vehicle drops off the network.
class MqttTransport : public ITransport {
public:
    MqttTransport(MqttClientConfig cfg, std::string topic_prefix);
    bool send(const std::vector<Message>& batch) override;

    MqttClient& client() { return client_; }
    std::function<void(const std::string&)> on_error;  // optional diagnostics hook

private:
    void ensureConnected();
    std::string topicFor(const std::string& payload) const;

    std::string prefix_;
    MqttClient client_;
};

// POSTs each batch as a JSON array. 2xx = delivered. 408/429/5xx/network
// errors = retry later. Other 4xx are permanent rejections: the batch is
// dropped (counted in rejected()) so one bad message cannot block the queue.
class HttpsTransport : public ITransport {
public:
    HttpsTransport(std::string url, std::shared_ptr<TlsContext> tls, std::string bearer_token = {},
                   milliseconds timeout = milliseconds(15'000));
    bool send(const std::vector<Message>& batch) override;

    uint64_t rejected() const { return rejected_; }
    std::function<void(const std::string&)> on_error;

private:
    std::string url_;
    std::string token_;
    HttpClient http_;
    std::atomic<uint64_t> rejected_{0};
};

}  // namespace telematics::net
