#pragma once
// Store-and-forward uplink: bounded, thread-safe queue with priorities,
// batching and exponential back-off, so data survives cellular dropouts.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace telematics {

enum class Priority { Normal, High };  // High = events/alerts, never dropped first

struct Message {
    std::string payload;
    Priority priority = Priority::Normal;
    uint64_t seq = 0;
};

// Abstract transport (MQTT, HTTPS, SMS...). Return true on acknowledged delivery.
class ITransport {
public:
    virtual ~ITransport() = default;
    virtual bool send(const std::vector<Message>& batch) = 0;
};

struct UplinkStats {
    uint64_t enqueued = 0;
    uint64_t sent = 0;
    uint64_t dropped = 0;
    uint64_t failed_attempts = 0;
};

class StoreAndForward {
public:
    StoreAndForward(ITransport& transport, size_t capacity = 10'000, size_t batch_size = 20,
                    std::chrono::milliseconds base_backoff = std::chrono::milliseconds(100),
                    std::chrono::milliseconds max_backoff = std::chrono::milliseconds(5000));
    ~StoreAndForward();

    StoreAndForward(const StoreAndForward&) = delete;
    StoreAndForward& operator=(const StoreAndForward&) = delete;

    // Non-blocking. When full, the oldest Normal message is evicted; if the
    // queue is entirely High priority, the oldest message is evicted.
    void enqueue(std::string payload, Priority p = Priority::Normal);

    void start();                   // background sender thread
    void stop(bool drain = true);   // optionally try to flush before stopping
    bool flushOnce();               // synchronous single batch (used by tests / stop)

    size_t pending() const;
    UplinkStats stats() const;

private:
    void run();
    std::vector<Message> peekBatch() const;

    ITransport& transport_;
    const size_t capacity_;
    const size_t batch_size_;
    const std::chrono::milliseconds base_backoff_;
    const std::chrono::milliseconds max_backoff_;

    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Message> queue_;  // High-priority messages are kept at the front
    UplinkStats stats_;
    uint64_t next_seq_ = 0;

    std::atomic<bool> running_{false};
    std::thread worker_;
};

}  // namespace telematics
