#include "telematics/store_forward.hpp"

#include <algorithm>

namespace telematics {

StoreAndForward::StoreAndForward(ITransport& transport, size_t capacity, size_t batch_size,
                                 std::chrono::milliseconds base_backoff,
                                 std::chrono::milliseconds max_backoff)
    : transport_(transport),
      capacity_(std::max<size_t>(capacity, 1)),
      batch_size_(std::max<size_t>(batch_size, 1)),
      base_backoff_(base_backoff),
      max_backoff_(max_backoff) {}

StoreAndForward::~StoreAndForward() { stop(false); }

void StoreAndForward::enqueue(std::string payload, Priority p) {
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (queue_.size() >= capacity_) {
            // Evict the oldest Normal message; alerts are kept as long as possible.
            auto it = std::find_if(queue_.begin(), queue_.end(),
                                   [](const Message& m) { return m.priority == Priority::Normal; });
            if (it != queue_.end()) {
                queue_.erase(it);
            } else if (p == Priority::Normal) {
                ++stats_.dropped;  // queue is all alerts; drop the new low-priority message
                return;
            } else {
                queue_.pop_front();
            }
            ++stats_.dropped;
        }

        Message m{std::move(payload), p, next_seq_++};
        if (p == Priority::High) {
            // Insert after existing High messages, preserving FIFO among alerts.
            auto pos = std::find_if(queue_.begin(), queue_.end(),
                                    [](const Message& x) { return x.priority == Priority::Normal; });
            queue_.insert(pos, std::move(m));
        } else {
            queue_.push_back(std::move(m));
        }
        ++stats_.enqueued;
    }
    cv_.notify_one();
}

std::vector<Message> StoreAndForward::peekBatch() const {
    std::lock_guard<std::mutex> lk(mu_);
    const size_t n = std::min(batch_size_, queue_.size());
    return {queue_.begin(), queue_.begin() + static_cast<std::ptrdiff_t>(n)};
}

bool StoreAndForward::flushOnce() {
    auto batch = peekBatch();
    if (batch.empty()) return true;
    // Messages stay queued until the transport acknowledges them, so a crash
    // or failed send never loses data. A High message enqueued during the send
    // would shift the front of the queue, so we remove acknowledged messages by
    // sequence number rather than by position.
    if (!transport_.send(batch)) {
        std::lock_guard<std::mutex> lk(mu_);
        ++stats_.failed_attempts;
        return false;
    }
    std::lock_guard<std::mutex> lk(mu_);
    for (const auto& sent : batch) {
        auto it = std::find_if(queue_.begin(), queue_.end(),
                               [&](const Message& m) { return m.seq == sent.seq; });
        if (it != queue_.end()) {
            queue_.erase(it);
            ++stats_.sent;
        }
    }
    return true;
}

void StoreAndForward::run() {
    auto backoff = base_backoff_;
    while (running_) {
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, std::chrono::milliseconds(200),
                         [&] { return !queue_.empty() || !running_; });
            if (!running_) break;
            if (queue_.empty()) continue;
        }
        if (flushOnce()) {
            backoff = base_backoff_;
        } else {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, backoff, [&] { return !running_.load(); });
            backoff = std::min(backoff * 2, max_backoff_);
        }
    }
}

void StoreAndForward::start() {
    if (running_.exchange(true)) return;
    worker_ = std::thread(&StoreAndForward::run, this);
}

void StoreAndForward::stop(bool drain) {
    if (running_.exchange(false)) {
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }
    if (drain) {
        for (int attempts = 0; pending() > 0 && attempts < 10; ++attempts) {
            if (!flushOnce()) break;
        }
    }
}

size_t StoreAndForward::pending() const {
    std::lock_guard<std::mutex> lk(mu_);
    return queue_.size();
}

UplinkStats StoreAndForward::stats() const {
    std::lock_guard<std::mutex> lk(mu_);
    return stats_;
}

}  // namespace telematics
