#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

/// Thread-safe bounded FIFO queue with drop-oldest overflow policy.
///
/// Producer never blocks: when the queue is full, the oldest item is
/// discarded to make room for the new one and an internal drop counter
/// is incremented. Consumer can wait with a timeout.
///
/// Designed for a single producer / single consumer pattern but works
/// for multiple producers/consumers too (all operations are mutex-guarded).
template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(std::size_t capacity)
        : cap_(capacity == 0 ? 1 : capacity) {}

    /// Producer side. Never blocks. If the queue is full, drops the
    /// oldest element and increments drop_count_.
    void push_drop_oldest(T item) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (q_.size() >= cap_) {
                q_.pop_front();
                ++drops_;
            }
            q_.push_back(std::move(item));
        }
        cv_.notify_one();
    }

    /// Consumer side. Waits up to `timeout` for an item.
    /// Returns true if `out` was populated, false on timeout or shutdown.
    bool pop_wait(T& out, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lk(mtx_);
        if (!cv_.wait_for(lk, timeout, [this] {
                return !q_.empty() || closed_;
            })) {
            return false; // timeout
        }
        if (q_.empty()) return false; // shutdown with empty queue
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }

    /// Wake every waiting consumer so they can exit.
    void shutdown() {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            closed_ = true;
        }
        cv_.notify_all();
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return q_.size();
    }

    std::size_t drop_count() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return drops_;
    }

    std::size_t capacity() const { return cap_; }

private:
    mutable std::mutex      mtx_;
    std::condition_variable cv_;
    std::deque<T>           q_;
    std::size_t             cap_;
    std::size_t             drops_  = 0;
    bool                    closed_ = false;
};
