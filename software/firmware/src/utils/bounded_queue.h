#ifndef OSSM_UTILS_BOUNDED_QUEUE_H
#define OSSM_UTILS_BOUNDED_QUEUE_H

#include <algorithm>
#include <cstddef>
#include <deque>
#include <mutex>

// FIFO shared between tasks: the BLE host task, the RAD BLE task and the
// NimBLE loop push, one worker pops. std::queue on its own is not safe for
// that (a push racing a pop corrupts the heap), and an unbounded queue lets
// a BLE write flood exhaust internal RAM while nothing consumes it.
template <typename T, std::size_t Capacity>
class BoundedQueue {
  public:
    static constexpr std::size_t capacity = Capacity;

    // Appends `item`. When full, drops the oldest item if `dropOldest`,
    // otherwise drops `item`. Returns false when something was dropped.
    bool push(const T &item, bool dropOldest = false) {
        std::lock_guard<std::mutex> guard(lock_);
        if (items_.size() >= Capacity) {
            ++dropped_;
            if (!dropOldest) return false;
            items_.pop_front();
            items_.push_back(item);
            return false;
        }
        items_.push_back(item);
        return true;
    }

    // Empties the queue, then appends `item` (a stop must never be dropped).
    void replaceAll(const T &item) {
        std::lock_guard<std::mutex> guard(lock_);
        dropped_ += items_.size();
        items_.clear();
        items_.push_back(item);
    }

    bool pop(T &out) {
        std::lock_guard<std::mutex> guard(lock_);
        if (items_.empty()) return false;
        out = items_.front();
        items_.pop_front();
        return true;
    }

    bool peek(T &out) const {
        std::lock_guard<std::mutex> guard(lock_);
        if (items_.empty()) return false;
        out = items_.front();
        return true;
    }

    // Retire only work belonging to the lost/superseded authority.
    template <class Predicate>
    void removeIf(Predicate predicate) {
        std::lock_guard<std::mutex> guard(lock_);
        items_.erase(std::remove_if(items_.begin(), items_.end(), predicate), items_.end());
    }

    void clear() {
        std::lock_guard<std::mutex> guard(lock_);
        items_.clear();
        items_.shrink_to_fit();
    }

    bool empty() const {
        std::lock_guard<std::mutex> guard(lock_);
        return items_.empty();
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> guard(lock_);
        return items_.size();
    }

    // Items dropped because the queue was full (or replaced), since boot.
    std::size_t dropped() const {
        std::lock_guard<std::mutex> guard(lock_);
        return dropped_;
    }

  private:
    mutable std::mutex lock_;
    std::deque<T> items_;
    std::size_t dropped_ = 0;
};

#endif  // OSSM_UTILS_BOUNDED_QUEUE_H
