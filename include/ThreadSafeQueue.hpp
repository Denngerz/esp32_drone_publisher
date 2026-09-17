#pragma once
#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>

// Minimal thread-safe FIFO used to pass commands from the mission thread to
// the physics thread.
template <typename T>
class ThreadSafeQueue
{
public:
    void push(T value)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(std::move(value));
        }
        cv_.notify_one();
    }

    // Non-blocking pop; std::nullopt if the queue is empty.
    std::optional<T> tryPop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
            return std::nullopt;
        T value = std::move(queue_.front());
        queue_.pop();
        return value;
    }

    // Drain the queue, returning the most recently pushed element (if any).
    // Handy for "latest command wins" semantics.
    std::optional<T> drainLatest()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::optional<T> latest;
        while (!queue_.empty())
        {
            latest = std::move(queue_.front());
            queue_.pop();
        }
        return latest;
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    mutable std::mutex      mutex_;
    std::condition_variable cv_;
    std::queue<T>           queue_;
};
