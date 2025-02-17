#pragma once

#include <mutex>
#include <condition_variable>
#include <deque>
#include <ostream>

namespace utils{
    template <typename T>
    class sharedQueue
    {
    private:
        std::mutex              d_mutex;
        std::condition_variable d_condition;
    public:
        void push(T const& value) {
            {
                d_queue.push_front(value);
            }
            this->d_condition.notify_one();
        }
        T pop() {
            std::unique_lock<std::mutex> lock(this->d_mutex);
            this->d_condition.wait(lock, [=]{ return !this->d_queue.empty(); });
            T rc(std::move(this->d_queue.back()));
            this->d_queue.pop_back();
            return rc;
        }
    std::deque<T> d_queue;
    };

    enum Color {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 49
    };
    std::ostream& operator<<(std::ostream& os, Color code) {
        return os << "\033[" << static_cast<int>(code) << "m";
    }
}
