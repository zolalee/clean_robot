#ifndef QUEUE_H
#define QUEUE_H

#include <queue>
#include <mutex>

namespace core {

template<typename T>
class Queue
{
public:
    Queue() {

    }

    bool enqueue(T e) {
        std::unique_lock<std::mutex> lck (mutex_);
        queue_.push(e);
        return true;
    }

    bool dequeue(T &e) {
        std::unique_lock<std::mutex> lck (mutex_);
        if (queue_.empty())
            return false;
        e = queue_.front();
        queue_.pop();
        return true;
    }

    void clear() {
        std::unique_lock<std::mutex> lck (mutex_);
        std::queue<T> empty;
        std::swap(empty, queue_);
    }

    size_t size() {
        std::unique_lock<std::mutex> lck (mutex_);
        return queue_.size();
    }

private:
    std::queue<T> queue_;
    std::mutex  mutex_;
};

}

#endif
