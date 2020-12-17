#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <utility>
#include <boost/lexical_cast.hpp>

std::string getenv(const char *variable_name, const char *default_value) {
    const char* value = getenv(variable_name);
    return value ? value : default_value;
}

template<typename T>
T getenv(const char *variable_name, T default_value) {
    const char* value = getenv(variable_name);
    return value ? boost::lexical_cast<T>(value) : default_value;
}

template<typename T>
class concurrent_blocking_queue {
public:
    concurrent_blocking_queue() {
        closed = false;
    }

    bool pop(T &item) {
        std::unique_lock<std::mutex> lock(m);
        while (q.empty() && !closed) {
            c.wait(lock);
        }
        if (closed) {
            return false;
        }
        item = q.front();
        q.pop();
        return true;
    }

    void push(const T &item) {
        std::unique_lock<std::mutex> lock(m);
        q.push(item);
        lock.unlock();
        c.notify_one();
    }

    void push(T &&item) {
        std::unique_lock<std::mutex> lock(m);
        q.push(std::move(item));
        lock.unlock();
        c.notify_one();
    }

    void close() {
        closed = true;
        c.notify_all();
    }

    int size() {
        std::unique_lock<std::mutex> lock(m);
        return q.size();
    }

private:
    std::queue<T> q;
    std::mutex m;
    std::condition_variable c;
    bool closed;
};
