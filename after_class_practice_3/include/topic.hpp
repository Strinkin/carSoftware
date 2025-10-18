#ifndef __TOPIC_HPP__
#define __TOPIC_HPP__

#include <queue>
#include <mutex>
#include <memory>
#include <condition_variable>

template <typename T>
class Topic {
private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_var_;
    
    // 私有构造函数，确保单例
    Topic() = default;
    Topic(const Topic&) = delete;
    Topic& operator=(const Topic&) = delete;

public:
    // 获取单例实例
    static Topic& getInstance() {
        static Topic instance;
        return instance;
    }

    // 向队列中添加数据
    void push(const T& value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(value);
        }
        cond_var_.notify_one();
    }

    // 从队列中取出数据
    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this] { return !queue_.empty(); });
        
        T value = queue_.front();
        queue_.pop();
        return value;
    }

    // 尝试从队列中取出数据，如果队列为空则立即返回false
    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        
        value = queue_.front();
        queue_.pop();
        return true;
    }

    // 检查队列是否为空
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    // 获取队列大小
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
};

#endif // __TOPIC_HPP__