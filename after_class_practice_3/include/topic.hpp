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
    Topic() = default; // 默认构造函数
    Topic(const Topic&) = delete; // 禁止拷贝构造
    Topic& operator=(const Topic&) = delete; // 禁止赋值操作

public:
    // 获取单例实例
    static Topic& getInstance() {
        static Topic instance; // 局部静态变量，线程安全的单例
        return instance; // 返回实例引用
    }

    // 向队列中添加数据
    void push(const T& value) {
        {
            std::lock_guard<std::mutex> lock(mutex_); // 获取锁
            queue_.push(value);
        }
        cond_var_.notify_one();
    }

    // 从队列中取出数据, 如果没数据则阻塞等待
    T pop() {
        std::unique_lock<std::mutex> lock(mutex_); // 获取锁
        cond_var_.wait(lock, [this] { return !queue_.empty(); }); // 等待直到队列非空
        
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