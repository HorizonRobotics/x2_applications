/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Method interface of xsoul framework
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */

#ifndef COMMON_CONCURRENCY_QUEUE_H_
#define COMMON_CONCURRENCY_QUEUE_H_

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>

namespace HobotXRoc {

template <typename T>
class ConcurrencyQueue {
 public:
  ConcurrencyQueue() {}

  void push_front(const T &t) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push_front(t);
    lock.unlock();
    condition_.notify_all();
  }

  void push_back(const T &t) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push_back(t);
    lock.unlock();
    condition_.notify_all();
  }

  void push(const T &t) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push_back(t);
    lock.unlock();
    condition_.notify_all();
  }

  bool try_pop(T *t, const std::chrono::microseconds &timeout =
                         std::chrono::microseconds(0)) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!condition_.wait_for(lock, timeout,
                             [this] { return !queue_.empty(); })) {
      lock.unlock();
      return false;
    }

    *t = queue_.front();
    queue_.pop_front();
    lock.unlock();
    return true;
  }

  // This logs a message if the threads needs to be blocked
  // useful for detecting e.g. when data feeding is too slow
  T pop() {
    std::unique_lock<std::mutex> lock(mutex_);

    condition_.wait(lock, [this] { return !queue_.empty(); });

    T t = queue_.front();
    queue_.pop_front();
    lock.unlock();
    return t;
  }

  T pop_back() {
    std::unique_lock<std::mutex> lock(mutex_);
    int wait_count = 0;
    condition_.wait(lock, [this] { return !queue_.empty(); });

    T t = queue_.back();
    queue_.pop_back();
    lock.unlock();
    return t;
  }

  bool try_peek(T *t) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      lock.unlock();
      return false;
    }

    *t = queue_.front();
    lock.unlock();
    return true;
  }

  // Return element without removing it
  T peek() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return !queue_.empty(); });

    lock.unlock();
    return queue_.front();
  }

  size_t size() {
    std::unique_lock<std::mutex> lock(mutex_);
    const size_t ret = queue_.size();
    lock.unlock();
    return ret;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
  }

 protected:
  std::deque<T> queue_;

  std::mutex mutex_;
  std::condition_variable condition_;

 private:
  ConcurrencyQueue(const ConcurrencyQueue &);

  ConcurrencyQueue &operator=(const ConcurrencyQueue &);
};
}  // namespace HobotXRoc

#endif  // COMMON_CONCURRENCY_QUEUE_H_
