/*!
 * Copyright (c) 2017 by Contributors
 * \file blocking_queue.hpp
 * \brief
 * \author liangzhujin
 */

#ifndef HOBOT_VISION_BLOCKING_QUEUE_HPP_
#define HOBOT_VISION_BLOCKING_QUEUE_HPP_

#include <deque>
#include <mutex>
#include <atomic>
#include <string>
#include <chrono>
#include <condition_variable>

namespace hobot {
namespace vision {

template<typename T>
class BlockingQueue {
 public:
  BlockingQueue() {}

  void push_front(const T &t) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push_front(t);
    size_.store(queue_.size());;
    lock.unlock();
    condition_.notify_one();
  }

  void push_back(const T &t) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push_back(t);
    size_.store(queue_.size());;
    lock.unlock();
    condition_.notify_one();
  }

  void push(const T &t) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push_back(t);
    size_.store(queue_.size());;
    lock.unlock();
    condition_.notify_one();
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
    size_.store(queue_.size());;
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
    size_.store(queue_.size());;
    lock.unlock();
    return t;
  }

  T pop_back() {
    std::unique_lock<std::mutex> lock(mutex_);
    int wait_count = 0;
    condition_.wait(lock, [this] { return !queue_.empty(); });

    T t = queue_.back();
    queue_.clear();
    size_.store(queue_.size());
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
    return size_.load();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    size_.store(queue_.size());
  }

 protected:
  std::deque<T> queue_;
  std::atomic<size_t> size_;
  std::mutex mutex_;
  std::condition_variable condition_;

 private:
  BlockingQueue(const BlockingQueue &);
  BlockingQueue &operator=(const BlockingQueue &);
};

}  // namespace vision
}  // namespace hobot

#endif  // HOBOT_VISION_BLOCKING_QUEUE_HPP_
