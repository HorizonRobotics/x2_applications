//
// Created by yaoyao.sun on 2019-05-12.
//

#ifndef BLOCK_QUEUE_H
#define BLOCK_QUEUE_H

#include <assert.h>
#include <mutex>
#include <condition_variable>
#include <deque>


template <typename T>
class BlockQueue {
 public:
  BlockQueue()
    : mutex_(),
      not_empty_(),
      queue_() { }
  void Push(const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(value);
    not_empty_.notify_one();
  }
  void Push(T &&value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(std::move(value));
    not_empty_.notify_one();
  }
  T Take() {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this]{return !this->queue_.empty();});
    assert(!queue_.empty());
    T front(std::move(queue_.front()));
    queue_.pop_front();
    return std::move(front);
  }
  size_t size() const {
    return queue_.size();
  }
 private:
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::deque<T> queue_;

};

#endif //BLOCK_QUEUE_H
