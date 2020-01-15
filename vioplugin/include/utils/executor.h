/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @file      executor.h
 * @brief
 * @author    ruoting.ding
 * @email     ruoting.ding@horizon.ai
 * @date      2019.08.10
 */

#ifndef HORIZON_EXECUTOR_HPP_
#define HORIZON_EXECUTOR_HPP_

#include <future>
#include <vector>
#include <list>
//#include "hobot_vision/blocking_queue.hpp"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace vioplugin {

class Executor {
 public:
  /// such as InputProducer::Run
  using exe_func = std::function<int()>;
  static std::shared_ptr<Executor> GetInstance();
  Executor();
  ~Executor();
  void Run();
  std::future<bool> AddTask(exe_func);
  int Pause();
  int Resume();
  int Stop();

 private:
  struct Task {
    std::shared_ptr<std::promise<bool>> p_;
    exe_func func_;
  };
  typedef std::list<std::shared_ptr<Task> > TaskContainer;
  std::atomic_bool stop_;
  std::atomic_bool pause_;
  std::condition_variable condition_;
  TaskContainer task_queue_;
  using ThreadPtr = std::shared_ptr<std::thread>;
  std::vector<ThreadPtr> threads_;
  mutable std::mutex task_queue_mutex_;
  static std::once_flag flag_;
  static std::shared_ptr<Executor> worker_;
  int thread_count_ = 2;
};

}  // namespace vioplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon

#endif  // HORIZON_INPUT_PRODUCER_HPP_
