/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     common functions
 * @author    jianbo.qin
 * @email     jianbo.qin@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.22
 */

#ifndef TIMER_TIMER_H_
#define TIMER_TIMER_H_

#include <functional>
#include <list>
#include <mutex>
#include <thread>
#include <vector>

namespace HobotXRoc {
class TimerManager;

class TimerTask {
 public:
  enum TimerType { ONCE, CIRCLE };

  explicit TimerTask(TimerManager &manager);

  ~TimerTask();

  void Start(std::function<void()> fun, unsigned interval,
             TimerType timeType = ONCE);

  void Stop();

 private:
  void OnTimer(uint64_t now);

 private:
  friend class TimerManager;

  TimerManager &manager_;
  TimerType timer_type_;
  std::function<void()> callback_fun_;
  uint32_t interval_;
  uint64_t expires_;

  int vecIndex_;
  std::list<TimerTask *>::iterator itr_;
};

class TimerManager {
 public:
  TimerManager();

  static uint64_t GetCurrentMillisecs();

  void DetectTimers();

 private:
  friend class TimerTask;

  void AddTimer(TimerTask *timer);

  void RemoveTimer(TimerTask *timer);

  int Cascade(int offset, int index);

 private:
  typedef std::list<TimerTask *> TimeList;
  std::vector<TimeList> tvec_;
  uint64_t check_time_;
};

class Timer {
 public:
  static Timer *Instance();

  void Init();

  void Final();

  void *AddTimer(std::function<void()> callback, uint32_t interval,
                 TimerTask::TimerType timeType = TimerTask::ONCE);

  void RemoveTimer(void *&ptr);

 private:
  Timer() {
    is_stop_ = false;
    thread_ptr_ = NULL;
  }

  static Timer *inst_;

 private:
  bool is_stop_;
  TimerManager timer_manager_;
  static std::mutex mutex_;
  std::thread *thread_ptr_;
};
}  // namespace HobotXRoc
#endif  // TIMER_TIMER_H_
