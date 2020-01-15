//
// Created by jianbo on 11/22/18.
//

#define _CRT_SECURE_NO_WARNINGS

#include "timer/timer.h"

#ifdef _MSC_VER
# include <sys/timeb.h>
#else

# include <sys/time.h>

#endif

namespace HobotXRoc {

#define TVN_BITS 6
#define TVR_BITS 8
#define TVN_SIZE (1 << TVN_BITS)  //64
#define TVR_SIZE (1 << TVR_BITS)  //256
#define TVN_MASK (TVN_SIZE - 1)   //64
#define TVR_MASK (TVR_SIZE - 1)   //255
#define OFFSET(N) (TVR_SIZE + (N) *TVN_SIZE) //256+N*64
#define INDEX(V, N) ((V >> (TVR_BITS + (N) *TVN_BITS)) & TVN_MASK)

//////////////////////////////////////////////////////////////////////////
// TimerTask

TimerTask::TimerTask(TimerManager &manager)
    : manager_(manager), vecIndex_(-1) {
}

TimerTask::~TimerTask() {
  Stop();
}

void TimerTask::Start(std::function<void()> fun, unsigned interval, TimerType timeType) {
  Stop();
  interval_ = interval;
  callback_fun_ = fun;
  timer_type_ = timeType;
  expires_ = interval_ + TimerManager::GetCurrentMillisecs();
  manager_.AddTimer(this);
}

void TimerTask::Stop() {
  if (vecIndex_ != -1) {
    manager_.RemoveTimer(this);
    vecIndex_ = -1;
  }
}

void TimerTask::OnTimer(uint64_t now) {
  if (timer_type_ == TimerTask::CIRCLE) {
    expires_ = interval_ + now;
    manager_.AddTimer(this);
  } else {
    vecIndex_ = -1;
  }
  callback_fun_();
}

//////////////////////////////////////////////////////////////////////////
// TimerManager

TimerManager::TimerManager() {
  tvec_.resize(TVR_SIZE + 4 * TVN_SIZE);
  check_time_ = GetCurrentMillisecs();
}

void TimerManager::AddTimer(TimerTask *timer) {
  unsigned long long expires = timer->expires_;
  unsigned long long idx = expires - check_time_;

  if (idx < TVR_SIZE) {
    timer->vecIndex_ = expires & TVR_MASK;
  } else if (idx < 1 << (TVR_BITS + TVN_BITS)) {
    timer->vecIndex_ = OFFSET(0) + INDEX(expires, 0);
  } else if (idx < 1 << (TVR_BITS + 2 * TVN_BITS)) {
    timer->vecIndex_ = OFFSET(1) + INDEX(expires, 1);
  } else if (idx < 1 << (TVR_BITS + 3 * TVN_BITS)) {
    timer->vecIndex_ = OFFSET(2) + INDEX(expires, 2);
  } else if ((long long) idx < 0) {
    timer->vecIndex_ = check_time_ & TVR_MASK;
  } else {
    if (idx > 0xffffffffUL) {
      idx = 0xffffffffUL;
      expires = idx + check_time_;
    }
    timer->vecIndex_ = OFFSET(3) + INDEX(expires, 3);
  }

  TimeList &tlist = tvec_[timer->vecIndex_];
  tlist.push_back(timer);
  timer->itr_ = tlist.end();
  --timer->itr_;
}

void TimerManager::RemoveTimer(TimerTask *timer) {
  TimeList &tlist = tvec_[timer->vecIndex_];
  tlist.erase(timer->itr_);
}

void TimerManager::DetectTimers() {
  unsigned long long now = GetCurrentMillisecs();
  while (check_time_ <= now) {
    int index = check_time_ & TVR_MASK;
    if (!index &&
        !Cascade(OFFSET(0), INDEX(check_time_, 0)) &&
        !Cascade(OFFSET(1), INDEX(check_time_, 1)) &&
        !Cascade(OFFSET(2), INDEX(check_time_, 2))) {
      Cascade(OFFSET(3), INDEX(check_time_, 3));
    }
    ++check_time_;

    TimeList temp;
    TimeList &tlist = tvec_[index];
    temp.splice(temp.end(), tlist);

    for (TimeList::iterator itr = temp.begin(); itr != temp.end(); ++itr) {
      (*itr)->OnTimer(now);
    }
  }
}

int TimerManager::Cascade(int offset, int index) {
  TimeList &tlist = tvec_[offset + index];
  TimeList temp;
  temp.splice(temp.end(), tlist);

  for (TimeList::iterator itr = temp.begin(); itr != temp.end(); ++itr) {
    AddTimer(*itr);
  }

  return index;
}

uint64_t TimerManager::GetCurrentMillisecs() {
#ifdef _MSC_VER
  _timeb timebuffer;
    _ftime(&timebuffer);
    unsigned long long ret = timebuffer.time;
    ret = ret * 1000 + timebuffer.millitm;
    return ret;
#else
  timeval tv;
  ::gettimeofday(&tv, 0);
  unsigned long long ret = tv.tv_sec;
  return ret * 1000 + tv.tv_usec / 1000;
#endif
}

std::mutex Timer::mutex_;
Timer *Timer::inst_ = nullptr;

Timer *Timer::Instance() {
  if (NULL == inst_) {
    std::lock_guard<std::mutex> guard(Timer::mutex_);
    if (NULL == inst_) {
      inst_ = new Timer();
      inst_->Init();
    }
  }
  return inst_;
}

void Timer::Init() {
  is_stop_ = false;
  thread_ptr_ = new std::thread([this] {
    std::chrono::milliseconds dur(128);
    while (!is_stop_) {
      {
        std::lock_guard<std::mutex> guard(Timer::mutex_);
        timer_manager_.DetectTimers();
      }
      std::this_thread::sleep_for(dur);
    }
  });
}

void Timer::Final() {
  std::lock_guard<std::mutex> guard(Timer::mutex_);
  is_stop_ = true;
  if (thread_ptr_->joinable()) {
    thread_ptr_->join();
  }
}

void *Timer::AddTimer(std::function<void()> callback, uint32_t interval, TimerTask::TimerType timeType) {
  if (interval < 512) {
    return NULL;
  }
  TimerTask *tmp = new TimerTask(timer_manager_);
  std::lock_guard<std::mutex> guard(Timer::mutex_);
  tmp->Start(callback, interval, timeType);
  return reinterpret_cast<void *>(tmp);
}

void Timer::RemoveTimer(void *&ptr) {
  if (NULL == ptr) {
    return;
  }
  TimerTask *timer = reinterpret_cast<TimerTask *>(ptr);
  ptr = NULL;
  std::lock_guard<std::mutex> guard(Timer::mutex_);
  delete (timer);
}
}
