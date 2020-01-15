/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong, chuanyi.yang
 * @Mail: songshan.gong@horizon.ai
 * @Mail: chuanyi.yang@horizon.ai
 * @Date: 2019-10-10 05:34:51
 * @Version: v0.0.1
 * @Brief: xRoc engine implementation.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-11-13 18:27:39
 */

#if defined(HR_POSIX)
#include <pthread.h>
#endif
#include <cstring>
#include <unordered_map>
#include "common/thread_pool.h"
#include "hobotlog/hobotlog.hpp"

namespace HobotXRoc {

#if defined(HR_LINUX) && defined(HR_POSIX)
static std::unordered_map<std::string, LinuxThreadPriority>
  g_str2priority = {
    {"SCHED_NORMAL", LinuxThreadPriority::LINUX_SCHED_NORMAL},
    {"SCHED_OTHER", LinuxThreadPriority::LINUX_SCHED_NORMAL},
    {"SCHED_BATCH", LinuxThreadPriority::LINUX_SCHED_BATCH},
    {"SCHED_RR", LinuxThreadPriority::LINUX_SCHED_RR},
    {"SCHED_FIFO", LinuxThreadPriority::LINUX_SCHED_FIFO},
    {"SCHED_IDLE", LinuxThreadPriority::LINUX_SCHED_IDLE}
  };
#endif

XThread::XThread(uint32_t thread_idx, int max_task_count) {
  thread_idx_ = thread_idx;
  max_task_count_ = max_task_count;

  thread_ = std::shared_ptr<std::thread>(
    new std::thread(&XThread::ExecLoop, this));
}

XThread::~XThread() {
  Stop();
}

int XThread::PostTimerTask(const std::string &post_from,
                           const FunctionTask &task,
                           std::chrono::milliseconds timeout) {
  // not implement
  return 0;
}

int XThread::PostAsyncTask(const std::string &post_from,
                           const FunctionTask &functask) {
  if (stop_) return 0;
  {
    std::lock_guard<std::mutex> lck(task_queue_mutex_);
    if (task_queue_.size() >= max_task_count_) {
      return -1;
    }
    auto task = std::shared_ptr<Task>(new Task(post_from, functask));
    task_queue_.push_back(task);
    condition_.notify_one();
  }
  return 0;
}

int XThread::Pause() {
  pause_++;
  return 0;
}

int XThread::Resume() {
  pause_--;
  condition_.notify_one();
  return 0;
}

int XThread::Stop() {
  if (!stop_) {
  {
    std::lock_guard<std::mutex> lck(task_queue_mutex_);
    stop_ = true;
    condition_.notify_one();
  }
    if (thread_) {
      thread_->join();
    }
  }
  return 0;
}

void XThread::ClearSpecificTasks(
  const std::string &post_from,
  std::list<std::shared_ptr<Task>> *removed) {
  std::lock_guard<std::mutex> lck(task_queue_mutex_);
  TaskContainer target_list;
  for (auto it = task_queue_.begin(); it != task_queue_.end();) {
    if ((*it)->post_from_ == post_from) {
      target_list.push_back(*it);
      it = task_queue_.erase(it);
    } else {
      it++;
    }
  }
  if (removed) {
    *removed = target_list;
  }
}

void XThread::ClearTasks(std::list<std::shared_ptr<Task>> *removed) {
  std::lock_guard<std::mutex> lck(task_queue_mutex_);
  TaskContainer target_list;
  task_queue_.swap(target_list);
  if (removed) {
    *removed = target_list;
  }
}

bool XThread::SetAffinity(int core_id) {
  // not implement
  return true;
}

bool XThread::SetPriority(const std::string &policy, int priority) {
#if defined(HR_POSIX) && defined(HR_LINUX)
  if (g_str2priority.find(policy) == g_str2priority.end()) {
    return false;
  }

  sched_param sch;
  int old_policy;
  pthread_getschedparam(thread_->native_handle(), &old_policy, &sch);
  auto enum_policy = g_str2priority.find(policy)->second;
  switch (enum_policy) {
    case LinuxThreadPriority::LINUX_SCHED_NORMAL: {
      LOGW << policy << " cannot set priority";
      if (pthread_setschedparam(thread_->native_handle(), SCHED_OTHER, &sch)) {
        LOGE << "Failed to setschedparam: " << std::strerror(errno);
      }
      break;
    }
    case LinuxThreadPriority::LINUX_SCHED_BATCH: {
      LOGW << policy << " cannot set priority";
      if (pthread_setschedparam(thread_->native_handle(), SCHED_BATCH, &sch)) {
        LOGE << "Failed to setschedparam: " << std::strerror(errno);
      }
      break;
    }
    case LinuxThreadPriority::LINUX_SCHED_IDLE: {
      LOGW << policy << " cannot set priority";
      if (pthread_setschedparam(thread_->native_handle(), SCHED_IDLE, &sch)) {
        LOGE << "Failed to setschedparam: " << std::strerror(errno);
      }
      break;
    }
    case LinuxThreadPriority::LINUX_SCHED_FIFO: {
      sch.sched_priority = priority;
      if (pthread_setschedparam(thread_->native_handle(), SCHED_FIFO, &sch)) {
        LOGE << "Failed to setschedparam: " << std::strerror(errno);
      }
      break;
    }
    case LinuxThreadPriority::LINUX_SCHED_RR: {
      sch.sched_priority = priority;
      if (pthread_setschedparam(thread_->native_handle(), SCHED_RR, &sch)) {
        LOGE << "Failed to setschedparam: " << std::strerror(errno);
      }
      break;
    }
    default: {
      LOGW << "NOT support sched policy:" << policy;
      break;
    }
  }

#endif
  return true;
}

uint32_t XThread::GetThreadIdx() const {
  return thread_idx_;
}

void XThread::ExecLoop() {
  do {
    std::shared_ptr<Task> tsk;
    {
      std::unique_lock<std::mutex> lck(task_queue_mutex_);
      condition_.wait(lck, [&] (){
        return (task_queue_.size() > 0 && !pause_) ||
                stop_;
      });
      if (stop_) {
        return;
      }

      tsk = task_queue_.front();
      task_queue_.pop_front();
    }
    tsk->func_();
  } while (!stop_);
}

XThreadPool::XThreadPool(const std::string &unique_name,
                         const std::vector<XThreadRawPtr> &ths,
                         const std::vector<WrapperFunctionTask> &prepares,
                         const std::vector<void*> &contexts) {
  unique_name_ = unique_name;
  threads_ = ths;
  prepares_ = prepares;
  contexts_ = contexts;
  find_keymatching_index_ = std::bind(&XThreadPool::DefaultKeyMatching,
                              this,
                              std::placeholders::_1,
                              std::placeholders::_2);

  HOBOT_CHECK((ths.size() > 0) &&
         (ths.size() == prepares.size()) &&
         (ths.size() == contexts.size()));
  // cast prepares functions firstly.
  for (size_t i = 0; i < threads_.size(); i++) {
    if (prepares[i]) {
     auto task = std::bind(prepares[i], contexts[i]);
     threads_[i]->PostAsyncTask(unique_name_, task);
    }
  }
}

XThreadPool::~XThreadPool() {
  Stop();
}

void XThreadPool::AddOneThread(const XThreadRawPtr th,
                               const WrapperFunctionTask& prepare,
                               void *context) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  threads_.push_back(th);
  prepares_.push_back(prepare);
  contexts_.push_back(context);

  if (prepare) {
    th->PostAsyncTask(unique_name_, std::bind(prepare, context));
  }
}

int XThreadPool::GetSelectThreadIdx(const void* key, int* select_th_idx) {
  switch (stgy_) {
    case PostStrategy::ROUND_BIN: {
      if (cur_post_pos_ >= threads_.size()) {
        cur_post_pos_ = 0;
      }
      *select_th_idx = cur_post_pos_++;
      break;
    }
    case PostStrategy::KEY_MATCHING : {
      if (key == nullptr) {
        // return error, can not post the task since no key is inputed
        LOGE << "ThreadPool: key is Null in KEY_MATCHING mode ";
        return -1;
      }
      *select_th_idx = find_keymatching_index_(key, contexts_);
      if (*select_th_idx < 0) {
        LOGE << "ThreadPool: Can not found the key matching thread";
        return -1;
      }
      break;
    }
    default: {
      // not support other post strategy currently.
      HOBOT_CHECK(false) << "Doesn't support this "<< (uint16_t)stgy_
        << " post strategy currently";
    }
  }
  return 0;
}

int XThreadPool::PostAsyncTaskInternal(const FunctionTask &task,
                                        const void* key) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (stop_) return 0;
  int selected_thr_idx = 0;

  if (GetSelectThreadIdx(key, &selected_thr_idx) < 0) {
    return -1;
  }

  threads_[selected_thr_idx]->PostAsyncTask(unique_name_, task);
  return 0;
}



XThreadRawPtr XThreadPool::DelOneThread() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (threads_.size() <= 1) return nullptr;
  auto rm_thr = threads_.back();
  threads_.pop_back();
  contexts_.pop_back();
  prepares_.pop_back();
  std::list<std::shared_ptr<Task>> rm_list;
  rm_thr->ClearSpecificTasks(unique_name_, &rm_list);
  for (auto task : rm_list) {
    PostAsyncTaskInternal(task->func_);
  }
  return rm_thr;
}

bool XThreadPool::SetPostStrategy(XThreadPool::PostStrategy stgy) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  stgy_ = stgy;

  return true;
}

int XThreadPool::PostTimerTask(const WrapperFunctionTask &task,
                               std::chrono::milliseconds timeout,
                               const void* key) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (stop_) return 0;
  int selected_thr_idx = 0;

  if (GetSelectThreadIdx(key, &selected_thr_idx) < 0) {
    return -1;
  }

  threads_[selected_thr_idx]->PostTimerTask(
    unique_name_,
    std::bind(task, contexts_[selected_thr_idx]),
    timeout);
  return 0;
}

int XThreadPool::PostAsyncTask(const WrapperFunctionTask &task,
                               const void* key) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (stop_) return 0;
  int selected_thr_idx = 0;
  if (GetSelectThreadIdx(key, &selected_thr_idx) < 0) {
    return -1;
  }
  threads_[selected_thr_idx]->PostAsyncTask(
    unique_name_,
    std::bind(task, contexts_[selected_thr_idx]));
  return 0;
}

int XThreadPool::Pause() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  pause_ = true;

  return 0;
}

int XThreadPool::Resume() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  pause_ = false;
  return 0;
}

int XThreadPool::Stop() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (!stop_) {
    stop_ = true;
    for (auto thr : threads_) {
#if 1
      thr->Pause();
      thr->ClearSpecificTasks(unique_name_);
      thr->Resume();
#else
      // 假设所有node同时停止。
      thr->Stop();
#endif
    }
  }
  return 0;
}

void XThreadPool::ClearTasks() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto thr : threads_) {
    thr->Pause();
    thr->ClearSpecificTasks(unique_name_);
    thr->Resume();
  }
}

bool XThreadPool::SetAffinity(int core_id) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto thr : threads_) {
    thr->SetAffinity(core_id);
  }
  return true;
}

bool XThreadPool::SetPriority(const std::string &policy, int priority) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto thr : threads_) {
    thr->SetPriority(policy, priority);
  }
  return true;
}

std::vector<uint32_t> XThreadPool::GetThreadIdx() const {
  std::vector<uint32_t> ret_vec;
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto thr : threads_) {
    ret_vec.push_back(thr->GetThreadIdx());
  }
  return ret_vec;
}

std::vector<XThreadRawPtr> XThreadPool::GetThreads() const {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  return threads_;
}

ThreadManager::~ThreadManager() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto it = threads_.begin(); it != threads_.end(); it++) {
    auto pthr = it->second;
    delete pthr;
  }
  threads_.clear();
}

XThreadRawPtr ThreadManager::CreateThread(uint32_t thread_idx) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (threads_.find(thread_idx) == threads_.end()) {
    threads_[thread_idx] = new XThread(thread_idx);
  }
  return threads_[thread_idx];
}

std::vector<XThreadRawPtr>
ThreadManager::CreateThreads(std::vector<uint32_t> thread_idxes) {
  std::vector<XThreadRawPtr> ret_threads;
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto thread_idx : thread_idxes) {
    if (threads_.find(thread_idx) == threads_.end()) {
      threads_[thread_idx] = new XThread(thread_idx);
    }
    ret_threads.push_back(threads_[thread_idx]);
  }

  return ret_threads;
}

XThreadRawPtr ThreadManager::GetThread(uint32_t thread_idx) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (threads_.find(thread_idx) != threads_.end()) {
    return threads_[thread_idx];
  }
  return nullptr;
}

bool ThreadManager::DelThread(uint32_t thread_idx) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (threads_.find(thread_idx) != threads_.end()) {
    auto thread = threads_[thread_idx];
    threads_.erase(thread_idx);
    delete thread;
  }
  return true;
}
}  // namespace HobotXRoc
