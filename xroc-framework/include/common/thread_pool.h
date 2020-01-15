/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong,chuanyi.yang
 * @Mail: songshan.gong@horizon.ai
 * @Mail: chuanyi.yang@horizon.ai
 * @Date: 2019-10-09 19:55:46
 * @Version: v0.0.1
 * @Brief: xRoc exec engine declarition.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-11-13 05:19:25
 */

#ifndef COMMON_THREAD_POOL_H_
#define COMMON_THREAD_POOL_H_

#include <cstddef>
#include <cstdint>
#include <limits.h>

#include <functional>
#include <chrono>
#include <list>
#include <memory>
#include <mutex>
#include <atomic>
#include <vector>
#include <thread>
#include <string>
#include <unordered_map>
#include <condition_variable>
#include "hobotlog/hobotlog.hpp"

namespace HobotXRoc {

#if defined(HR_POSIX) && defined(HR_LINUX)
enum class LinuxThreadPriority {
  LINUX_SCHED_NORMAL,
  LINUX_SCHED_BATCH,
  LINUX_SCHED_FIFO,
  LINUX_SCHED_RR,
  LINUX_SCHED_IDLE
};
#endif

typedef std::function<void()> FunctionTask;
typedef std::function<void(void*)> WrapperFunctionTask;

struct Task {
  std::string post_from_;
  FunctionTask func_;

  explicit Task(const std::string &post_from,
    const FunctionTask &task) : post_from_(post_from), func_(task) {}
};

class XThread {
 public:
  explicit XThread(uint32_t thread_idx, int max_task_count = INT_MAX);

  virtual ~XThread();

  void SetMaxTaskCount(int max_task_count) {
    max_task_count_ = max_task_count;
  }

  int PostTimerTask(const std::string &post_from,
                    const FunctionTask &task,
                    std::chrono::milliseconds timeout);

  int PostAsyncTask(const std::string &post_from,
                    const FunctionTask &task);

  int Pause();

  int Resume();

  int Stop();

  // Clear task with key @post_from.
  void ClearSpecificTasks(
    const std::string &post_from,
    std::list<std::shared_ptr<Task>> *removed = nullptr);

  // Clear all tasks.
  void ClearTasks(std::list<std::shared_ptr<Task>> *removed = nullptr);

  bool SetAffinity(int core_id);

  bool SetPriority(const std::string &policy, int priority);

  uint32_t GetThreadIdx() const;

 private:
  XThread() = delete;
  void ExecLoop();
  uint32_t thread_idx_;
  std::shared_ptr<std::thread> thread_;

  int TaskCount() { return task_queue_.size();}

  typedef std::list<std::shared_ptr<Task> > TaskContainer;
  TaskContainer task_queue_;
  uint32_t max_task_count_;
  mutable std::mutex task_queue_mutex_;
  std::condition_variable condition_;

  std::atomic<bool> stop_ {false};
  std::atomic<int> pause_{0};
};
typedef XThread *XThreadRawPtr;

typedef std::function<int (const void*, const std::vector<void*>&)>
  ThreadPoolKeyMatchingFunc;

class XThreadPool {
 public:
  /**
   * @brief Construct a new Thread Pool object
   *
   * @param unique_name 线程池的唯一名字，
   *                    clear task时用于区分是否是该线程池发射的task
   * @param ths 需要添加进线程池的线程
   * @param prepares 每个线程执行正式任务之前需要执行的操作
   * @param contexts 每个线程对应的context信息。
   */
  explicit XThreadPool(const std::string &unique_name,
                      const std::vector<XThreadRawPtr> &ths,
                      const std::vector<WrapperFunctionTask> &prepares,
                      const std::vector<void*> &contexts);

  virtual ~XThreadPool();

  // 动态添加线程
  void AddOneThread(const XThreadRawPtr th,
                    const WrapperFunctionTask &prepare,
                    void *context);

  // 动态删除线程，只是从线程池中删掉，不释放线程资源。
  // 会自动调用ThreadStop，并把该线程剩余负载迁移到其他线程;
  // 如果当前线程池中只有一个线程，删除失败，返回NULL;
  XThreadRawPtr DelOneThread();

  enum class PostStrategy {
    ROUND_BIN = 0,
    KEY_MATCHING = 1,
    // TODO(songshan.gong): support other post strategy.
  };

  bool SetPostStrategy(PostStrategy stgy = PostStrategy::ROUND_BIN);

  bool SetKeyMatchingFunc(const ThreadPoolKeyMatchingFunc& key_matching_func) {
    if (key_matching_func == nullptr)
      return false;
    find_keymatching_index_ = key_matching_func;
    return true;
  }

  int PostTimerTask(const WrapperFunctionTask &task,
                    std::chrono::milliseconds timeout,
                    const void* key = nullptr);

  int PostAsyncTask(const WrapperFunctionTask &task, const void* key = nullptr);

  int Pause();

  int Resume();

  int Stop();

  void ClearTasks();

  bool SetAffinity(int core_id);

  bool SetPriority(const std::string &policy, int priority);

  std::vector<uint32_t> GetThreadIdx() const;

  std::vector<XThreadRawPtr> GetThreads() const;

 private:
  XThreadPool() = delete;
  int PostAsyncTaskInternal(const FunctionTask &task,
                            const void* key = nullptr);

  int DefaultKeyMatching(const void *key, const std::vector<void*>& contexts) {
    return -1;
  }

  int GetSelectThreadIdx(const void* key, int* select_th_idx);


  std::vector<XThreadRawPtr> threads_;
  mutable std::mutex thread_mutex_;
  std::string unique_name_;

  std::vector<WrapperFunctionTask> prepares_;
  std::vector<void*> contexts_;
  ThreadPoolKeyMatchingFunc find_keymatching_index_;

  PostStrategy stgy_{PostStrategy::ROUND_BIN};
  uint32_t cur_post_pos_{0};

  std::atomic<bool> stop_ {false};
  std::atomic<bool> pause_{false};
};

typedef std::shared_ptr<XThreadPool> XThreadPoolPtr;

// user thread idx: 0~999;
// reserved thread_idx: 1000 ~1099
// auto-produce thread_idx:1100~INT_MAX
#define XROC_SCHEDULE_THREAD_UPPER_IDX 1000
#define XROC_SCHEDULE_THREAD_DOWN_IDX 1001
#define XROC_AUTO_PRODUCE_THREAD_IDX_BASE 1100

// 按需分配线程资源，并通过数字索引分配的线程;
class ThreadManager {
 public:
  static ThreadManager* Instance() {
    return new ThreadManager();
  }
  ThreadManager() = default;
  ~ThreadManager();

  XThreadRawPtr CreateThread(uint32_t thread_idx);

  XThreadRawPtr CreateAutoThread() {
    std::lock_guard<std::mutex> lck(thread_mutex_);
    uint32_t thread_idx = XROC_AUTO_PRODUCE_THREAD_IDX_BASE + auto_thread_cnt_;
    auto_thread_cnt_++;
    HOBOT_CHECK(threads_.find(thread_idx) == threads_.end())
    << "Anyone set auto-produce thread idx:" << thread_idx;
    threads_[thread_idx] = new XThread(thread_idx);
    return threads_[thread_idx];
  }

  // 每个线程一个context，每个线程一个唯一索引idx
  std::vector<XThreadRawPtr>
  CreateThreads(std::vector<uint32_t> thread_idxes);

  // 根据thread_idx获取线程，如果尚未分配则返回NULL;
  XThreadRawPtr GetThread(uint32_t thread_idx);

  std::vector<XThreadRawPtr> GetThreads() const {
    std::vector<XThreadRawPtr> ret;
    for (auto it = threads_.begin(); it != threads_.end(); ++it) {
      ret.push_back(it->second);
    }
    return ret;
  }

  // 删除对应线程，并释放资源，要保证调用这个之前，先调用对应线程的Stop操作。
  bool DelThread(uint32_t thread_idx);

 private:
  ThreadManager(const ThreadManager &other) = delete;
  ThreadManager(ThreadManager &&other) = delete;

  std::unordered_map<uint32_t, XThreadRawPtr> threads_;
  mutable std::mutex thread_mutex_;

  int auto_thread_cnt_{0};
};

}  // namespace HobotXRoc

#endif  // COMMON_THREAD_POOL_H_
