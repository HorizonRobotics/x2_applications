/**
* Copyright (c) 2019 Horizon Robotics. All rights reserved.
* @file profiler.cpp
* @brief 
* @author ruoting.ding
* @email ruoting.ding@horizon.ai
* @date 2019/4/15
*/


#include "hobotxroc/profiler.h"
#include <mutex>
#include <sstream>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <atomic>
#include <chrono>
#include "hobotlog/hobotlog.hpp"

namespace {

inline std::int64_t getMilliSecond() {
  auto time_now = std::chrono::system_clock::now();
  auto duration_in_ms =
      std::chrono::duration_cast
          <std::chrono::milliseconds>(time_now.time_since_epoch());
  return duration_in_ms.count();
}

struct FpsStatisticInfo {
  /// the interval of Statistical result output
  static int cycle_ms;
  /// FPS start time
  std::atomic_int_fast64_t pre_time;
  /// current total count
  std::atomic_int_fast64_t cnt;

  FpsStatisticInfo() {
    pre_time = 0;
    cnt = 0;
  }
};

int FpsStatisticInfo::cycle_ms = 3000;

class ScopeFPS : public ProfilerScope {
 public:
  ScopeFPS(const std::string &name,
           std::shared_ptr<FpsStatisticInfo> stat,
           const std::string &tag) {
    stat_ = stat;
    name_ = name;
    tag_ = tag;
    if (stat_->pre_time == 0) {
      stat_->pre_time = getMilliSecond();
    }
  }
  virtual ~ScopeFPS() {
    int64_t curr_time = getMilliSecond();
    stat_->cnt++;
    if (curr_time - stat_->pre_time > stat_->cycle_ms) {
      std::stringstream ss;

      ss << "[" + tag_ + "] [" << name_
         << "] fps : "
         << stat_->cnt / ((curr_time - stat_->pre_time) / 1000.0)
         << "\n";
      Profiler::Get()->Log(ss);
      stat_->pre_time = curr_time;
      stat_->cnt = 0;
    }
  }
 private:
  std::shared_ptr<FpsStatisticInfo> stat_;
};

class FpsProfilerListener : public ProfilerCollector {
 public:
  FpsProfilerListener() = default;
  void OnProfilerChanged(bool on) override {
    for (auto &stat : stats_) {
      stat.second->pre_time = 0;
      stat.second->cnt = 0;
    }
  }
  std::unique_ptr<ProfilerScope> CreateScope(
      const std::string &name,
      const std::string &tag) override {
    if (stats_.find(name) == stats_.end()) {
      stats_[name] = std::make_shared<FpsStatisticInfo>();
    }
    std::unique_ptr<ProfilerScope> scope;
    scope.reset(new ScopeFPS(name, stats_[name], tag));
    return scope;
  }
 private:
  std::unordered_map<std::string, std::shared_ptr<FpsStatisticInfo>> stats_;
};

struct ProcessTimeStatisticInfo {
  /// the interval of Statistical result output
  static int cycle_num;
  /// total process time
  std::atomic_int_fast64_t sum_time;
  /// current total count
  std::atomic_int_fast64_t cnt;
  /// min_process_time
  std::atomic_int_fast64_t min_time;
  /// max_process_time
  std::atomic_int_fast64_t max_time;

  ProcessTimeStatisticInfo() {
    sum_time = 0;
    cnt = 0;
    min_time = -1;
    max_time = -1;
  }
};

int ProcessTimeStatisticInfo::cycle_num = 10;

class ScopeProcessTime : public ProfilerScope {
 public:
  ScopeProcessTime(const std::string &name,
                   std::shared_ptr<ProcessTimeStatisticInfo> stat,
                   const std::string &tag) {
    begin_ = getMilliSecond();
    stat_ = stat;
    name_ = name;
    tag_ = tag;
  }
  virtual ~ScopeProcessTime() {
    auto cur_proc_time = getMilliSecond() - begin_;
    stat_->sum_time += cur_proc_time;
    if (-1 == stat_->min_time || cur_proc_time < stat_->min_time) {
      stat_->min_time = cur_proc_time;
    }
    if (cur_proc_time > stat_->max_time) {
      stat_->max_time = cur_proc_time;
    }
    if (++stat_->cnt >= stat_->cycle_num) {
      std::stringstream ss;
      ss << "[" + tag_ + "] [" << name_ << "] average :  "
         << static_cast<float>(stat_->sum_time) / stat_->cnt
         << " (ms), min : " << stat_->min_time
         << " (ms), max : " << stat_->max_time
         << " (ms)" << "\n";
      Profiler::Get()->Log(ss);
      stat_->cnt = 0;
      stat_->sum_time = 0;
      stat_->min_time = -1;
      stat_->max_time = -1;
    }
  }
 private:
  int64_t begin_ = 0;
  std::shared_ptr<ProcessTimeStatisticInfo> stat_;
};

class ProcessTimeListener : public ProfilerCollector {
 public:
  ProcessTimeListener() = default;
  void OnProfilerChanged(bool on) override {
    for (auto &stat : stats_) {
      stat.second->sum_time = 0;
      stat.second->cnt = 0;
      stat.second->max_time = -1;
      stat.second->min_time = -1;
    }
  }
  std::unique_ptr<ProfilerScope> CreateScope(
      const std::string &name,
      const std::string &tag) override {
    if (stats_.find(name) == stats_.end()) {
      stats_[name] = std::make_shared<ProcessTimeStatisticInfo>();
    }
    std::unique_ptr<ProfilerScope> scope;
    scope.reset(new ScopeProcessTime(name, stats_[name], tag));
    return scope;
  }
 private:
  std::unordered_map<std::string, std::shared_ptr<ProcessTimeStatisticInfo>>
      stats_;
};

} // end of namespace



ProfilerListener::ProfilerListener() {
  profiler = Profiler::Get();
  profiler->OnRegister(this);
}
ProfilerListener::~ProfilerListener() {
  profiler->UnRegister(this);
}

std::shared_ptr<ProfilerCollector> ProfilerCollector::Create(ProfilerCollector::Type type) {
  switch (type) {
    case Type::kFps: {
      return std::shared_ptr<ProfilerCollector>(new FpsProfilerListener());
    }
    case Type::kProcessTime: {
      return std::shared_ptr<ProfilerCollector>(new ProcessTimeListener());
    }
  }
  HOBOT_CHECK(false) << "Error type ";
  return nullptr;
}

std::shared_ptr<Profiler> Profiler::instance_;

/// used to guarantee the Profiler::instance_ is created only once
std::once_flag create_profiler_flag;

std::shared_ptr<Profiler> Profiler::Get() {
  if (!instance_) {
    std::call_once(create_profiler_flag,
                   []() {
                     instance_ = std::shared_ptr<Profiler>(new Profiler());
                   });
  }
  return instance_;
}

Profiler::~Profiler() {
  if (foi_.is_open()) {
    foi_.close();
  }
}

void Profiler::Log(const std::stringstream &ss) {
  if (foi_.is_open()) {
    foi_ << ss.str();
  } else {
    std::cout << ss.str() << std::endl;
  }
}

void Profiler::OnRegister(ProfilerListener *listener) {
  if (listener) {
    listeners_.emplace_back(listener);
    listener->OnProfilerChanged(IsRunning());
  }
}

void Profiler::UnRegister(ProfilerListener *listener) {
  if (listener) {
    for (uint i = 0; i < listeners_.size(); ++i) {
      if (listeners_[i] == listener) {
        listeners_.erase(listeners_.begin() + i);
        return;
      }
    }
  }
}

void Profiler::SetState(Profiler::State state) {
  if (state != state_) {
    state_ = state;
    for (const auto &listener:listeners_) {
      listener->OnProfilerChanged(IsRunning());
    }
  }
}

bool Profiler::SetOutputFile(const std::string &file) {
  if (foi_.is_open()) {
    foi_.close();
  }
  foi_.open(file, std::fstream::out | std::fstream::binary);
  if (foi_.fail()) {
    LOGE << "Failed to open " << file;
    return false;
  }
  return true;
}

void Profiler::SetFrameIntervalForTimeStat(int cycle_num) {
  HOBOT_CHECK_GE(cycle_num, 1);
  ProcessTimeStatisticInfo::cycle_num = cycle_num;
  LOGI << "SetFrameIntervalForTimeStat to " << cycle_num;
}

void Profiler::SetTimeIntervalForFPSStat(int cycle_ms) {
  HOBOT_CHECK_GE(cycle_ms, 1);
  FpsStatisticInfo::cycle_ms = cycle_ms;
  LOGI << "SetTimeIntervalForFPSStat to " << cycle_ms;
}
