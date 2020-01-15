/**
* Copyright (c) 2019 Horizon Robotics. All rights reserved.
* @file profiler.h
* @brief 
* @author ruoting.ding
* @email ruoting.ding@horizon.ai
* @date 2019/4/15
*/

#ifndef HOBOTXROC_PROFILER_H_
#define HOBOTXROC_PROFILER_H_

#include <string>
#include <memory>
#include <vector>
#include <fstream>
/**
 * \brief base class for scope log
 */
class ProfilerScope {
 public:
  ProfilerScope() = default;
  virtual ~ProfilerScope() {}
 protected:
  std::string name_;
  std::string tag_;
};
/**
 * \brief a listener ,which will be notified when global profiler status is changed.
 */
class Profiler;
class ProfilerListener {
 public:
  ProfilerListener();
  ~ProfilerListener();
  /// will be called by profiler
  virtual void OnProfilerChanged(bool on) {}

 private:
  std::shared_ptr<Profiler> profiler;
};
/**
 * \brief base class for profiler collector
 */
class ProfilerCollector : public ProfilerListener {
 public:
  enum class Type {
    kFps,
    kProcessTime
  };
  static std::shared_ptr<ProfilerCollector> Create(ProfilerCollector::Type type);
  virtual std::unique_ptr<ProfilerScope> CreateScope(
      const std::string &name,
      const std::string &tag) = 0;
 protected:
  ProfilerCollector() = default;
};

class Profiler {
 public:
  /**
   * \brief status of the profiler
   */
  enum class State {
    kNotRunning = 0,
    kRunning
  };

  ~Profiler();
  /**
 * \brief get singleton
 * @return
 */
  static std::shared_ptr<Profiler> Get();
  /**
   * \brief check the status of profiler
   * @return
   */
  inline bool IsRunning() const {
    return state_ == State::kRunning;
  }

  void Log(const std::stringstream &ss);

  void OnRegister(ProfilerListener *listener);
  void UnRegister(ProfilerListener *listener);

  inline void Start() {
    SetState(State::kRunning);
  }
  inline void Stop() {
    SetState(State::kNotRunning);
  }
/**
 * \brief set profiler file , otherwise will use default log file.
 * \note the file could only be set once
 * @param file
 * @return success (true) or failure (false)
 */
  bool SetOutputFile(const std::string &file);

  void SetFrameIntervalForTimeStat(int cycle_num);
  void SetTimeIntervalForFPSStat(int cycle_ms);
 private:
  Profiler() = default;
  Profiler(const Profiler &) = delete;
  void SetState(Profiler::State state);
  static std::shared_ptr<Profiler> instance_;
  State state_ = State::kNotRunning;
  std::vector<ProfilerListener *> listeners_;
  std::fstream foi_;
};

#define RUN_PROCESS_TIME_PROFILER_WITH_TAG(name, tag) \
  static std::shared_ptr<ProfilerCollector> proc_time_collector \
  = ProfilerCollector::Create(ProfilerCollector::Type::kProcessTime);\
  std::unique_ptr<ProfilerScope> proc_time_scope; \
  if (Profiler::Get()->IsRunning()) { \
    proc_time_scope = proc_time_collector->CreateScope(name, tag); \
  } \


#define RUN_FPS_PROFILER_WIGH_TAG(name, tag) \
  static std::shared_ptr<ProfilerCollector> fps_collector \
  = ProfilerCollector::Create(ProfilerCollector::Type::kFps);\
  std::unique_ptr<ProfilerScope> fps_scope; \
  if (Profiler::Get()->IsRunning()) { \
    fps_scope = fps_collector->CreateScope(name, tag); \
  } \


#define RUN_FPS_PROFILER(name) RUN_FPS_PROFILER_WIGH_TAG(name, "FPS")

#define RUN_PROCESS_TIME_PROFILER(name) \
RUN_PROCESS_TIME_PROFILER_WITH_TAG(name, "TIME")

#endif //HOBOTXROC_PROFILER_H_


