/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     MethodManager interface of xsoul framework
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef HOBOTXROC_METHOD_MANAGER_H_
#define HOBOTXROC_METHOD_MANAGER_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include "common/rw_mutex.h"
#include "common/thread_pool.h"
#include "hobotxroc/method.h"
#include "hobotxroc/xroc_config.h"
#include "hobotxsdk/xroc_data.h"
#include "hobotxroc/profiler.h"

namespace HobotXRoc {

enum class MethodManagerContextState { NONE, INITIALIZED, FINALIZED };

struct MethodManagerContext {
  std::unordered_map<int32_t, MethodPtr> method_list_;
  MethodManagerContextState state_ = MethodManagerContextState::NONE;
};

typedef std::shared_ptr<MethodManagerContext> MethodManagerContextPtr;

class MethodManager : public ProfilerListener {
 public:
  using ResultCallback =
  std::function<void(const std::vector<std::vector<BaseDataPtr>> &)>;

  MethodManager() = default;

  virtual ~MethodManager();

  void Init(const Config &config,
            const XRocSharedConfig &shared_config,
            ThreadManager *engine);

  int UpdateParameter(InputParamPtr ptr);

  InputParamPtr GetParameter() const;

  std::string GetVersion() const;

  bool IsNeedReorder();

  int ProcessAsyncTask(const std::vector<std::vector<BaseDataPtr>> &inputs,
                       const std::vector<InputParamPtr> &params,
                       ResultCallback methodCallback,
                       size_t source_id);

  std::string MethodType() const { return method_type_; }

  std::string MethodName() const { return method_name_; }

  void OnProfilerChanged(bool on) override;

  XRocSharedConfig GetSharedConfig() const { return shared_config_; }
 private:
  /// methods 的实例列表，当is_src_ctx_dept == true，methods的个数为sourcenumber
  /// 此时的method里的index 对应source id
  std::vector<MethodPtr> methods_;
  std::vector<MethodManagerContextPtr> contexts_;

  Config config_;
  // configuration shared within all nodes
  XRocSharedConfig shared_config_;

  std::string method_type_;
  std::string method_name_;

  XThreadPoolPtr thread_pool_{nullptr};
  std::vector<XThreadRawPtr> threads_;

  void WaitUntilInit();

  uint32_t GenMethodKey(const std::vector<std::vector<BaseDataPtr>> &inputs,
    const std::vector<InputParamPtr> &params,
    size_t source_id);

  void InitMethod(void *context);

  void Process(const std::vector<std::vector<BaseDataPtr>> &inputs,
               const std::vector<InputParamPtr> &params,
               ResultCallback methodCallback,
               uint32_t source_id, void *context);

  RWLock lock_;
};

typedef std::shared_ptr<MethodManager> MethodManagerPtr;

}  // namespace HobotXRoc

#endif  // HOBOTXROC_METHOD_MANAGER_H_
