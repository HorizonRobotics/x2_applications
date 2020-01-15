/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     the scheduler of xsoul framework
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef HOBOTXROC_SCHEDULER_H_
#define HOBOTXROC_SCHEDULER_H_

#include <atomic>
#include <set>
#include <string>
#include <unordered_map>
#include <map>
#include <vector>
#include "hobotxroc/framework_data.h"
#include "hobotxroc/node.h"
#include "hobotxroc/xroc_config.h"
#include "hobotxsdk/xroc_data.h"
#include "hobotxsdk/xroc_error.h"

namespace HobotXRoc {

struct SlotInfo {
  NodePtr prod_node_;
  std::set<NodePtr> dep_nodes_;
};

class Scheduler {  // 调度模块
 public:
  Scheduler() {
    // 默认只有一个源
    sequence_id_list_.push_back(std::make_shared<std::atomic_ullong>(0));
  }

  virtual ~Scheduler();

  int Init(XRocConfigPtr config);

  int UpdateConfig(std::string method_name, InputParamPtr param_ptr);

  InputParamPtr GetConfig(const std::string &method_name) const;

  int SetCallback(XRocCallback callback);

  int SetCallback(XRocCallback callback, const std::string &name);

  int SetFreeMemery(bool is_enable);

  int64_t Input(InputDataPtr data, void *sync_context);

  int Schedule(FrameworkDataPtr data, NodePtr readyNode);

  std::string GetVersion(const std::string &method_name) const;

  XThreadRawPtr GetCommNodeDaemon() const {
    return comm_node_daemon_.get();
  }

  ThreadManager* GetEngine() const {
    return engine_.get();
  }

 private:
  bool IsNodeReadyToDo(const FrameworkDataPtr &framework_data, NodePtr node);

  // 释放无效数据
  int FreeDataSlot(const FrameworkDataPtr &framework_data,
                              const std::vector<int> &slots);

  void PrepareNodeBeforeToDo(const FrameworkDataPtr &framework_data,
                             NodePtr node);

  // 用于同步接口判断当帧是否结束
  bool IsFrameDone(const FrameworkDataPtr &framework_data);

  // 用于异步接口判断单路output是否结束
  bool IsSingleOutputDone(const FrameworkDataPtr &framework_data,
                          std::string output_type);

  int Schedule4SlotImp2(FrameworkDataPtr framework_data,
                        const std::vector<int> &readySlots);

  int ScheduleImp2(FrameworkDataPtr framework_data, NodePtr readyNode);

  // 多路输出时，封装单路输出，用于异步调用
  OutputDataPtr SingleOutput(FrameworkDataPtr data, std::string output_type);

  // 多路整体输出，用于同步调用
  std::vector<OutputDataPtr> MultipleOutput(FrameworkDataPtr data);

  int OutputMethodResult(FrameworkDataPtr data, NodePtr readyNode);

  int CreateNodes();

  std::vector<int> CreateSlot(const std::vector<std::string> &datas);

  std::shared_ptr<ThreadManager> engine_{nullptr};

  std::unordered_map<std::string, NodePtr> name2ptr_;
  std::unordered_map<NodePtr, std::vector<int>> node_input_slots_;
  std::unordered_map<NodePtr, std::vector<int>> node_output_slots_;
  std::unordered_map<std::string, int> data_slots_;
  std::vector<std::string> data_slot_names_;
  // 支持多路输出
  std::map<std::string, std::vector<std::string>> output_type_names_;
  // xroc输出集合
  std::vector<std::string> xroc_output_names_;
  std::vector<SlotInfo> slot_infos_;

  // 是否需要释放帧内无用数据
  bool is_need_free_data_ = false;
  NodePtr input_node_;
  NodePtr output_node_;
  XRocCallback callback_;
  std::unordered_map<NodePtr, XRocCallback> node_callbacks_;
  SchedulerConfigPtr scheduler_config_;

  std::shared_ptr<XThread> comm_node_daemon_;
  std::shared_ptr<XThread> thread_;

  std::vector<std::shared_ptr<std::atomic_ullong>> sequence_id_list_;
  std::atomic_ullong global_sequence_id_;
  bool is_init_{false};
};

typedef std::shared_ptr<Scheduler> SchedulerPtr;

}  // namespace HobotXRoc

#endif  // HOBOTXROC_SCHEDULER_H_
