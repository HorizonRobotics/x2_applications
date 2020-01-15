/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     the scheduler of xroc framework
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */

#include "hobotxroc/scheduler.h"
#include <future>
#include <iterator>
#include <set>
#include <string>
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/method_factory.h"
#include "hobotxroc/profiler.h"
#include "hobotxsdk/xroc_error.h"
#include "hobotxroc/json_key.h"

namespace HobotXRoc {

int Scheduler::Init(XRocConfigPtr config) {
  if (is_init_) {
    LOGE << "Scheduler already Init";
    return -2;
  }
  LOGD << "Scheduler Init";
  scheduler_config_ = std::make_shared<SchedulerConfig>(config);

  int ret = scheduler_config_->CheckConfigValid();
  if (0 != ret) {
      return ret;
  }

  engine_.reset(ThreadManager::Instance());

  thread_.reset(engine_->CreateAutoThread(),
  [](XThreadRawPtr th) {
    th->Stop();
  });

  thread_->SetMaxTaskCount(scheduler_config_->GetMaxRunningCount());
  comm_node_daemon_.reset(engine_->CreateAutoThread(),
  [](XThreadRawPtr th) {
    th->Stop();
    });


  auto optional_config = scheduler_config_->GetOptionalConfig();
  if (!optional_config.isNull()) {
    if (optional_config.isMember(kSchedUpper)) {
      auto sched_upper = optional_config[kSchedUpper];
      auto policy = sched_upper[kPolicy].asString();
      auto priority = sched_upper[kPriority].asInt();
      comm_node_daemon_->SetPriority(policy, priority);
    }
    if (optional_config.isMember(kSchedDown)) {
      auto sched_upper = optional_config[kSchedDown];
      auto policy = sched_upper[kPolicy].asString();
      auto priority = sched_upper[kPriority].asInt();
      thread_->SetPriority(policy, priority);
    }
  }
  sequence_id_list_.clear();
  for (size_t i = 0; i < scheduler_config_->GetSourceNumber(); ++i)
    sequence_id_list_.push_back(std::make_shared<std::atomic_ullong>(0));

  if (0 != CreateNodes()) {
    LOGE << "CreateNodes failed";
    return -1;
  }

  is_init_ = true;
  return 0;
}

Scheduler::~Scheduler() {
  if (is_init_) {
    /*
   * 线程平滑退出方式：
   * 1.Stop所有Node的执行线程；
   * 2.删除公共线程池。
   */
  // thread_.reset();
  // comm_node_daemon_.reset();

    auto all_threads = engine_->GetThreads();
    for (auto thr : all_threads) {
      thr->Stop();
    }
  }
}

int Scheduler::SetCallback(XRocCallback callback) {
  callback_ = callback;
  return 0;
}

int Scheduler::SetCallback(XRocCallback callback, const std::string &name) {
  auto node_ptr_i = name2ptr_.find(name);
  if (node_ptr_i == name2ptr_.end()) {
    LOGE << "Failed to find " << name << " in the configs";
    return HOBOTXROC_ERROR_INVALID_PARAM;
  } else {
    if (callback) {
      LOGI << "Set callback for " << name;
      node_callbacks_[node_ptr_i->second] = callback;
    } else {
      LOGI << "Unset callback for " << name;
      node_callbacks_.erase(node_ptr_i->second);
    }
    return 0;
  }
}

int Scheduler::SetFreeMemery(bool is_enable) {
  is_need_free_data_ = is_enable;
  return 0;
}

int Scheduler::UpdateConfig(std::string method_name, InputParamPtr param_ptr) {
  auto iter = name2ptr_.find(method_name);
  if (iter != name2ptr_.end()) {
    return iter->second->SetParameter(param_ptr);
  }
  return 1;
}

InputParamPtr Scheduler::GetConfig(const std::string &method_name) const {
  auto iter = name2ptr_.find(method_name);
  if (iter != name2ptr_.end()) {
    return iter->second->GetParameter();
  }
  return nullptr;
}

std::string Scheduler::GetVersion(const std::string &method_name) const {
  auto iter = name2ptr_.find(method_name);
  if (iter != name2ptr_.end()) {
    return iter->second->GetVersion();
  }
  return std::string{};
}
/// 将用户输入数据转换成框架数据
int64_t Scheduler::Input(InputDataPtr input, void *sync_context) {
  RUN_FPS_PROFILER("workflow input")
  auto framework_data = std::make_shared<FrameworkData>();

  HOBOT_CHECK(input->source_id_ < scheduler_config_->GetSourceNumber())
      << "source id " << input->source_id_ << " is out of range (0-"
      << scheduler_config_->GetSourceNumber()- 1;

  framework_data->source_id_ = input->source_id_;
  framework_data->driven_nodes_nums_.resize(data_slots_.size(), 0);
  framework_data->datas_.resize(data_slots_.size());
  framework_data->datas_state_.resize(data_slots_.size());

  for (auto &state : framework_data->datas_state_) {
    state = DataState_None;
  }

  for (const auto &base_data : input->datas_) {
    auto itr = data_slots_.find(base_data->name_);
    HOBOT_CHECK(itr != data_slots_.end())
        << "failed to find " << base_data->name_ << " in the config";
    auto index = itr->second;
    framework_data->datas_[index] = base_data;
    framework_data->datas_state_[index] = DataState_Ready;
  }
  for (const auto &param : input->params_) {
    framework_data->method_param_[param->method_name_] = param;
  }

  framework_data->context_ = input->context_;
  framework_data->sync_context_ = sync_context;
  framework_data->sequence_id_ =
                    (*sequence_id_list_[framework_data->source_id_])++;
  framework_data->golbal_squence_id_ = global_sequence_id_++;
  framework_data->timestamp_ = framework_data->sequence_id_;

  int ret = Schedule(framework_data, nullptr);
  return (ret >= 0) ? framework_data->sequence_id_ : ret;
}

OutputDataPtr Scheduler::SingleOutput(FrameworkDataPtr framework_data,
                                      std::string output_type) {
  // RUN_FPS_PROFILER(output_type + " output")
  OutputDataPtr result(new OutputData());
  result->context_ = framework_data->context_;
  result->sequence_id_ = framework_data->sequence_id_;
  result->source_id_ = framework_data->source_id_;
  result->error_code_ = 0;
  result->output_type_ = output_type;
  for (const auto &name : output_type_names_[output_type]) {
    auto slot = data_slots_[name];
    auto &output_basedata = framework_data->datas_[slot];
    if (!output_basedata) {
      result->error_code_ += HOBOTXROC_ERROR_OUTPUT_NOT_READY;
      result->error_detail_ += (name + " is not ready;");
      continue;
    }

    output_basedata->name_ = name;
    result->datas_.push_back(output_basedata);
    result->error_code_ += output_basedata->error_code_;
    result->error_detail_ += output_basedata->error_detail_;
    result->global_sequence_id_ = framework_data->golbal_squence_id_;
  }
  return result;
}

std::vector<OutputDataPtr>
  Scheduler::MultipleOutput(FrameworkDataPtr framework_data) {
  // RUN_FPS_PROFILER("workflow output")
  std::vector<OutputDataPtr> multiple_result;
  for (auto single_output : output_type_names_) {
    std::string output_type = single_output.first;
    OutputDataPtr result(new OutputData());
    result->context_ = framework_data->context_;
    result->sequence_id_ = framework_data->sequence_id_;
    result->source_id_ = framework_data->source_id_;
    result->error_code_ = 0;
    result->output_type_ = output_type;
    // 封装单路输出
    for (const auto &name : output_type_names_[output_type]) {
      auto slot = data_slots_[name];
      auto &output_basedata = framework_data->datas_[slot];
      if (!output_basedata) {
        result->error_code_ += HOBOTXROC_ERROR_OUTPUT_NOT_READY;
        result->error_detail_ += (name + " is not ready;");
        continue;
      }
      output_basedata->name_ = name;
      result->datas_.push_back(output_basedata);
      result->error_code_ += output_basedata->error_code_;
      result->error_detail_ += output_basedata->error_detail_;
      result->global_sequence_id_ = framework_data->golbal_squence_id_;
    }
    multiple_result.push_back(result);
  }
  return multiple_result;
}

/// only support async call
int Scheduler::OutputMethodResult(FrameworkDataPtr framework_data,
                                  NodePtr readyNode) {
  RUN_FPS_PROFILER(readyNode->GetUniqueName() + "method output")
  if (framework_data->sync_context_ != nullptr) {
    return -1;
  } else {
    auto node_callback_i = node_callbacks_.find(readyNode);
    if (node_callbacks_.end() != node_callback_i) {
      auto &node_callback = node_callback_i->second;
      OutputDataPtr result(new OutputData());
      result->context_ = framework_data->context_;
      result->sequence_id_ = framework_data->sequence_id_;
      result->method_name_ = readyNode->GetUniqueName();
      result->error_code_ = 0;
      auto out_slots = node_output_slots_[readyNode];
      for (auto slot : out_slots) {
        auto &output_basedata = framework_data->datas_[slot];
        auto &name = data_slot_names_[slot];
        if (!output_basedata) {
          result->error_code_ += HOBOTXROC_ERROR_OUTPUT_NOT_READY;
          result->error_detail_ += (name + " is not ready;");
          continue;
        }
        output_basedata->name_ = name;
        result->datas_.push_back(output_basedata);
        result->error_code_ += output_basedata->error_code_;
        result->error_detail_ += output_basedata->error_detail_;
      }
      node_callback(result);
    }
  }
  return 0;
}

int Scheduler::Schedule(FrameworkDataPtr framework_data, NodePtr readyNode) {
  int ret = thread_->PostAsyncTask(
      readyNode ? readyNode->GetUniqueName() : NODE_UNINAME_RESERVED_INPUT,
      std::bind(&Scheduler::ScheduleImp2, this, framework_data, readyNode));
  if (ret < 0) {
    return HOBOTXROC_ERROR_EXCEED_MAX_RUNNING_COUNT;
  }
  return 0;
}

bool Scheduler::IsNodeReadyToDo(const FrameworkDataPtr &framework_data,
                                NodePtr node) {
  auto &in_slots = node_input_slots_[node];
  size_t ready_count = 0;
  for (const auto &slot : in_slots) {
    if (framework_data->datas_state_[slot] != DataState_Ready) {
      break;
    }
    ready_count++;
  }
  return in_slots.size() == ready_count;
}

int Scheduler::FreeDataSlot(const FrameworkDataPtr &framework_data,
                              const std::vector<int> &slots) {
  // 不需要释放帧内数据
  if (!is_need_free_data_) return 0;

  for (auto slot : slots) {
    // 0. 将slot编号转换为name
    std::string data_name = data_slot_names_[slot];

    // 1. 其驱动节点Node是否已被驱动
    int dep_nodes_num = slot_infos_[slot].dep_nodes_.size();
    if (dep_nodes_num > framework_data->driven_nodes_nums_[slot]) {
      continue;
    }

    // 2. 当前数据是否是输出数据
    if (std::find(xroc_output_names_.begin(),
                  xroc_output_names_.end(),
                  data_name) != xroc_output_names_.end()) {
      continue;
    }
    framework_data->datas_[slot] = nullptr;
  }
  return 0;
}

void Scheduler::PrepareNodeBeforeToDo(const FrameworkDataPtr &framework_data,
                                      NodePtr node) {
  auto &out_slots = node_output_slots_[node];
  for (const auto &slot : out_slots) {
    if (DataState_None != framework_data->datas_state_[slot])
      continue;
    /// change status from DataState_None to DataState_Doing
    framework_data->datas_state_[slot] = DataState_Doing;
  }
}

bool Scheduler::IsFrameDone(const FrameworkDataPtr &framework_data) {
  bool is_frame_ready = true;
  for (auto &o_name : xroc_output_names_) {
    auto data_state = framework_data->datas_state_[data_slots_[o_name]];
    if (DataState_Ready != data_state && DataState_Error != data_state) {
      is_frame_ready = false;
      break;
    }
  }
  return is_frame_ready;
}

bool Scheduler::IsSingleOutputDone(const FrameworkDataPtr &framework_data,
                                   std::string output_type) {
  // 判断该路输出是否存在
  auto itr = output_type_names_.find(output_type);
  HOBOT_CHECK(itr != output_type_names_.end())
      << "failed to find " << output_type << " in the config";

  bool is_output_ready = true;
  auto single_outputs = itr->second;
  for ( auto outputs_name : single_outputs ) {
    auto data_state = framework_data->datas_state_[data_slots_[outputs_name]];
    if ( DataState_Ready != data_state && DataState_Error != data_state ) {
      is_output_ready = false;
      break;
    }
  }
  return is_output_ready;
}

int Scheduler::Schedule4SlotImp2(FrameworkDataPtr framework_data,
                                 const std::vector<int> &readySlots) {
  // 如果 FrameworkDataState_Ready 那么此数据已经上报结果没必要再次上报
  if (FrameworkDataState_Ready == framework_data->state_) {
    LOGW << "FrameworkDataState_Ready twice";
    return 0;
  }
  // 1. 异步处理结果
  if (framework_data->sync_context_ == nullptr) {
    for (auto single_output : output_type_names_) {
      std::string output_type = single_output.first;
      // 1.1 未上报 && 已完成的单路输出
      if ((std::find(framework_data->output_type_isout_.begin(),
                      framework_data->output_type_isout_.end(),
                      output_type) ==
           framework_data->output_type_isout_.end()) &&
          IsSingleOutputDone(framework_data, output_type)) {
          callback_(SingleOutput(framework_data, output_type));  // 上报单路输出
          framework_data->output_type_isout_.push_back(output_type);
      }
    }
  }
  // 2. 判断当前帧是否结束
  if (IsFrameDone(framework_data)) {
    // 此帧结束，置帧数据状态为ready
    LOGD << "FrameworkDataState_Ready";
    framework_data->state_ = FrameworkDataState_Ready;
    // 2.1 同步处理结果
    if (framework_data->sync_context_ != nullptr) {
      auto promise = static_cast<std::promise<std::vector<OutputDataPtr>> *>(
                     framework_data->sync_context_);
      promise->set_value(MultipleOutput(framework_data));
    }
    return 0;
  }

  // 此帧数据还没有结束，准备驱动input已经ready的node继续工作
  std::set<NodePtr> prepare_nodes;
  for (auto rSlot : readySlots) {
    for (auto &node : slot_infos_[rSlot].dep_nodes_) {
      if (IsNodeReadyToDo(framework_data, node)) prepare_nodes.insert(node);
    }
  }
  for (auto &node : prepare_nodes) {
    PrepareNodeBeforeToDo(framework_data, node);
    node->Do(framework_data);
    for (auto inslot : node_input_slots_[node]) {
      framework_data->driven_nodes_nums_[inslot]++;
    }
    FreeDataSlot(framework_data, node_input_slots_[node]);
  }
  return 0;
}

int Scheduler::ScheduleImp2(FrameworkDataPtr framework_data,
                            NodePtr readyNode) {
  if (readyNode) {
    LOGD << "ScheduleImp2: " << readyNode->GetUniqueName();
    auto out_slots = node_output_slots_[readyNode];
    for (auto slot : out_slots) {
      framework_data->datas_state_[slot] = DataState_Ready;
    }
    OutputMethodResult(framework_data, readyNode);
    // Node输出内存资源释放
    FreeDataSlot(framework_data, node_output_slots_[readyNode]);

    return Schedule4SlotImp2(framework_data, out_slots);
  } else {
    // process for input, first schedule
    std::vector<int> slots;
    size_t slot_num = framework_data->datas_state_.size();
    for (size_t slot = 0; slot < slot_num; slot++) {
      if (framework_data->datas_state_[slot] == DataState_Ready)
        slots.push_back(slot);
    }
    return Schedule4SlotImp2(framework_data, slots);
  }
}

int Scheduler::CreateNodes() {
  for (const auto &nodeName : scheduler_config_->GetNodesName()) {
    if (name2ptr_.find(nodeName) != name2ptr_.end()) {
      LOGE << "node name'" << nodeName << " is not unique!";
      return -1;
    }
    auto node = std::make_shared<HobotXRoc::Node>(nodeName);
    auto inputs = scheduler_config_->GetNodeInputs(nodeName);
    auto outputs = scheduler_config_->GetNodeOutputs(nodeName);
    auto inputSlot = CreateSlot(inputs);
    auto outputSlot = CreateSlot(outputs);
    node_input_slots_[node] = inputSlot;
    node_output_slots_[node] = outputSlot;
    for (auto slot : inputSlot) {
      if (slot < 0) continue;
      if (slot_infos_.size() <= static_cast<size_t>(slot)) {
        slot_infos_.resize(slot + 1);
      }
      slot_infos_[slot].dep_nodes_.insert(node);
    }
    for (auto slot : outputSlot) {
      if (slot < 0) continue;
      if (slot_infos_.size() <= static_cast<size_t>(slot)) {
        slot_infos_.resize(slot + 1);
      }
      slot_infos_[slot].prod_node_ = node;
    }

    node->Init(std::bind(&Scheduler::Schedule, this, std::placeholders::_1,
                         std::placeholders::_2),
               inputSlot, outputSlot,
               scheduler_config_->GetNodeConfig(nodeName),
               std::make_shared<NodeRunContext>(
                 GetCommNodeDaemon(),
                 GetEngine(),
                 scheduler_config_->GetSharedConfg()));
    name2ptr_[nodeName] = node;
  }
  xroc_output_names_ = scheduler_config_->GetFlowOutputsUnion();
  output_type_names_ = scheduler_config_->GetFlowOutputs();
  return 0;
}

std::vector<int> Scheduler::CreateSlot(
    const std::vector<std::string> &dataNames) {
  LOGD << "CreateSlot";
  std::vector<int> dataSlots;
  int index = data_slots_.size();
  for (const auto &dataName : dataNames) {
    if (data_slots_.find(dataName) == data_slots_.end()) {
      data_slots_[dataName] = index++;
      data_slot_names_.push_back(dataName);
    }
    dataSlots.push_back(data_slots_[dataName]);
  }
  return dataSlots;
}

}  // namespace HobotXRoc

