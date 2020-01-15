/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Node in xsoul framework
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#include "hobotxroc/node.h"

#include <cassert>
#include <memory>
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/json_key.h"
#include "hobotxroc/profiler.h"
#include "hobotxroc/xroc_config.h"
#include "hobotxsdk/xroc_error.h"
#include "timer/timer.h"

namespace HobotXRoc {

struct ExtraStateInfoWithinNode : public ExtraInfoWithinNode {
  void *timer_token_ = nullptr;
  bool done_ = false;
};

void Node::Init(
    std::function<int(FrameworkDataPtr data, NodePtr readyNode)> callback,
    const std::vector<int> &inputSlots, const std::vector<int> &outputSlots,
    const Config &config,
    std::shared_ptr<NodeRunContext> run_context) {
  LOGD << "Node Init";
  input_slots_ = inputSlots;
  // 需要的每个output slot在总的data slot里的index
  output_slots_ = outputSlots;
  on_ready_ = callback;
  // TODO(jet) 设置node参数
  // 初始化method_manager
  method_manager_.Init(config,
    run_context->GetSharedConfig(),
      run_context->GetEngine());

  is_need_reorder_ = method_manager_.IsNeedReorder();
  daemon_thread_ = run_context->GetNodeDaemon();
  unique_name_ = config[kMethodName].asString();
  if (is_need_reorder_) {
    // construct sponge list when node need reorder
    sponge_list_.resize(run_context->GetSharedConfig().source_num_);
  }
}

void Node::OnGetResult(FrameworkDataShellPtr result) {
  auto shared_user_data = std::static_pointer_cast<ExtraStateInfoWithinNode>(
      result->shared_user_data_);
  if (shared_user_data->done_) {
    return;
  }
  shared_user_data->done_ = true;
  // release timer
  if (shared_user_data->timer_token_) {
    Timer::Instance()->RemoveTimer(shared_user_data->timer_token_);
  }

  if (FrameworkDataShell::ShellType::METHOD == result->type_) {
    SetOutputData(result->datas_, result->GetResult());
  } else if (FrameworkDataShell::ShellType::TIMER == result->type_) {
    SetTimeoutFlag(result->datas_);
  } else {
    // TODO(jet) Warning
  }
  for (auto data : result->datas_->datas_) {
    on_ready_(data, shared_from_this());
  }
}

void Node::OnFakeResult(FrameworkDataPtr result) {
  on_ready_(result, shared_from_this());
}

void Node::PostResult(FrameworkDataShellPtr result) {
  daemon_thread_->PostAsyncTask(unique_name_,
                                std::bind(&Node::OnGetResult, this, result));
}

bool Node::IsNeedSkip(const FrameworkDataPtr &framework_data) {
  bool need_skip = false;
  do {
    // filter by input data state
    for (auto slot : input_slots_) {
      if (NULL == framework_data->datas_[slot] ||
          0 != framework_data->datas_[slot]->error_code_) {
        need_skip = true;
        break;
      }
    }
    // filter by parameter
    auto iter =
        framework_data->method_param_.find(method_manager_.MethodName());
    if (iter != framework_data->method_param_.end() &&
        !iter->second->is_enable_this_method_) {
      need_skip = true;
      break;
    }
  } while (0);

  return need_skip;
}

void Node::FakeResult(const FrameworkDataPtr &framework_data) {
  LOGV << "skip " << this->method_manager_.MethodName();
  auto iter = framework_data->method_param_.find(method_manager_.MethodName());
  if (iter != framework_data->method_param_.end() &&
      !iter->second->is_enable_this_method_) {
    auto param = dynamic_cast<DisableParam *>(iter->second.get());
    if (param) {
      switch (param->mode_) {
        case DisableParam::Mode::PassThrough: {
          if (input_slots_.size() == output_slots_.size()) {
            for (decltype(input_slots_)::size_type i = 0;
                 i < input_slots_.size(); ++i) {
              framework_data->datas_[output_slots_[i]] =
                  framework_data->datas_[input_slots_[i]];
            }
          } else {
            LOGE << method_manager_.MethodName()
                 << " : input slot size is not equal to the output data , skip "
                    "data pass through ";
          }
          break;
        }
        case DisableParam::Mode::BestEffortPassThrough: {
          for (decltype(output_slots_)::size_type i = 0;
               i < output_slots_.size(); ++i) {
            if (input_slots_.size() > i) {
              framework_data->datas_[output_slots_[i]] =
                  framework_data->datas_[input_slots_[i]];
            } else {
              BaseDataPtr invalid_data(new BaseData());
              invalid_data->state_ = DataState::INVALID;
              framework_data->datas_[output_slots_[i]] = invalid_data;
            }
          }
          break;
        }
        case DisableParam::Mode::UsePreDefine: {
          if (param->pre_datas_.size() == output_slots_.size()) {
            for (decltype(output_slots_)::size_type i = 0;
                 i < output_slots_.size(); ++i) {
              framework_data->datas_[output_slots_[i]] = param->pre_datas_[i];
            }
          } else {
            LOGI << method_manager_.MethodName() << " : predefine data size is "
                 << param->pre_datas_.size()
                 << ", while method output data size is "
                 << output_slots_.size() << ", skip data pass through ";
          }
          break;
        }
        case DisableParam::Mode::Invalid: {
          for (decltype(output_slots_)::size_type i = 0;
               i < output_slots_.size(); ++i) {
            BaseDataPtr invalid_data(new BaseData());
            invalid_data->state_ = DataState::INVALID;
            framework_data->datas_[output_slots_[i]] = invalid_data;
          }
          break;
        }
      }
    }
  }
}

bool Sponge::Sop(const FrameworkDataPtr &data,
                 std::list<FrameworkDataPtr> *ready) {
  ready->clear();
  if (!is_start_) {
    is_start_ = true;
    expected_sequence_id_ = 0;
  }

  if (expected_sequence_id_ == data->sequence_id_) {
    expected_sequence_id_ = data->sequence_id_ + 1;
    ready->push_back(data);
    while (!cache_list_.empty()) {
      auto front = cache_list_.front();
      if (front->sequence_id_ == expected_sequence_id_) {
        ready->emplace_back(front);
        cache_list_.pop_front();
        ++expected_sequence_id_;
      } else if ((front->sequence_id_ < expected_sequence_id_ &&
                  (expected_sequence_id_ - front->sequence_id_ <
                   cache_max_size_)) ||
                 ((front->sequence_id_ > expected_sequence_id_) &&
                  (front->sequence_id_ - expected_sequence_id_ >
                   0xFFFFFFFFFFF0000))) {
        // when expected_sequence_id_ == curr data->sequence_id_, drop
        // all old frame:
        // 1. cache seqid < expected_sequence_id && distance smaller
        //    than cache_max_size_;
        // 2. cache seqid > expected_sequence_id && distance larger than
        //    0xFFFFFFFFFFF0000, which happens on sequence id overflow.
        ready->emplace_back(front);
        cache_list_.pop_front();
      } else {
        break;
      }
    }
    return true;
  } else {
    if (expected_sequence_id_ > data->sequence_id_) {
      auto res = expected_sequence_id_ - data->sequence_id_;
      if (res > 0xFFFFFFFFFFF0000) {
        // cache
        auto it = cache_list_.begin();
        for (; it != cache_list_.end(); it++) {
          if ((*it)->sequence_id_ < 0xFFFF &&
              (*it)->sequence_id_ > data->sequence_id_) {
            break;
          }
        }
        cache_list_.insert(it, data);
        if (cache_list_.size() > cache_max_size_) {
          ready->swap(cache_list_);
          expected_sequence_id_ = (*ready->rbegin())->sequence_id_ + 1;
        }
        return true;
      }
    } else {
      auto res = data->sequence_id_ - expected_sequence_id_;
      if (res < 0xFFFF) {
        // cache
        auto it = cache_list_.begin();
        for (; it != cache_list_.end(); it++) {
          if ((*it)->sequence_id_ > data->sequence_id_) {
            break;
          }
        }
        cache_list_.insert(it, data);
        if (cache_list_.size() > cache_max_size_) {
          ready->swap(cache_list_);
          expected_sequence_id_ = (*ready->rbegin())->sequence_id_ + 1;
        }
        return true;
      }
    }
    return false;
  }
}

void Node::Do(const FrameworkDataPtr &framework_data) {
  std::list<FrameworkDataPtr> ready;
  if (is_need_reorder_) {
    if (!sponge_list_[framework_data->source_id_].Sop(framework_data, &ready)) {
      LOGW << "Sop failed, curr seqid=" << framework_data->sequence_id_;
      FakeResult(framework_data);
      return;
    }
  } else {
    ready.push_back(framework_data);
  }

  for (auto ptr : ready) {
    if (IsNeedSkip(ptr)) {
      FakeResult(ptr);
      daemon_thread_->PostAsyncTask(unique_name_,
                                    std::bind(&Node::OnFakeResult, this, ptr));
    } else {
      Handle(ptr);
    }
  }
}

void Node::Handle(const FrameworkDataPtr &framework_data) {
  // 构造Batch
  auto data = std::make_shared<FrameworkDataBatch>();
  data->datas_.push_back(framework_data);
  data->timestamp_ = 0;
  // 创建shell user data部分
  auto state_info = std::make_shared<ExtraStateInfoWithinNode>();
  auto extra = std::static_pointer_cast<ExtraInfoWithinNode>(state_info);
  // 创建data shell
  auto shell = std::make_shared<FrameworkDataShell>(
      data, FrameworkDataShell::ShellType::METHOD, extra);
  auto timershell = std::make_shared<FrameworkDataShell>(
      data, FrameworkDataShell::ShellType::TIMER, extra);
  // 创建callback
  auto method_callback =
      [this,
       shell](const std::vector<std::vector<BaseDataPtr>> &method_output) {
        RUN_FPS_PROFILER_WIGH_TAG(this->method_manager_.MethodName(),
                                  "Framework FPS")
        LOGV << this->method_manager_.MethodName() << " OnMethodCallback";
        // 拿到后把数据放到shell里面
        shell->SetResult(method_output);
        // 把shell放到node线程的结果消息队列
        PostResult(shell);
      };
  // 构造method input
  std::vector<std::vector<BaseDataPtr>> inputs = GetInputData(data);
  // 构造method params
  auto params = GetInputParams(data);
  if (setting_timeout_duration_ms_ <= 0) {
    // 注册timer
    state_info->timer_token_ = Timer::Instance()->AddTimer(
        [this, timershell]() {
          LOGI << "OnTimeout()";
          PostResult(timershell);
        },
        setting_timeout_duration_ms_);
  }
  // 启动异步任务
  method_manager_.ProcessAsyncTask(inputs, params,
     method_callback, framework_data->source_id_);
}

std::vector<std::vector<BaseDataPtr>> Node::GetInputData(
    const FrameworkDataBatchPtr &frame_data) const {
  std::vector<std::vector<BaseDataPtr>> data;
  auto &frames = frame_data->datas_;
  size_t batch_size = frames.size();
  data.resize(batch_size);
  for (size_t batch_i = 0; batch_i < batch_size; ++batch_i) {
    auto &f = frames[batch_i];
    auto input_size = input_slots_.size();
    auto &d = data[batch_i];
    d.resize(input_size);
    for (size_t data_i = 0; data_i < input_size; ++data_i) {
      d[data_i] = f->datas_[input_slots_[data_i]];
    }
  }
  return data;
}

std::vector<InputParamPtr> Node::GetInputParams(
    const FrameworkDataBatchPtr &frame_data) const {
  std::vector<InputParamPtr> ret;
  auto &frames = frame_data->datas_;
  size_t batch_size = frames.size();
  ret.resize(batch_size);
  auto method_name = method_manager_.MethodName();
  for (size_t batch_i = 0; batch_i < batch_size; ++batch_i) {
    auto &f = frames[batch_i];
    if (f->method_param_.find(method_name) != f->method_param_.end()) {
      ret[batch_i] = f->method_param_[method_name];
    } else {
      ret[batch_i] = nullptr;
    }
  }
  return ret;
}

void Node::CheckResult(const FrameworkData &frame_data) {
  for (auto in_slot : input_slots_) {
    for (auto out_slot : output_slots_) {
      const auto &in_data = frame_data.datas_[in_slot];
      const auto &out_data = frame_data.datas_[out_slot];
      HOBOT_CHECK(in_data != out_data)
          << "Error on check in_slot " << in_slot << " and out_slot "
          << out_slot << " of method: " << method_manager_.MethodName();
    }
  }
}

void Node::SetOutputData(FrameworkDataBatchPtr frame_data,
                         const std::vector<std::vector<BaseDataPtr>> &data) {
  auto &frames = frame_data->datas_;
  assert(frame_data->datas_.size() == data.size());
  size_t batch_size = frames.size();
  for (size_t batch_i = 0; batch_i < batch_size; ++batch_i) {
    auto &f = frames[batch_i];
    auto &d = data[batch_i];
    HOBOT_CHECK(d.size() >= output_slots_.size());
    if (d.size() > output_slots_.size()) {
      LOGD << this->method_manager_.MethodName()
           << " : current result output size is " << d.size() << ", while "
           << " total needed output slot size is " << output_slots_.size()
           << " , will only use first " << output_slots_.size() << " data";
    }
    HOBOT_CHECK(f->datas_.size() >= d.size());
    auto output_size = output_slots_.size();
    for (size_t data_i = 0; data_i < output_size; ++data_i) {
      f->datas_[output_slots_[data_i]] = d[data_i];
    }
    // check the result, not allow output is input
    // CheckResult(*f);
  }
}

void Node::SetTimeoutFlag(FrameworkDataBatchPtr frame_data) {
  auto &frames = frame_data->datas_;
  size_t batch_size = frames.size();
  for (size_t batch_i = 0; batch_i < batch_size; ++batch_i) {
    auto &f = frames[batch_i];
    assert(f->datas_.size() >= output_slots_.size());
    auto output_size = output_slots_.size();
    for (size_t data_i = 0; data_i < output_size; ++data_i) {
      auto err_data = BaseDataPtr(new BaseData());
      f->datas_[output_slots_[data_i]] = err_data;
      // TODO(jet) define enumeration for error codes
      err_data->error_code_ = HOBOTXROC_ERROR_METHOD_TIMEOUT;
    }
  }
}

int Node::SetParameter(InputParamPtr ptr) {
  return method_manager_.UpdateParameter(ptr);
}

InputParamPtr Node::GetParameter() const {
  return method_manager_.GetParameter();
}

std::string Node::GetVersion() const { return method_manager_.GetVersion(); }

std::string Node::GetUniqueName() const { return unique_name_; }

}  // namespace HobotXRoc
