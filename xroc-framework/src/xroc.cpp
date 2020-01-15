/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc framework interface
 * @file      xroc.cpp
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */

#include "hobotxroc/xroc.h"
#include <future>
#include <string>
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/profiler.h"
#include "hobotxroc/xroc_config.h"
#include "hobotxsdk/xroc_error.h"
namespace HobotXRoc {

XRocSDK *XRocSDK::CreateSDK() { return new XRocFlow(); }

XRocFlow::XRocFlow() {
  is_initial_ = false;
  scheduler_ = std::make_shared<Scheduler>();
}

XRocFlow::~XRocFlow() {}

int XRocFlow::Init() {
  std::unique_lock<std::mutex> locker(mutex_);
  if (is_initial_) {
    return -2;
  }
  auto config = std::make_shared<XRocConfig>();
  int ret;
  if (0 == (ret = config->LoadFile(config_file_))) {
    if (0 == (ret = scheduler_->Init(config))) {
      is_initial_ = true;
      return 0;
    }
  }
  return ret;
}

int XRocFlow::SetConfig(const std::string &key, const std::string &value) {
  std::unique_lock<std::mutex> locker(mutex_);
  if (key.compare("config_file") == 0) {
    config_file_ = value;
    param_dict_["config_file"] = config_file_;
  } else if (key.compare("profiler") == 0) {
    if (value.compare("on") == 0) {
      Profiler::Get()->Start();
    } else {
      Profiler::Get()->Stop();
    }
  } else if (key.compare("profiler_file") == 0) {
    if (!Profiler::Get()->SetOutputFile(value)) {
      return -1;
    }
  } else if (key.compare("profiler_frame_interval") == 0) {
    Profiler::Get()->SetFrameIntervalForTimeStat(std::stoi(value));
  } else if (key.compare("profiler_time_interval") == 0) {
    Profiler::Get()->SetTimeIntervalForFPSStat(std::stoi(value));
  } else if (key.compare("free_framedata") == 0) {
    if (value.compare("on") == 0) {
      scheduler_->SetFreeMemery(true);
    } else {
      scheduler_->SetFreeMemery(false);
    }
  } else {
    LOGD << "config " << key << " is not supported yet";
    return -1;
  }
  return 0;
}

int XRocFlow::SetCallback(XRocCallback callback, const std::string &name) {
  std::unique_lock<std::mutex> locker(mutex_);
  if ("" == name) {
    callback_ = callback;
    return scheduler_->SetCallback(callback_);
  } else {
    return scheduler_->SetCallback(callback, name);
  }
}

int XRocFlow::UpdateConfig(const std::string &method_name,
                           InputParamPtr param_ptr) {
  std::unique_lock<std::mutex> locker(mutex_);
  if (!is_initial_) {
    return -1;
  }
  return scheduler_->UpdateConfig(method_name, param_ptr);
}

InputParamPtr XRocFlow::GetConfig(const std::string &method_name) const {
  //  std::unique_lock<std::mutex> locker(mutex_);
  return scheduler_->GetConfig(method_name);
}

std::string XRocFlow::GetVersion(const std::string &method_name) const {
  return scheduler_->GetVersion(method_name);
}

OutputDataPtr XRocFlow::OnError(int64_t error_code,
                                const std::string &error_detail) {
  auto output = std::make_shared<OutputData>();
  output->error_code_ = error_code;
  output->error_detail_ = error_detail;
  return output;
}

// 同步接口，单路输出
OutputDataPtr XRocFlow::SyncPredict(InputDataPtr input) {
  if (!input || input->datas_.size() <= 0) {
    OnError(HOBOTXROC_ERROR_INPUT_INVALID, "input error");
  }
  std::promise<std::vector<OutputDataPtr>> promise;
  auto future = promise.get_future();

  int64_t ret = scheduler_->Input(input, &promise);
  if (ret >= 0) {
    auto output = future.get();
    HOBOT_CHECK(output.size() == 1) << "Output size error";
    return output[0];
  } else {
    return OnError(ret, "exceed max running count");
  }
}

// 同步接口，多路输出
std::vector<OutputDataPtr> XRocFlow::SyncPredict2(InputDataPtr input) {
  if (!input || input->datas_.size() <= 0) {
    OnError(HOBOTXROC_ERROR_INPUT_INVALID, "input error");
  }
  std::promise<std::vector<OutputDataPtr>> promise;
  auto future = promise.get_future();

  int64_t ret = scheduler_->Input(input, &promise);
  if (ret >= 0) {
    return future.get();
  } else {
    std::vector<OutputDataPtr> err_output;
    err_output.push_back(OnError(ret, "exceed max running count"));
    return err_output;
  }
}

int64_t XRocFlow::AsyncPredict(InputDataPtr input) {
  HOBOT_CHECK(callback_)
      << "callback error, AsyncPredict need a valid callback function.";
  return scheduler_->Input(input, nullptr);
}

}  // namespace HobotXRoc
