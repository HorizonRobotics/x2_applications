/**
* Copyright (c) 2019 Horizon Robotics. All rights reserved.
* @file callback.h
* @brief 
* @author ruoting.ding
* @email ruoting.ding@horizon.ai
* @date 2019/4/17
*/

#ifndef XROC_FRAMEWORK_EXAMPLE_BBOX_FILTER_INCLUDE_CALLBACK_H_
#define XROC_FRAMEWORK_EXAMPLE_BBOX_FILTER_INCLUDE_CALLBACK_H_
#include "hobotxsdk/xroc_error.h"
#include "hobotxsdk/xroc_sdk.h"
#include <atomic>
#include <iostream>
#include <thread>
class Callback {
 public:
  Callback(int target) {
    num_ = 0;
    target_ = target;
  }

  void OnCallback(HobotXRoc::OutputDataPtr output) {
    using HobotXRoc::BaseDataVector;
    std::cout << "seq: " << output->sequence_id_ << std::endl;
    std::cout << "output_type: " << output->output_type_ << std::endl;
    std::cout << "error_code: " << output->error_code_ << std::endl;
    std::cout << "error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "datas_ size: " << output->datas_.size() << std::endl;
    for (auto data : output->datas_) {
      if (data->error_code_ < 0) {
        std::cout << "data error: " << data->error_code_ << std::endl;
        continue;
      }
      std::cout << "data type_name : " << data->type_ << " " << data->name_
                << std::endl;
      BaseDataVector *pdata = reinterpret_cast<BaseDataVector *>(data.get());
      std::cout << "pdata size: " << pdata->datas_.size() << std::endl;
    }

    if (++num_ == target_) {
      ready = true;
    }
  }

  void OnReady() const {
    while (!ready) {
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    return;
  }
 private:
  std::atomic_int_fast8_t num_;
  int target_;
  bool ready = false;
};
#endif //XROC_FRAMEWORK_EXAMPLE_BBOX_FILTER_INCLUDE_CALLBACK_H_

