/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file passthroughMethod.h
 * @brief
 * @author ronghui.zhang
 * @email ronghui.zhang@horizon.ai
 * @date 2019/12/4
 */

#ifndef TEST_INCLUDE_PASSTHROUGHMETHOD_H_
#define TEST_INCLUDE_PASSTHROUGHMETHOD_H_

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/data_types/orderdata.h"

namespace HobotXRoc {

class passthroughMethod : public Method {
 public:
  int Init(const std::string &config_file_path) override {
    return 0;
}

  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<HobotXRoc::InputParamPtr> &param) override {
  std::cout << "passthroughMethod doprocess" << std::endl;
  std::vector<std::vector<BaseDataPtr>> output;

  output.resize(input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    auto &in_batch_i = input[i];
    auto &out_batch_i = output[i];
    out_batch_i.resize(in_batch_i.size());
    std::cout << "input size: " << in_batch_i.size() << std::endl;
    // 只支持n个输入，输入格式是BBox的数组
    for (size_t j = 0; j < in_batch_i.size(); ++j) {
      if (in_batch_i[j]->state_ == DataState::INVALID) {
        std::cout << "input slot " << j << " is invalid" << std::endl;
        continue;
      }
      auto in_datas = std::static_pointer_cast<BaseDataVector>(in_batch_i[j]);
      auto out_datas = std::make_shared<BaseDataVector>();
      out_batch_i[j] = std::static_pointer_cast<BaseData>(out_datas);
      for (auto &in_data : in_datas->datas_) {
        auto safeData = std::static_pointer_cast<HobotXRoc::OrderData>(in_data);
        std::cout << "!!!!!passthrough  sequence_id "
        << safeData->sequence_id << std::endl;

        out_datas->datas_.push_back(in_data);
      }
    }
  }

    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_int_distribution<> dis(1, 100);

    int duration = dis(engine);
    std::cout << "this thread sleep " << duration << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));

    return output;
  }

  void Finalize() override {}

  int UpdateParameter(InputParamPtr ptr) override { return 0; }

  InputParamPtr GetParameter() const override { return InputParamPtr(); }

  std::string GetVersion() const override { return "0.0.0"; }

  MethodInfo GetMethodInfo() override {
    MethodInfo orderMethod = MethodInfo();
    orderMethod.is_thread_safe_ = false;
    orderMethod.is_need_reorder = true;

    return orderMethod;
  }

  void OnProfilerChanged(bool on) override {}
};
}  // namespace HobotXRoc
#endif  // TEST_INCLUDE_PASSTHROUGHMETHOD_H_
