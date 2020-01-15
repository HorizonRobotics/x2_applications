/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file OrderTestMethod.h
 * @brief
 * @author ronghui.zhang
 * @email ronghui.zhang@horizon.ai
 * @date 2019/11/25
 */

#ifndef TEST_INCLUDE_ORDERTESTMETHOD_H_
#define TEST_INCLUDE_ORDERTESTMETHOD_H_

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <random>
#include "hobotlog/hobotlog.hpp"

namespace HobotXRoc {
static int g_method_id = 0;

class OrderTestThread : public Method {
 public:
  int Init(const std::string &config_file_path) override {
    method_id = g_method_id;
    g_method_id++; return 0; }

  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<HobotXRoc::InputParamPtr> &param) override {
    std::cout << "method_id = " << method_id <<
    " OrderTestThread DoProcess input.size(): " << input.size() << std::endl;

    std::vector<std::vector<BaseDataPtr>> output;
    output.resize(input.size());
    for (uint i = 0; i < input.size(); ++i) {
      HOBOT_CHECK(input[i].size() == 1);
      output[i].resize(input[i].size());

    for (uint j = 0; j < output[i].size(); ++j) {
          auto value = new BaseData();
          std::cout << "ordertestthread value " << value << std::endl;
          *value = *input[i][0].get();
          std::cout << "ordertestthread value " << value << std::endl;

          output[i][j] = BaseDataPtr(value);
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

 private:
  int method_id;
};
}  // namespace HobotXRoc
#endif  // TEST_INCLUDE_ORDERTESTMETHOD_H_
