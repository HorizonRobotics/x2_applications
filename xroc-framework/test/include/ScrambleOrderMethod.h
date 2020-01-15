/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file ScrambleOrderMethod.h
 * @brief
 * @author guoqian.sun
 * @email guoqian.sun@horizon.ai
 * @date 2019/12/06
 */

#ifndef XROC_FRAMEWORK_TEST_INCLUDE_SCRAMBLEORDERMETHOD_H_
#define XROC_FRAMEWORK_TEST_INCLUDE_SCRAMBLEORDERMETHOD_H_

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <random>
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/method.h"
namespace HobotXRoc {

class ScrambleOrderMethod : public Method {
 public:
  int Init(const std::string &config_file_path) override {return 0;};

  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<HobotXRoc::InputParamPtr> &param) override {
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_int_distribution<> dis(1, 100);

    int duration = dis(engine);
    LOGD << "this thread sleep " << duration << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));

    return input;
  }

  void Finalize() override {}

  int UpdateParameter(InputParamPtr ptr) override { return 0; }

  InputParamPtr GetParameter() const override { return InputParamPtr(); }

  std::string GetVersion() const override { return "reorder0.0.0"; }

  void OnProfilerChanged(bool on) override {}
};
}  // namespace HobotXRoc
#endif  // XROC_FRAMEWORK_TEST_INCLUDE_SCRAMBLEORDERMETHOD_H_
