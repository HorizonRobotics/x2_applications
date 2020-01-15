/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file TestMethod.h
 * @brief
 * @author ruoting.ding
 * @email ruoting.ding@horizon.ai
 * @date 2019/9/24
 */

#ifndef TEST_INCLUDE_TESTMETHOD_H_
#define TEST_INCLUDE_TESTMETHOD_H_

#include <iostream>
#include <string>
#include <vector>
#include "hobotlog/hobotlog.hpp"
namespace HobotXRoc {

class TestMethod : public Method {
 public:
  int Init(const std::string &config_file_path) override { return 0; }

  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<HobotXRoc::InputParamPtr> &param) override {
    std::vector<std::vector<BaseDataPtr>> output;
    output.resize(input.size());
    for (uint i = 0; i < input.size(); ++i) {
      HOBOT_CHECK(input[i].size() == 1);
      output[i].resize(4);
      // 1 is ok
      // 2 and 3 is nullptr
      // 4 is ok
      for (uint j = 0; j < 4; ++j) {
        if (j == 0 || j == 3) {
          auto value = new BaseData();
          *value = *input[i][0].get();
          output[i][j] = BaseDataPtr(value);
        } else {
          output[i][j] = BaseDataPtr();
        }
      }
    }

    return output;
  }

  void Finalize() override {}

  int UpdateParameter(InputParamPtr ptr) override { return 0; }

  InputParamPtr GetParameter() const override { return InputParamPtr(); }

  std::string GetVersion() const override { return "0.0.0"; }

  void OnProfilerChanged(bool on) override {}
};

}  //  namespace HobotXRoc

#endif  //  TEST_INCLUDE_TESTMETHOD_H_
