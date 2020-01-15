/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file test_param.h
 * @brief
 * @author tangji.sun
 * @email tangji.sun@horizon.ai
 * @date 2019/10/8
 */
#ifndef TEST_INCLUDE_TEST_PARAM_H_
#define TEST_INCLUDE_TEST_PARAM_H_

#include <string>
#include "hobotxsdk/xroc_capi_type.h"
#include "hobotxsdk/xroc_data.h"
namespace HobotXRoc {
class TestParam : public InputParam {
 public:
  explicit TestParam(std::string method_name) : InputParam(method_name) {}
  virtual ~TestParam() = default;

  std::string Format() override { return ""; }
};

}  // namespace HobotXRoc

#endif  // TEST_INCLUDE_TEST_PARAM_H_
