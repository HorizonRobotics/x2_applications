/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file cpp_api_test.cc
 * @brief
 * @author tangji.sun
 * @email tangji.sun@horizon.ai
 * @date 2019/10/8
 */
#include <gtest/gtest.h>
#include <future>
#include "hobotxsdk/xroc_data.h"
#include "hobotxsdk/xroc_error.h"
#include "hobotxsdk/xroc_sdk.h"
#include "include/test_param.h"
#include "memory"

using PromiseType = std::promise<HobotXRoc::OutputDataPtr>;
namespace XRocAPITest {
class Callback {
 public:
  void OnCallback(HobotXRoc::OutputDataPtr output) {
    ASSERT_TRUE(output);
    ASSERT_TRUE(output->context_);
    if (output->method_name_.empty()) {
      auto promise = reinterpret_cast<const PromiseType *>(output->context_);
      const_cast<PromiseType *>(promise)->set_value(output);
    } else {
      tmp_result = output;
    }
  }
  HobotXRoc::OutputDataPtr tmp_result = nullptr;
};
}  // namespace XRocAPITest

TEST(Interface, Config) {
  auto xroc = HobotXRoc::XRocSDK::CreateSDK();
  ASSERT_TRUE(xroc);
  XRocAPITest::Callback callback;

  EXPECT_EQ(0, xroc->SetConfig("config_file", "./test/configs/basic.json"));
  EXPECT_EQ(0, xroc->SetConfig("profiler", "on"));
  EXPECT_EQ(0, xroc->SetConfig("profiler_file", "./profiler.txt"));
  EXPECT_EQ(0, xroc->Init());
  ASSERT_EQ(0, xroc->SetCallback(
    std::bind(&XRocAPITest::Callback::OnCallback,
      &callback,
      std::placeholders::_1)));
  ASSERT_EQ(0, xroc->SetCallback(
    std::bind(&XRocAPITest::Callback::OnCallback,
      &callback,
      std::placeholders::_1),
    "first_method"));
  HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
  auto xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
  xroc_input_data->name_ = "global_in";
  xroc_input_data->state_ = HobotXRoc::DataState::INVALID;
  inputdata->datas_.emplace_back(xroc_input_data);
  PromiseType p;
  auto f = p.get_future();
  inputdata->context_ = &p;
  std::string method_name = "first_method";
  auto ipp = std::make_shared<HobotXRoc::TestParam>(method_name);
  EXPECT_EQ(xroc->GetConfig(method_name), nullptr);
  EXPECT_EQ(xroc->UpdateConfig(method_name, ipp), 0);
  EXPECT_EQ(xroc->GetVersion(method_name), "0.0.0");
  xroc->AsyncPredict(inputdata);
  auto output = f.get();
  EXPECT_EQ(output->error_code_, 0);
  EXPECT_EQ(output->datas_.size(), inputdata->datas_.size());
  EXPECT_EQ(output->datas_.front()->state_, inputdata->datas_.front()->state_);
  delete xroc;
}
TEST(Interface, SyncPredict) {
  auto xroc = HobotXRoc::XRocSDK::CreateSDK();
  ASSERT_TRUE(xroc);
  EXPECT_EQ(0, xroc->SetConfig("config_file", "./test/configs/basic.json"));
  EXPECT_EQ(0, xroc->Init());
  HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
  auto xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
  xroc_input_data->name_ = "global_in";
  xroc_input_data->state_ = HobotXRoc::DataState::INVALID;
  inputdata->datas_.emplace_back(xroc_input_data);
  auto output = xroc->SyncPredict(inputdata);
  EXPECT_EQ(output->error_code_, 0);
  EXPECT_EQ(output->datas_.size(), inputdata->datas_.size());
  EXPECT_EQ(output->datas_.front()->state_, inputdata->datas_.front()->state_);
  delete xroc;
}
TEST(Interface, DisableParam) {
  auto xroc = HobotXRoc::XRocSDK::CreateSDK();
  ASSERT_TRUE(xroc);
  EXPECT_EQ(0, xroc->SetConfig("config_file", "./test/configs/basic.json"));
  EXPECT_EQ(0, xroc->Init());
  HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
  auto xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
  xroc_input_data->name_ = "global_in";
  inputdata->datas_.emplace_back(xroc_input_data);

  HobotXRoc::InputParamPtr invalid(new HobotXRoc::DisableParam("first_method"));
  inputdata->params_.push_back(invalid);
  auto output = xroc->SyncPredict(inputdata);
  EXPECT_EQ(output->error_code_, 0);
  EXPECT_EQ(output->datas_.front()->state_, HobotXRoc::DataState::INVALID);

  inputdata->params_.clear();
  HobotXRoc::InputParamPtr best_effort_pass_through(new HobotXRoc::DisableParam(
      "first_method", HobotXRoc::DisableParam::Mode::BestEffortPassThrough));
  inputdata->params_.push_back(best_effort_pass_through);
  output = xroc->SyncPredict(inputdata);
  EXPECT_EQ(output->datas_.size(), inputdata->datas_.size());

  inputdata->params_.clear();
  HobotXRoc::DisableParamPtr pass_through(new HobotXRoc::DisableParam(
      "first_method", HobotXRoc::DisableParam::Mode::PassThrough));
  inputdata->params_.push_back(pass_through);
  output = xroc->SyncPredict(inputdata);
  // EXPECT_EQ(output->datas_.size(), inputdata->datas_.size());
  EXPECT_EQ(output->error_code_, HOBOTXROC_ERROR_OUTPUT_NOT_READY);

  inputdata->params_.clear();
  HobotXRoc::DisableParamPtr pre_define(new HobotXRoc::DisableParam(
      "first_method", HobotXRoc::DisableParam::Mode::UsePreDefine));
  HobotXRoc::InputDataPtr pre_data(new HobotXRoc::InputData());
  xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
  xroc_input_data->name_ = "global_in";
  xroc_input_data->state_ = HobotXRoc::DataState::INVALID;
  pre_data->datas_.emplace_back(xroc_input_data);
  pre_data->params_.push_back(pre_define);
  output = xroc->SyncPredict(pre_data);
  EXPECT_EQ(output->error_code_, HOBOTXROC_ERROR_OUTPUT_NOT_READY);
  // EXPECT_EQ(output->datas_.size(), inputdata->datas_.size());

  delete xroc;
}
