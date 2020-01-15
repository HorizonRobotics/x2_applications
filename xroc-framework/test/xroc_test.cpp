/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file xroc_test.cc
 * @brief
 * @author shiqing.xie
 * @email shiqing.xie@horizon.ai
 * @date 2019/11/21
 */
#include <gtest/gtest.h>
#include <future>
#include <string>
#include <iostream>
#include <thread>
#include <vector>
#include "hobotxsdk/xroc_data.h"
#include "hobotxsdk/xroc_error.h"
#include "hobotxsdk/xroc_sdk.h"
#include "include/test_param.h"
#include "memory"

using PromiseType = std::promise<HobotXRoc::OutputDataPtr>;
namespace xroc_test
{
class Callback
{
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
}  //  namespace xroc_test

TEST(Interface, UpdataConfig)
{
  auto xroc = HobotXRoc::XRocSDK::CreateSDK();
  ASSERT_TRUE(xroc);
  xroc_test::Callback callback;

  EXPECT_EQ(0, xroc->SetConfig("config_file",
                               "./test/configs/UpdataConfig.json"));
  EXPECT_EQ(0, xroc->SetConfig("profiler", "on"));
  EXPECT_EQ(0, xroc->SetConfig("profiler_file", "./profiler.txt"));
  EXPECT_EQ(0, xroc->Init());
  ASSERT_EQ(0, xroc->SetCallback(
    std::bind(&xroc_test::Callback::OnCallback, &callback,
      std::placeholders::_1)));
  ASSERT_EQ(0, xroc->SetCallback(std::bind(
      &xroc_test::Callback::OnCallback, &callback,
      std::placeholders::_1), "method_1"));
  HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
  auto xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
  xroc_input_data->name_ = "input_in";
  xroc_input_data->state_ = HobotXRoc::DataState::INVALID;
  inputdata->datas_.emplace_back(xroc_input_data);
  PromiseType p;
  auto f = p.get_future();
  inputdata->context_ = &p;
  std::string method_name = "method_1";
  auto ipp = std::make_shared<HobotXRoc::TestParam>(method_name);
  EXPECT_EQ(xroc->UpdateConfig(method_name, ipp), 0);
  xroc->AsyncPredict(inputdata);
  auto output = f.get();
  EXPECT_EQ(output->error_code_, 0);
  EXPECT_EQ(output->datas_.size(), inputdata->datas_.size());
  EXPECT_EQ(output->datas_.front()->state_, inputdata->datas_.front()->state_);
  delete xroc;
}

/* 确保不调用Init接口的情况下，XRocSDK也可以正常析构 */
TEST(Interface, SingleAPI_CreateSDK) {
  bool is_normal = true;
  {
    auto sdk = std::shared_ptr<HobotXRoc::XRocSDK>(
      HobotXRoc::XRocSDK::CreateSDK());
  }

  ASSERT_TRUE(is_normal);
}

TEST(Interface, ReCreateSDK) {
  for (int i = 0; i < 10; ++i) {
    auto xroc = HobotXRoc::XRocSDK::CreateSDK();
    ASSERT_TRUE(xroc);
    xroc_test::Callback callback;
    EXPECT_EQ(0, xroc->SetConfig("config_file",
      "./test/configs/UpdataConfig.json"));
    EXPECT_EQ(0, xroc->Init());
    HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
    auto xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
    xroc_input_data->name_ = "input_in";
    xroc_input_data->state_ = HobotXRoc::DataState::VALID;
    inputdata->datas_.emplace_back(xroc_input_data);
    PromiseType p;
    auto f = p.get_future();
    inputdata->context_ = &p;
    auto out = xroc->SyncPredict(inputdata);
    callback.OnCallback(out);
    delete xroc;
  }
}

TEST(Interface, MultiSDK)
{
  auto xroc = HobotXRoc::XRocSDK::CreateSDK();
  auto xroc_2 = HobotXRoc::XRocSDK::CreateSDK();
  ASSERT_TRUE(xroc);
  ASSERT_TRUE(xroc_2);
  xroc_test::Callback callback;

  EXPECT_EQ(0, xroc->SetConfig("config_file",
    "./test/configs/UpdataConfig.json"));
  EXPECT_EQ(0, xroc->SetConfig("profiler", "on"));
  EXPECT_EQ(0, xroc->Init());

  EXPECT_EQ(0, xroc_2->SetConfig("config_file",
    "./test/configs/UpdataConfig.json"));
  EXPECT_EQ(0, xroc_2->SetConfig("profiler", "on"));
  EXPECT_EQ(0, xroc_2->Init());

  ASSERT_EQ(0, xroc->SetCallback(std::bind(
    &xroc_test::Callback::OnCallback, &callback,
    std::placeholders::_1)));
  ASSERT_EQ(0, xroc->SetCallback(std::bind(
    &xroc_test::Callback::OnCallback, &callback,
    std::placeholders::_1), "method_1"));
  ASSERT_EQ(0, xroc->SetCallback(std::bind(
    &xroc_test::Callback::OnCallback, &callback,
    std::placeholders::_1), "method_2"));
  ASSERT_EQ(0, xroc->SetCallback(std::bind(
    &xroc_test::Callback::OnCallback, &callback,
    std::placeholders::_1), "method_3"));

  HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
  auto xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
  xroc_input_data->name_ = "input_in";
  xroc_input_data->state_ = HobotXRoc::DataState::VALID;
  inputdata->datas_.emplace_back(xroc_input_data);
  PromiseType p;
  auto f = p.get_future();
  inputdata->context_ = &p;
  std::string method_name = "method_1";
  auto ipp = std::make_shared<HobotXRoc::TestParam>(method_name);
  EXPECT_EQ(xroc->UpdateConfig(method_name, ipp), 0);
  std::string method_name_2 = "method_2";
  auto ipp_2 = std::make_shared<HobotXRoc::TestParam>(method_name_2);
  EXPECT_EQ(xroc->UpdateConfig(method_name_2, ipp_2), 0);
  std::string method_name_3 = "method_3";
  auto ipp_3 = std::make_shared<HobotXRoc::TestParam>(method_name_3);
  EXPECT_EQ(xroc->UpdateConfig(method_name_3, ipp_3), 0);

  xroc->AsyncPredict(inputdata);
  auto output = f.get();
  EXPECT_EQ(output->error_code_, 0);
  EXPECT_EQ(output->datas_.size(), inputdata->datas_.size());
  EXPECT_EQ(output->datas_.front()->state_, inputdata->datas_.front()->state_);

  PromiseType p1;
  auto f1 = p1.get_future();
  inputdata->context_ = &p1;
  auto ipp_SyncPredict = std::make_shared<HobotXRoc::TestParam>(method_name);
  EXPECT_EQ(xroc_2->UpdateConfig(method_name, ipp_SyncPredict), 0);
  auto out = xroc_2->SyncPredict(inputdata);
  callback.OnCallback(out);

  delete xroc;
  delete xroc_2;
}

void TestSDK(int IterNum, const std::string &config)
{
  for (int i = 0; i < 100; i++)
  {
    std::cout << "Start running the:[" << IterNum + 1
              << "] Thread and start create the:["
              << i + 1 << "] xroc-framework-SDK\n"
              << std::endl;
    auto xroc = HobotXRoc::XRocSDK::CreateSDK();
    ASSERT_TRUE(xroc);
    xroc_test::Callback callback;
    EXPECT_EQ(0, xroc->SetConfig("config_file", config));
    EXPECT_EQ(0, xroc->Init());
    HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
    auto xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
    xroc_input_data->name_ = "input_in";
    xroc_input_data->state_ = HobotXRoc::DataState::VALID;
    inputdata->datas_.emplace_back(xroc_input_data);
    PromiseType p;
    auto f = p.get_future();
    inputdata->context_ = &p;
    auto out = xroc->SyncPredict(inputdata);
    callback.OnCallback(out);
    EXPECT_EQ(out->datas_.front()->state_,
              HobotXRoc::DataState::VALID);
    delete xroc;
  }
}

int MulTHDCreateSDKTest(const std::string &config)
{
  std::cout << "MulTHDCreateSDKTest" << std::endl;
  std::vector<std::thread> testthread;
  for (int i = 0; i < 20; i++)
  {
    testthread.push_back(std::thread(TestSDK, i, config));
  }
  for (auto iter = testthread.begin();
       iter != testthread.end(); ++iter)
  {
    iter->join();
  }

  return 0;
}

TEST(Interface, MulTHDCreateSDK)
{
  MulTHDCreateSDKTest("./test/configs/UpdataConfig.json");
}
