/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file method_cb.cpp
 * @brief
 * @author ruoting.ding
 * @email ruoting.ding@horizon.ai
 * @date 2019/9/24
 */

#include <gtest/gtest.h>
#include <future>
#include <vector>
#include "hobotxsdk/xroc_sdk.h"
#include "memory"
#include "hobotlog/hobotlog.hpp"

using PromiseType = std::promise<HobotXRoc::OutputDataPtr>;
namespace MethodCallback {
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
}  // namespace MethodCallback

TEST(CallBack, Global) {
  auto xroc = HobotXRoc::XRocSDK::CreateSDK();
  ASSERT_TRUE(xroc);
  MethodCallback::Callback callback;
  EXPECT_EQ(0, xroc->SetConfig("config_file", "./test/configs/basic.json"));
  EXPECT_EQ(0, xroc->Init());
  xroc->SetCallback(
      std::bind(&MethodCallback::Callback::OnCallback,
        &callback,
        std::placeholders::_1));
  HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
  auto xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
  xroc_input_data->name_ = "global_in";
  xroc_input_data->state_ = HobotXRoc::DataState::INVALID;
  inputdata->datas_.emplace_back(xroc_input_data);
  // Call 10 times at the same time
  std::vector<PromiseType> promises;
  promises.resize(10);
  std::vector<std::future<HobotXRoc::OutputDataPtr>> futures;
  for (auto &promise : promises) {
    inputdata->context_ = &promise;
    xroc->AsyncPredict(inputdata);
    futures.emplace_back(promise.get_future());
  }

  for (auto &future : futures) {
    auto output = future.get();
    EXPECT_EQ(output->error_code_, 0);
    EXPECT_EQ(output->datas_.size(), inputdata->datas_.size());
    EXPECT_EQ(output->datas_.front()->state_,
              inputdata->datas_.front()->state_);
  }

  delete xroc;
}

TEST(CallBack, Method) {
  auto xroc = HobotXRoc::XRocSDK::CreateSDK();
  ASSERT_TRUE(xroc);
  EXPECT_EQ(0, xroc->SetConfig("config_file", "./test/configs/basic.json"));
  EXPECT_EQ(0, xroc->Init());
  MethodCallback::Callback callback;
  xroc->SetCallback(
      std::bind(&MethodCallback::Callback::OnCallback,
        &callback,
        std::placeholders::_1));
  xroc->SetCallback(
      std::bind(&MethodCallback::Callback::OnCallback,
        &callback,
        std::placeholders::_1),
      "first_method");
  HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
  auto xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
  xroc_input_data->name_ = "global_in";
  xroc_input_data->state_ = HobotXRoc::DataState::INVALID;
  inputdata->datas_.emplace_back(xroc_input_data);
  PromiseType p;
  auto f = p.get_future();
  inputdata->context_ = &p;
  xroc->AsyncPredict(inputdata);
  auto output = f.get();
  EXPECT_EQ(output->error_code_, 0);
  EXPECT_EQ(output->datas_.size(), inputdata->datas_.size());
  EXPECT_EQ(output->datas_.front()->state_, inputdata->datas_.front()->state_);
  EXPECT_TRUE(callback.tmp_result);
  if (callback.tmp_result) {
    EXPECT_NE(callback.tmp_result->error_code_, 0);
    EXPECT_EQ(callback.tmp_result->datas_.size(), uint(2));
    if (callback.tmp_result->datas_.size() == 4) {
      EXPECT_EQ(callback.tmp_result->datas_[0]->name_, "tmp_data1");
      EXPECT_EQ(callback.tmp_result->datas_[1]->name_, "tmp_data2");
    }
  }
  // unset callback
  callback.tmp_result = nullptr;
  xroc->SetCallback(nullptr, "first_method");
  PromiseType p_unset_cb;
  auto f_unset_cb = p_unset_cb.get_future();
  inputdata->context_ = &p_unset_cb;
  xroc->SetConfig("free_framedata", "on");
  xroc->AsyncPredict(inputdata);
  output = f_unset_cb.get();
  EXPECT_EQ(output->error_code_, 0);
  EXPECT_EQ(callback.tmp_result, nullptr);
  delete xroc;
}
