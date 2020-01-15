/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: ronghui zhang
 * @Mail: zhangronghui@horizon.ai
 * @Date: 2019-11-22 01:15:22
 * @Version: v0.0.1
 * @Brief: test thread order
 * @Last Modified by: Ronghui Zhang
 * @Last Modified time: 2019-11-11 21:29:55
 */

#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <random>
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/data_types/orderdata.h"
#include "hobotxroc/data_types/filter_param.h"
#include "hobotxroc/xroc_config.h"
#include "hobotxsdk/xroc_error.h"
#include "hobotxsdk/xroc_sdk.h"
#include "method/bbox_filter.h"
#include "OrderTestMethod.h"


namespace ThreadOrder {
clock_t begin_clock = 1;
static int g_sequence_id = 0;
class Callback {
 public:
  Callback() {}

  ~Callback() {}

  void OnCallback(HobotXRoc::OutputDataPtr output) {
    using HobotXRoc::BaseDataVector;
    if ((output->sequence_id_ == 99999) ||
        (output->sequence_id_ % 1000 == 0)) {
      auto duration = clock() - begin_clock;
      std::cout << "seq: " << output->sequence_id_
                << " duration:" << duration << std::endl;
    }

    std::cout << "======================" << std::endl;
    std::cout << "seq: " << output->sequence_id_ << std::endl;
    std::cout << "method_name: " << output->method_name_ << std::endl;
    std::cout << "error_code: " << output->error_code_ << std::endl;
    std::cout << "error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "datas_ size: " << output->datas_.size() << std::endl;
    std::cout << "sequence_id_: "<< output->sequence_id_ << std::endl;
    EXPECT_EQ(g_sequence_id, output->sequence_id_);
    g_sequence_id++;
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
  }
};


int OrderTest(const std::string &config) {
  using HobotXRoc::BaseData;
  using HobotXRoc::BaseDataPtr;
  using HobotXRoc::BaseDataVector;
  using HobotXRoc::InputData;
  using HobotXRoc::InputDataPtr;

  HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(std::bind(&Callback::OnCallback,
                    &callback,
                    std::placeholders::_1));
  flow->SetConfig("config_file", config);
  flow->Init();

  std::cout << "data_test Method Version : "
            << flow->GetVersion("data_test_order1") << std::endl;

  InputDataPtr inputdata(new InputData());
  BaseDataVector *data(new BaseDataVector);
  HobotXRoc::OrderData *testData(new HobotXRoc::OrderData(1));
  testData->name_ = "input_test_data";
  data->name_ = "input_test_data";
  data->datas_.push_back(BaseDataPtr(testData));
  inputdata->datas_.push_back(BaseDataPtr(data));

  std::cout << "***********************" << std::endl
            << "testing aysnc function" << std::endl
            << "***********************" << std::endl;
  for (int i = 0; i < 10; i++) {
    flow->AsyncPredict(inputdata);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // waiting for async function done
  std::this_thread::sleep_for(std::chrono::seconds(1));
  delete flow;
  // 初始化sdk
  return 0;
}

}  // namespace ThreadOrder

TEST(ThreadOrder, compatiable) {
  ThreadOrder::OrderTest(
      "./test/configs/dataOrder_Test.json");
}

