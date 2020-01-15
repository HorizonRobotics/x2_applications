/**
* Copyright (c) 2019 Horizon Robotics. All rights reserved.
* @file profiler_test.cpp
* @brief 
* @author ruoting.ding
* @email ruoting.ding@horizon.ai
* @date 2019/4/17
*/

#include <chrono>
#include <iostream>
#include <thread>
#include "hobotxroc/data_types/bbox.h"
#include "hobotxsdk/xroc_sdk.h"
#include "callback.h"

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    printf("Usage : ./profiler_test config\n");
    printf("Example : ./profiler_test ./config/filter.json\n");
    return -1;
  }
  auto config = argv[1];

  using HobotXRoc::BaseData;
  using HobotXRoc::BaseDataPtr;
  using HobotXRoc::BaseDataVector;
  using HobotXRoc::InputData;
  using HobotXRoc::InputDataPtr;
  HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
  int target = 10;
  Callback callback(target);
  flow->SetCallback(std::bind(&Callback::OnCallback,
                              &callback,
                              std::placeholders::_1));
  flow->SetConfig("config_file", config);
  flow->SetConfig("profiler", "on");
  flow->SetConfig("profiler_file", "./profiler.txt");
  flow->SetConfig("profiler_frame_interval", "8");
  flow->SetConfig("profiler_time_interval", "100");
  flow->Init();

  InputDataPtr inputdata(new InputData());
  BaseDataVector *data(new BaseDataVector);
  HobotXRoc::BBox *bbox1(new HobotXRoc::BBox(
    hobot::vision::BBox(0, 0, 1000, 1000)));
  bbox1->type_ = "BBox";
  data->name_ = "face_head_box";
  data->datas_.push_back(BaseDataPtr(bbox1));
  inputdata->datas_.push_back(BaseDataPtr(data));

  for (auto i = 0; i < target; ++i) {
    flow->AsyncPredict(inputdata);
  }
  callback.OnReady();
  delete flow;
  // 初始化sdk
  return 0;
}
