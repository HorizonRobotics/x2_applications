/**
* Copyright (c) 2019 Horizon Robotics. All rights reserved.
* @file disable_method_test.cpp
* @brief 
* @author ruoting.ding
* @email ruoting.ding@horizon.ai
* @date 2019/4/19
*/

#include <chrono>
#include <iostream>
#include <thread>
#include "hobotxroc/data_types/bbox.h"
#include "hobotxsdk/xroc_sdk.h"
#include "callback.h"

using HobotXRoc::InputParam;
using HobotXRoc::InputParamPtr;
using HobotXRoc::BaseData;
using HobotXRoc::BaseDataPtr;
using HobotXRoc::BaseDataVector;
using HobotXRoc::InputData;
using HobotXRoc::InputDataPtr;

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    printf("Usage : ./disable_method_test config\n");
    printf("Example : ./disable_method_test ./config/filter.json\n");
    return -1;
  }
  auto config = argv[1];
  HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();

  Callback callback(1);
  flow->SetCallback(std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", config);
  flow->Init();

  InputDataPtr inputdata(new InputData());
  BaseDataVector *data(new BaseDataVector);
  HobotXRoc::BBox *bbox1(new HobotXRoc::BBox(
    hobot::vision::BBox(0, 0, 10, 10)));
  bbox1->type_ = "BBox";
  data->name_ = "face_head_box";
  data->datas_.push_back(BaseDataPtr(bbox1));
  inputdata->datas_.push_back(BaseDataPtr(data));
  // 1 invalid output
  std::cout << "------------test invalid output------------" << std::endl;
  HobotXRoc::InputParamPtr invalid(new HobotXRoc::DisableParam("BBoxFilter_1"));
  inputdata->params_.push_back(invalid);
  auto out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);

  // 2 pass through output
  std::cout << "------------test pass through output----------" << std::endl;
  inputdata->params_.clear();
  HobotXRoc::InputParamPtr
      pass_through(new HobotXRoc::DisableParam("BBoxFilter_1", HobotXRoc::DisableParam::Mode::PassThrough));
  inputdata->params_.push_back(pass_through);
  out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);

  // 3 use pre-defined input data
  std::cout << "------------test pre-defined output----------" << std::endl;
  inputdata->params_.clear();
  HobotXRoc::DisableParamPtr
      pre_define(new HobotXRoc::DisableParam("BBoxFilter_1", HobotXRoc::DisableParam::Mode::UsePreDefine));
  BaseDataVector *pre_data(new BaseDataVector);
  HobotXRoc::BBox *pre_bbox1(new HobotXRoc::BBox(
    hobot::vision::BBox(0, 0, 20, 20)));
  pre_bbox1->type_ = "BBox";
  pre_data->name_ = "face_head_box";
  pre_data->datas_.push_back(BaseDataPtr(pre_bbox1));

  pre_define->pre_datas_.emplace_back(BaseDataPtr(pre_data));

  inputdata->params_.push_back(pre_define);
  out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);

  delete flow;
  // 初始化sdk
  return 0;
}
