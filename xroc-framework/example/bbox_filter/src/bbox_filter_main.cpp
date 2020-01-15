/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Method interface of xsoul framework
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include "hobotxroc/data_types/bbox.h"
#include "hobotxroc/data_types/filter_param.h"
#include "hobotxroc/xroc_config.h"
#include "hobotxsdk/xroc_error.h"
#include "hobotxsdk/xroc_sdk.h"

clock_t begin_clock = 0;
class Callback {
 public:
  Callback() {}

  ~Callback() {}

  void OnCallback(HobotXRoc::OutputDataPtr output) {
    using HobotXRoc::BaseDataVector;
    if ((output->sequence_id_ == 99999) || (output->sequence_id_ % 1000 == 0)) {
      auto duration = clock() - begin_clock;
      std::cout << "seq: " << output->sequence_id_ << " duration:" << duration << std::endl;
    }
    std::cout << "======================" << std::endl;
    std::cout << "seq: " << output->sequence_id_ << std::endl;
    std::cout << "output_type: " << output->output_type_ << std::endl;
    std::cout << "method_name: " << output->method_name_ << std::endl;
    std::cout << "error_code: " << output->error_code_ << std::endl;
    std::cout << "error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "datas_ size: " << output->datas_.size() << std::endl;
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

int main(int argc, char const *argv[]) {
  using HobotXRoc::BaseData;
  using HobotXRoc::BaseDataPtr;
  using HobotXRoc::BaseDataVector;
  using HobotXRoc::InputData;
  using HobotXRoc::InputDataPtr;
  if (argc < 2) {
    printf("Usage : ./bbox_filter_main config (multipuloutput_flag)\n");
    printf("Example : ./bbox_filter_main ./config/filter.json\n");
    printf("Example : ./bbox_filter_main ./config/filter_multioutput.json 1\n");
    return -1;
  }
  auto config = argv[1];
  HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", config);
  flow->Init();
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1),
      "BBoxFilter_1");
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1),
      "BBoxFilter_2");
  /// Get Method Version
  std::cout << "BBoxFilter_1 Method Version : " << flow->GetVersion("BBoxFilter_1") << std::endl;

  InputDataPtr inputdata(new InputData());
  BaseDataVector *data(new BaseDataVector);

  HobotXRoc::BBox *bbox1(new HobotXRoc::BBox(
    hobot::vision::BBox(0, 0, 1000, 1000)));
  HobotXRoc::BBox *bbox2(new HobotXRoc::BBox(
    hobot::vision::BBox(0, 0, 10, 10)));
  bbox1->type_ = "BBox";
  bbox2->type_ = "BBox";
  std::cout << "bbox1: " << bbox1->value << std::endl;
  std::cout << "bbox2: " << bbox2->value << std::endl;

  data->datas_.push_back(BaseDataPtr(bbox1));
  data->datas_.push_back(BaseDataPtr(bbox2));
  data->name_ = "face_head_box";
  inputdata->datas_.push_back(BaseDataPtr(data));
  begin_clock = clock();
  if (argc == 2) {
  std::cout << "***********************" << std::endl
            << "testing synchronous function" << std::endl
            << "***********************" << std::endl;
  for (int i = 0; i < 10; i++) {
    auto out = flow->SyncPredict(inputdata);
    callback.OnCallback(out);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (i == 5) {
      std::string method_name("BBoxFilter_1");
      auto ptr = std::make_shared<HobotXRoc::FilterParam>(method_name);
      ptr->SetThreshold(90.0);
      flow->UpdateConfig(ptr->method_name_, ptr);
    }
  }
  }
  std::cout << "***********************" << std::endl
            << "testing multi_output synchronous function" << std::endl
            << "***********************" << std::endl;
  for (int i = 0; i < 10; i++) {
    auto out = flow->SyncPredict2(inputdata);
    for (auto single_out : out) {
      callback.OnCallback(single_out);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (i == 5) {
      std::string method_name("BBoxFilter_1");
      auto ptr = std::make_shared<HobotXRoc::FilterParam>(method_name);
      ptr->SetThreshold(90.0);
      flow->UpdateConfig(ptr->method_name_, ptr);
    }
  }
  std::cout << "***********************" << std::endl
            << "testing aysnc function" << std::endl
            << "***********************" << std::endl;
  for (int i = 0; i < 10; i++) {
    flow->AsyncPredict(inputdata);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (i == 5) {
      std::string method_name("BBoxFilter_1");
      auto ptr = std::make_shared<HobotXRoc::FilterParam>(method_name);
      ptr->SetThreshold(90.0);
      flow->UpdateConfig(ptr->method_name_, ptr);
    }
  }
  auto method_config = flow->GetConfig("BBoxFilter_1");
  if (method_config) {
    auto real_ptr = dynamic_cast<HobotXRoc::FilterParam *>(method_config.get());
    std::cout << "threshold:" << real_ptr->GetThreshold() << std::endl;
  }
  // waiting for async function done
  std::this_thread::sleep_for(std::chrono::seconds(1));
  delete flow;
  // 初始化sdk
  return 0;
}
