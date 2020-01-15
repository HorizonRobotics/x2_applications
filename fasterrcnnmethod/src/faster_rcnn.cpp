/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     FasterRCNN Method
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <stdint.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <fstream>

#include "hobotxsdk/xroc_data.h"
#include "hobotlog/hobotlog.hpp"

#include "FasterRCNNMethod/FasterRCNNMethod.h"
#include "faster_rcnn_imp.h"
#include "FasterRCNNMethod/result.h"


namespace HobotXRoc {

int FasterRCNNMethod::Init(const std::string &config_file) {
  LOGD << "Init FasterRCNNMethod";
  faster_rcnn_imp_.reset(new faster_rcnn_method::FasterRCNNImp());
  int ret = faster_rcnn_imp_->Init(config_file);
  HOBOT_CHECK(ret == 0) << "faster_rcnn_method init failed.";
  method_param_.reset(new FasterRCNNParam("FasterRCNNMethod"));
  LOGD << "Finish FasterRCNNMethod Init";
  return 0;
}

int FasterRCNNMethod::UpdateParameter(InputParamPtr ptr) {
  faster_rcnn_imp_->UpdateParameter(ptr);
  return 0;
}

InputParamPtr FasterRCNNMethod::GetParameter() const {
  return method_param_;
}

std::string FasterRCNNMethod::GetVersion() const {
  return faster_rcnn_imp_->GetVersion();
}

std::vector<std::vector<BaseDataPtr>>
FasterRCNNMethod::DoProcess(const std::vector<std::vector<BaseDataPtr>> &input,
                            const std::vector<InputParamPtr> &param) {
  LOGD << "Run FasterRCNNMethod";
  LOGD << "input's size: " << input.size();
  std::vector<std::vector<BaseDataPtr>> output;
  output.resize(input.size());
  // input size > 0 -> many frames， batch mode
  for (size_t i = 0; i < input.size(); ++i) {
    const auto &frame_input = input[i];
    auto &frame_output = output[i];
    faster_rcnn_imp_->RunSingleFrame(frame_input, frame_output);
  }
  return output;
}



void FasterRCNNMethod::Finalize() {
  faster_rcnn_imp_->Finalize();
}

} // namespace HobotXRoc
