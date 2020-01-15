/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Grading Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.25
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <cassert>

#include "GradingMethod/GradingMethod.h"
#include "GradingMethod/WeightGrading.h"
#include "json/json.h"
#include "GradingMethod/error_code.h"
#include "hobotlog/hobotlog.hpp"


namespace HobotXRoc {

int GradingMethod::Init(const std::string& config_file_path) {
  LOGI << "GradingMethod::Init " << config_file_path << std::endl;
  std::ifstream config_if(config_file_path);
  if (!config_if.good()) {
    LOGI << "SnapShotParam: no config, using default parameters" << std::endl;
  } else {
    Json::Value config_jv;
    config_if >> config_jv;
    std::string grading_type;
    if (config_jv.isMember("grading_type") &&
        config_jv["grading_type"].isString())
      grading_type = config_jv["grading_type"].asString();
    if (grading_type != "weight_grading") {
      LOGE << "config param error";
      return XROC_GRADING_ERR_PARAM;
    }
  }
  grading_ = std::make_shared<WeightGrading>();
  grading_->GradingInit(config_file_path);

  return XROC_GRADING_OK;
}

std::vector<std::vector<BaseDataPtr>> GradingMethod::DoProcess(
    const std::vector<std::vector<BaseDataPtr>>& input,
    const std::vector<InputParamPtr>& param) {

  LOGI << "GradingMethod::DoProcess" << std::endl;
  std::vector<std::vector<BaseDataPtr>> output;
  output.resize(input.size());

  for (size_t i = 0; i < input.size(); ++i) {
    auto& in_batch_i = input[i];
    auto& out_batch_i = output[i];
    auto& param_batch_i = param[i];
    grading_->ProcessFrame(in_batch_i, param_batch_i, out_batch_i);
  }
  return output;
}

int GradingMethod::UpdateParameter(InputParamPtr ptr) {
  if (ptr->is_json_format_) {
    std::string content = ptr->Format();
    return grading_->UpdateParameter(content);
  } else {
    HOBOT_CHECK(0) << "only support json format config";
    return XROC_GRADING_ERR_PARAM;
  }
}

InputParamPtr GradingMethod::GetParameter() const {
  return grading_->GetParameter();
}

std::string GradingMethod::GetVersion() const {
  return "0.0.12";
}

void GradingMethod::Finalize() {
  grading_->GradingFinalize();
  LOGI << "GradingMethod::GFinalize" << std::endl;
}


int GradingParam::UpdateParameter(const std::string &content) {
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
  try {
    bool ret = json_reader->parse(content.c_str(), content.c_str()
        + content.size(), &config_jv, &error);
    SET_GRADING_METHOD_PARAM(config_jv, String, grading_type);
    if (ret) {
      return XROC_GRADING_OK;
    } else {
      return XROC_GRADING_ERR_PARAM;
    }
  } catch (std::exception &e) {
    return XROC_GRADING_ERR_PARAM;
  }
}

std::string GradingParam::Format() {
  return config_jv.toStyledString();
}
}  // namespace HobotXRoc
