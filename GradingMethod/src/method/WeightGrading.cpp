/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     WeightGrading Method
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

#include "GradingMethod/WeightGrading.h"
#include "json/json.h"
#include "GradingMethod/error_code.h"
#include "hobotlog/hobotlog.hpp"


namespace HobotXRoc {

int WeightGradingParam::UpdateParameter(const std::string &content) {
  int ret = GradingParam::UpdateParameter(content);
  if (XROC_GRADING_OK != ret) {
    return ret;
  }
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
  try {
    ret = json_reader->parse(content.c_str(), content.c_str()
        + content.size(), &config_jv, &error);
    SET_GRADING_METHOD_PARAM(config_jv, Double, size_min);
    SET_GRADING_METHOD_PARAM(config_jv, Double, size_max);
    SET_GRADING_METHOD_PARAM(config_jv, Double, size_inflexion);
    SET_GRADING_METHOD_PARAM(config_jv, Double, frontal_thr);
    SET_GRADING_METHOD_PARAM(config_jv, Double, size_weight);
    SET_GRADING_METHOD_PARAM(config_jv, Double, pose_weight);
    SET_GRADING_METHOD_PARAM(config_jv, Double, lmk_weight);
    SET_GRADING_METHOD_PARAM(config_jv, Double, quality_weight);
    SET_GRADING_METHOD_PARAM(config_jv, Double, normalize_lmk_divisor);
    if (ret) {
      return XROC_GRADING_OK;
    } else {
      return XROC_GRADING_ERR_PARAM;
    }
  } catch (std::exception &e) {
    return XROC_GRADING_ERR_PARAM;
  }
}

int WeightGrading::GradingInit(const std::string &config_file_path) {
  LOGI << "GradingMethod::GradingInit " << config_file_path << std::endl;
  std::ifstream config_if(config_file_path);
  if (!config_if.good()) {
    LOGI << "WeightGradingParam: no config, using default parameters" << std::endl;
  }
  std::ostringstream buf;
  char ch;
  while (buf && config_if.get(ch)) {
    buf.put(ch);
  }
  config_param_ = std::make_shared<WeightGradingParam>(buf.str());
  return XROC_GRADING_OK;
}

int WeightGrading::ProcessFrame(const std::vector<BaseDataPtr> &in,
                                const InputParamPtr &param,
                                std::vector<BaseDataPtr> &out) {
  if (param) {
    if (param->is_json_format_) {
      auto content = param->Format();
      int ret = UpdateParameter(content);
      if (ret != XROC_GRADING_OK)
        return ret;
    }
  }
  HOBOT_CHECK(in.size() >= 3);
  auto box_list = std::static_pointer_cast<BaseDataVector>(in[0]);
  auto pose_3d_list = std::static_pointer_cast<BaseDataVector>(in[1]);
  auto land_mark_list = std::static_pointer_cast<BaseDataVector>(in[2]);

  assert("BaseDataVector" == box_list->type_);
  assert("BaseDataVector" == pose_3d_list->type_);
  assert("BaseDataVector" == land_mark_list->type_);

  size_t item_size = box_list->datas_.size();
  std::shared_ptr<BaseDataVector> quality_list;
  if (in.size() == 4) {
    quality_list = std::static_pointer_cast<BaseDataVector>(in[3]);
    assert("BaseDataVector" == quality_list->type_);
    assert(item_size == pose_3d_list->datas_.size()
               && item_size == land_mark_list->datas_.size()
               && item_size == quality_list->datas_.size());
  } else {
    assert(item_size == pose_3d_list->datas_.size()
               && item_size == land_mark_list->datas_.size());
  }

  auto scores = std::make_shared<BaseDataVector>();
  out.push_back(std::static_pointer_cast<BaseData>(scores));

  for (size_t i = 0; i < box_list->datas_.size(); i++) {
    std::shared_ptr<XRocBBox> bbox;
    std::shared_ptr<XRocPose3D> pose3d;
    std::shared_ptr<XRocLandmarks> lmk;
    if (box_list->state_ == DataState::INVALID) {
      bbox = std::make_shared<XRocBBox>();
    } else {
      bbox = std::static_pointer_cast<XRocBBox>(box_list->datas_[i]);
    }
    if (pose_3d_list->state_ == DataState::INVALID) {
      pose3d = std::make_shared<XRocPose3D>();
    } else {
      pose3d = std::static_pointer_cast<XRocPose3D>(pose_3d_list->datas_[i]);
    }
    if (land_mark_list->state_ == DataState::INVALID) {
      lmk = std::make_shared<XRocLandmarks>();
    } else {
      lmk = std::static_pointer_cast<XRocLandmarks>(land_mark_list->datas_[i]);
    }

    std::shared_ptr<XRocFloat> score(new XRocFloat());
    score->type_ = "Number";

    if (in.size() == 4) {
      std::shared_ptr<XRocQuality> quality;
      if (quality_list->state_ == DataState::INVALID) {
        quality = std::make_shared<XRocQuality>();
      } else {
        quality = std::static_pointer_cast<XRocQuality>(quality_list->datas_[i]);
      }
      score->value = config_param_->size_weight * ProcessRect(*bbox)
          + config_param_->pose_weight * ProcessPose3d(*pose3d)
          + config_param_->lmk_weight * ProcessLmk(*lmk)
          + config_param_->quality_weight * ProcessQuality(*quality);
    } else {
      score->value = config_param_->size_weight * ProcessRect(*bbox)
          + config_param_->pose_weight * ProcessPose3d(*pose3d)
          + config_param_->lmk_weight * ProcessLmk(*lmk);
    }

    LOGI << "score:" << score->value;
    scores->datas_.push_back(score);
  }
  return XROC_GRADING_OK;
}

float WeightGrading::ProcessRect(XRocBBox &bbox) {
  auto w = bbox.value.Width();
  auto h = bbox.value.Height();
  float size = std::min(w, h);
  float normalized_size = 0;
  if (size < config_param_->size_min) {
    normalized_size = 0;
  } else if (size > config_param_->size_max) {
    normalized_size = 1;
  } else {
    if (size > config_param_->size_inflexion)
      normalized_size =
          (size - config_param_->size_inflexion)
              / (config_param_->size_max
              - config_param_->size_inflexion) * 0.5f + 0.5f;
    else
      normalized_size =
          (size - config_param_->size_min)
              / (config_param_->size_inflexion
              - config_param_->size_min) * 0.5f;
  }
  return normalized_size;
}

float WeightGrading::ProcessPose3d(XRocPose3D &pose3d) {
  auto &pitch = pose3d.value.pitch;
  auto &yaw = pose3d.value.yaw;
  auto &roll = pose3d.value.roll;
  if (yaw < -90) { yaw = -90; }
  if (yaw > 90) { yaw = 90; }
  if (pitch < -90) { pitch = -90; }
  if (pitch > 90) { pitch = 90; }
  float pos_frontal = (2000 - (yaw * yaw / 16 + pitch * pitch / 9) * 10);
  if (pos_frontal < -1999) {
    pos_frontal = -1999;
  } else if (pos_frontal > 2000) {
    pos_frontal = 2000;
  }
  return (pos_frontal - config_param_->frontal_thr)
      / (2000 - config_param_->frontal_thr);
}

float WeightGrading::ProcessLmk(XRocLandmarks &lmk) {
  float normalized_lmk = 0;
  for (auto &point : lmk.value.values) {
    normalized_lmk += point.score;
  }
  normalized_lmk /= config_param_->normalize_lmk_divisor;
  if (normalized_lmk < 0)
    normalized_lmk = 0;
  if (normalized_lmk > 1)
    normalized_lmk = 1;
  return normalized_lmk;
}

float WeightGrading::ProcessQuality(XRocQuality &quality) {
  float normalized_blur = 0;
  if (quality.value.value < 0)
      normalized_blur = 0;
  else if (quality.value.value > 1)
    normalized_blur = 1;
  else
    normalized_blur = quality.value.value;
  return normalized_blur;
}

void WeightGrading::GradingFinalize() {}

InputParamPtr WeightGrading::GetParameter() {
  return config_param_;
}

int WeightGrading::UpdateParameter(const std::string &content) {
  return config_param_->UpdateParameter(content);
}
}  // namespace HobotXRoc


