/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: FaceQualityPostPredictor.cpp
 * @Brief: definition of the FaceQualityPostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-09-26 21:38:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-09-26 21:38:28
 */

#include "CNNMethod/PostPredictor/FaceQualityPostPredictor.h"
#include <memory>
#include <vector>
#include "CNNMethod/CNNConst.h"
#include "CNNMethod/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/profiler.h"

namespace HobotXRoc {

int32_t
FaceQualityPostPredictor::Init(std::shared_ptr<CNNMethodConfig> config) {
  PostPredictor::Init(config);
  threshold_ = config->GetFloatValue("threshold");
  return 0;
}

void FaceQualityPostPredictor::UpdateParam(
    std::shared_ptr<CNNMethodConfig> config) {
  PostPredictor::UpdateParam(config);
  if (config->KeyExist("threshold")) {
    threshold_ = config->GetFloatValue("threshold");
  }
}

void FaceQualityPostPredictor::Do(CNNMethodRunData *run_data) {
  int batch_size = run_data->input_dim_size.size();
  run_data->output.resize(batch_size);
  for (int batch_idx = 0; batch_idx < batch_size; batch_idx++) {
    int dim_size = run_data->input_dim_size[batch_idx];
    auto &mxnet_output = run_data->mxnet_output[batch_idx];
    std::vector<BaseDataPtr> &batch_output = run_data->output[batch_idx];
    batch_output.resize(output_slot_size_);
    for (int i = 0; i < output_slot_size_; i++) {
      auto base_data_vector = std::make_shared<BaseDataVector>();
      batch_output[i] = std::static_pointer_cast<BaseData>(base_data_vector);
    }
    {
      RUN_PROCESS_TIME_PROFILER(model_name_ + "_post");
      RUN_FPS_PROFILER(model_name_ + "_post");
      for (int dim_idx = 0; dim_idx < dim_size; dim_idx++) {  // loop target
        std::vector<BaseDataPtr> output;
        FaceQualityPostPro(mxnet_output[dim_idx], &output);
        for (int slot_idx = 0; slot_idx < output_slot_size_; slot_idx++) {
          auto base_data_vector =
              std::static_pointer_cast<BaseDataVector>(batch_output[slot_idx]);
          base_data_vector->datas_.push_back(output[slot_idx]);
        }
      }
    }
  }
}
// blur bright eye_emot mouth_emot leye reye lbrow rbrow fhead lcheek rcheek
// nose mouth jaw
void FaceQualityPostPredictor::FaceQualityPostPro(
    const std::vector<std::vector<int8_t>> &mxnet_outs,
    std::vector<BaseDataPtr> *output) {
  output->resize(output_slot_size_);
  // layer 1
  for (int slot_idx = 0; slot_idx < output_slot_size_ - 1; slot_idx++) {
    auto attribute =
        std::make_shared<XRocData<hobot::vision::Attribute<int>>>();
    if (mxnet_outs.size() == 0 || mxnet_outs[0].size() == 0) {
      attribute->value.value = -1;
      attribute->value.score = 1.0f;
      attribute->state_ = DataState::INVALID;
    } else {
      auto mxnet_out = reinterpret_cast<const float *>(mxnet_outs[0].data());
      attribute->value.score = SigMoid(mxnet_out[slot_idx]);
      attribute->value.value = attribute->value.score < threshold_ ? 0 : 1;
    }
    int out_idx = slot_idx == 0 ? slot_idx : slot_idx + 1;
    (*output)[out_idx] = std::static_pointer_cast<BaseData>(attribute);
  }

  // layer 2
  auto attribute = std::make_shared<XRocData<hobot::vision::Attribute<int>>>();
  if (mxnet_outs.size() == 0 || mxnet_outs[1].size() == 0) {
    attribute->value.value = -1;
    attribute->value.score = 1.0f;
    attribute->state_ = DataState::INVALID;
  } else {
    auto mxnet_out = reinterpret_cast<const float *>(mxnet_outs[1].data());
    std::vector<float> attrs = {
        mxnet_out[0], mxnet_out[1], mxnet_out[2], mxnet_out[3]};
    SoftMax(attrs);
    auto largest = std::max_element(std::begin(attrs), std::end(attrs));
    auto pos = std::distance(std::begin(attrs), largest);
    attribute->value.value = pos;
    attribute->value.score = attrs[pos];
  }
  (*output)[1] = std::static_pointer_cast<BaseData>(attribute);
}

}  // namespace HobotXRoc
