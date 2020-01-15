/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: AntiSpfPostPredictor.cpp
 * @Brief: definition of the AntiSpfPostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 15:18:10
 */

#include "CNNMethod/PostPredictor/AntiSpfPostPredictor.h"
#include <vector>
#include "CNNMethod/CNNConst.h"
#include "CNNMethod/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/profiler.h"

namespace HobotXRoc {

int32_t AntiSpfPostPredictor::Init(std::shared_ptr<CNNMethodConfig> config) {
  PostPredictor::Init(config);
  anti_spf_threshold_ = config->GetFloatValue("threshold");
  return 0;
}

void AntiSpfPostPredictor::UpdateParam(
    std::shared_ptr<CNNMethodConfig> config) {
  PostPredictor::UpdateParam(config);
  if (config->KeyExist("threshold")) {
    anti_spf_threshold_ = config->GetFloatValue("threshold");
  }
}

void AntiSpfPostPredictor::Do(CNNMethodRunData *run_data) {
  int batch_size = run_data->input_dim_size.size();
  run_data->output.resize(batch_size);
  for (int batch_idx = 0; batch_idx < batch_size; batch_idx++) {
    auto &norm_rois = run_data->norm_rois[batch_idx];
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
      auto data_vector =
          std::static_pointer_cast<BaseDataVector>(batch_output[0]);
      auto norm_vector =
          std::static_pointer_cast<BaseDataVector>(batch_output[1]);
      for (int dim_idx = 0; dim_idx < dim_size; dim_idx++) {  // loop target
        BaseDataPtr anti_spf = FaceAntiSpfPostPro(mxnet_output[dim_idx],
                                                  run_data->real_nhwc[0][3]);
        data_vector->datas_.push_back(anti_spf);
        norm_vector->datas_.push_back(norm_rois[dim_idx]);
      }
    }
  }
}

BaseDataPtr AntiSpfPostPredictor::FaceAntiSpfPostPro(
    const std::vector<std::vector<int8_t>> &mxnet_outs, int channel_size) {
  auto anti_spf = std::make_shared<XRocData<hobot::vision::Attribute<int>>>();
  if (mxnet_outs.size() == 0 || mxnet_outs[0].size() == 0) {
    anti_spf->value.value = -1;
    anti_spf->value.score = 1.0f;
    anti_spf->state_ = DataState::INVALID;
  } else {
    auto mxnet_out = reinterpret_cast<const float *>(mxnet_outs[0].data());
    if (channel_size == 1) {
      anti_spf->value.score = SigMoid(mxnet_out[0]);
      anti_spf->value.value =
          anti_spf->value.score > anti_spf_threshold_ ? 1 : 0;
    } else {
      std::vector<float> attrs = {mxnet_out[0], mxnet_out[1]};
      SoftMax(attrs);
      anti_spf->value.score = attrs[1];
      anti_spf->value.value =
          anti_spf->value.score > anti_spf_threshold_ ? 1 : 0;
    }
  }
  return std::static_pointer_cast<BaseData>(anti_spf);
}

}  // namespace HobotXRoc
