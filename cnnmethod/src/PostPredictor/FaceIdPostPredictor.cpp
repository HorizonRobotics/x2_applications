/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: FaceIdPostPredictor.cpp
 * @Brief: definition of the FaceIdPostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 15:18:10
 */

#include "CNNMethod/PostPredictor/FaceIdPostPredictor.h"
#include <vector>
#include "CNNMethod/CNNConst.h"
#include "CNNMethod/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/profiler.h"

namespace HobotXRoc {

void FaceIdPostPredictor::Do(CNNMethodRunData *run_data) {
  int batch_size = run_data->input_dim_size.size();
  run_data->output.resize(batch_size);
  for (int batch_idx = 0; batch_idx < batch_size; batch_idx++) {
    auto &input_data = (*(run_data->input))[batch_idx];
    auto snaps = std::static_pointer_cast<BaseDataVector>(input_data[0]);
    int person_num = snaps->datas_.size();
    int total_snap = run_data->input_dim_size[batch_idx];
    auto &mxnet_output = run_data->mxnet_output[batch_idx];

    std::vector<BaseDataPtr> &batch_output = run_data->output[batch_idx];
    batch_output.resize(output_slot_size_);
    for (int i = 0; i < output_slot_size_; i++) {
      auto base_data_vector = std::make_shared<BaseDataVector>();
      batch_output[i] = std::static_pointer_cast<BaseData>(base_data_vector);
    }

    auto data_vector =
        std::static_pointer_cast<BaseDataVector>(batch_output[0]);
    for (uint32_t person_idx = 0, g_snap_idx = 0; person_idx < person_num;
         person_idx++) {
      auto face_features = std::make_shared<BaseDataVector>();  // one person
      auto one_person_snaps =
          dynamic_cast<BaseDataVector *>(snaps->datas_[person_idx].get());
      if (!one_person_snaps) {
        continue;
      }
      for (uint32_t snap_idx = 0; snap_idx < one_person_snaps->datas_.size()
                                  && g_snap_idx < total_snap; snap_idx++) {
        RUN_PROCESS_TIME_PROFILER(model_name_ + "_post");
        RUN_FPS_PROFILER(model_name_ + "_post");
        auto face_feature = FaceFeaturePostPro(mxnet_output[g_snap_idx++]);
        face_features->datas_.push_back(face_feature);
      }
      data_vector->datas_.push_back(face_features);
    }
  }
}

BaseDataPtr FaceIdPostPredictor::FaceFeaturePostPro(
    const std::vector<std::vector<int8_t>> &mxnet_outs) {
  if (mxnet_outs.size() == 0 || mxnet_outs[0].size() == 0) {
    auto feature_invalid = std::make_shared<XRocData<hobot::vision::Feature>>();
    feature_invalid->state_ = DataState::INVALID;
    return std::static_pointer_cast<BaseData>(feature_invalid);
  }
  static const int kFeatureCnt = 128;
  auto mxnet_rlt = reinterpret_cast<const float *>(mxnet_outs[0].data());

  auto feature = std::make_shared<XRocData<hobot::vision::Feature>>();
  feature->value.values.resize(kFeatureCnt);
  for (int i = 0; i < kFeatureCnt; i++) {
    feature->value.values[i] = mxnet_rlt[i];
  }

  l2_norm(feature->value.values, kFeatureCnt);
  return std::static_pointer_cast<BaseData>(feature);
}

}  // namespace HobotXRoc
