/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: LmkPosePostPredictor.cpp
 * @Brief: definition of the LmkPosePostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 15:18:10
 */

#include <vector>
#include "CNNMethod/PostPredictor/LmkPosePostPredictor.h"
#include "CNNMethod/CNNConst.h"
#include "CNNMethod/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/profiler.h"

namespace HobotXRoc {

void LmkPosePostPredictor::Do(CNNMethodRunData *run_data) {
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
      auto boxes = std::static_pointer_cast<BaseDataVector>(
          (*(run_data->input))[batch_idx][0]);

      for (int dim_idx = 0; dim_idx < dim_size; dim_idx++) {
        std::vector<BaseDataPtr> output;
        auto xroc_box = std::static_pointer_cast<XRocData<hobot::vision::BBox>>(
            boxes->datas_[dim_idx]);
        HandleLmkPose(mxnet_output[dim_idx],
                      xroc_box->value,
                      run_data->real_nhwc,
                      &output);

        for (int i = 0; i < output_slot_size_; i++) {
          auto base_data_vector =
              std::static_pointer_cast<BaseDataVector>(batch_output[i]);
          base_data_vector->datas_.push_back(output[i]);
        }
      }
    }
  }
}

void LmkPosePostPredictor::HandleLmkPose(
    const std::vector<std::vector<int8_t>> &mxnet_outs,
    const hobot::vision::BBox &box,
    const std::vector<std::vector<uint32_t>> &nhwc,
    std::vector<BaseDataPtr> *output) {
  if (mxnet_outs.size()) {
    auto lmk = LmkPostPro(mxnet_outs, box, nhwc);
    output->push_back(lmk);
    if (mxnet_outs.size() > 3) {
      auto pose = PosePostPro(mxnet_outs[3]);
      output->push_back(pose);
    } else {
      auto pose = std::make_shared<XRocData<hobot::vision::Pose3D>>();
      pose->state_ = DataState::INVALID;
      output->push_back(std::static_pointer_cast<BaseData>(pose));
    }
  } else {
    auto landmarks = std::make_shared<XRocData<hobot::vision::Landmarks>>();
    landmarks->state_ = DataState::INVALID;
    output->push_back(std::static_pointer_cast<BaseData>(landmarks));
    auto pose = std::make_shared<XRocData<hobot::vision::Pose3D>>();
    pose->state_ = DataState::INVALID;
    output->push_back(std::static_pointer_cast<BaseData>(pose));
  }
}

BaseDataPtr LmkPosePostPredictor::LmkPostPro(
    const std::vector<std::vector<int8_t>> &mxnet_outs,
    const hobot::vision::BBox &box,
    const std::vector<std::vector<uint32_t>> &nhwc) {
  static const float SCORE_THRESH = 0.0;
  static const float REGRESSION_RADIUS = 3.0;
  static const float STRIDE = 4.0;
  static const float num = 1;
  static const float height_m = 16;
  static const float width_m = 16;

  auto fl_scores = reinterpret_cast<const float *>(mxnet_outs[0].data());
  auto fl_coords = reinterpret_cast<const float *>(mxnet_outs[1].data());
  std::vector<std::vector<float>> points_score;
  std::vector<std::vector<float>> points_x;
  std::vector<std::vector<float>> points_y;
  points_score.resize(5);
  points_x.resize(5);
  points_y.resize(5);

  // nhwc, 1x16x16x5, 1x16x16x10
  for (int n = 0; n < num; ++n) {          // n
    for (int i = 0; i < height_m; ++i) {   // h
      for (int j = 0; j < width_m; ++j) {  // w
        int index_score = n * nhwc[0][1] * nhwc[0][2] * nhwc[0][3]
                          + i * nhwc[0][2] * nhwc[0][3] + j * nhwc[0][3];
        int index_coords = n * nhwc[1][1] * nhwc[1][2] * nhwc[0][3]
                           + i * nhwc[1][2] * nhwc[1][3] + j * nhwc[1][3];
        for (int k = 0; k < 5; ++k) {  // c
          auto score = fl_scores[index_score + k];
          if (score > SCORE_THRESH) {
            points_score[k].push_back(score);
            float x =
                (j + 0.5 - fl_coords[index_coords + 2 * k] * REGRESSION_RADIUS)
                * STRIDE;
            float y =
                (i + 0.5
                 - fl_coords[index_coords + 2 * k + 1] * REGRESSION_RADIUS)
                * STRIDE;
            x = std::min(std::max(x, 0.0f), width_m * STRIDE);
            y = std::min(std::max(y, 0.0f), height_m * STRIDE);
            points_x[k].push_back(x);
            points_y[k].push_back(y);
          }
        }
      }
    }
  }
  auto landmarks = std::make_shared<XRocData<hobot::vision::Landmarks>>();
  landmarks->value.values.resize(5);
  for (int i = 0; i < 5; ++i) {
    auto &poi = landmarks->value.values[i];
    poi.x = Mean(points_x[i]);
    poi.y = Mean(points_y[i]);
    poi.x = box.x1 + poi.x / 64 * (box.x2 - box.x1);
    poi.y = box.y1 + poi.y / 64 * (box.y2 - box.y1);
    poi.score = static_cast<float>(points_score[i].size());
    if (poi.score <= 0.000001 && mxnet_outs.size() > 2) {
      auto reg_coords = reinterpret_cast<const float *>(mxnet_outs[2].data());
      poi.x = box.x1 + reg_coords[i << 1] * (box.x2 - box.x1);
      poi.y = box.y1 + reg_coords[(i << 1) + 1] * (box.y2 - box.y1);
    }
  }
  return std::static_pointer_cast<BaseData>(landmarks);
}

BaseDataPtr
LmkPosePostPredictor::PosePostPro(const std::vector<int8_t> &mxnet_outs) {
  auto pose = std::make_shared<XRocData<hobot::vision::Pose3D>>();
  auto mxnet_out = reinterpret_cast<const float *>(mxnet_outs.data());
  pose->value.yaw = mxnet_out[0] * 90.0;
  pose->value.pitch = mxnet_out[1] * 90.0;
  pose->value.roll = mxnet_out[2] * 90.0;
  return std::static_pointer_cast<BaseData>(pose);
}
}  // namespace HobotXRoc
