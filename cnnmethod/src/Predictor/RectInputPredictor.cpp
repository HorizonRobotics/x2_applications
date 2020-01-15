/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: RectInputPredictor.cpp
 * @Brief: definition of the RectInputPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-16 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-16 16:23:27
 */

#include "CNNMethod/Predictor/RectInputPredictor.h"
#include <algorithm>
#include <memory>
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/profiler.h"
#include "horizon/vision_type/vision_type.hpp"
#include "horizon/vision_type/vision_type_common.h"

using hobot::vision::BBox;
using hobot::vision::ImageFrame;
using hobot::vision::PymImageFrame;
typedef std::shared_ptr<ImageFrame> ImageFramePtr;

namespace HobotXRoc {

void RectInputPredictor::Do(CNNMethodRunData *run_data) {
  int frame_size = run_data->input->size();
  run_data->mxnet_output.resize(frame_size);
  run_data->input_dim_size.resize(frame_size);
  run_data->norm_rois.resize(frame_size);
  run_data->real_nhwc = model_info_.real_nhwc_;
  run_data->elem_size = model_info_.elem_size_;

  for (int frame_idx = 0; frame_idx < frame_size; frame_idx++) {  // loop frame
    auto &input_data = (*(run_data->input))[frame_idx];

    int ret = 0;
    auto rois = std::static_pointer_cast<BaseDataVector>(input_data[0]);
    auto xroc_pyramid =
        std::static_pointer_cast<XRocData<ImageFramePtr>>(input_data[1]);
    auto pyramid = std::static_pointer_cast<PymImageFrame>(xroc_pyramid->value);

    int box_num = rois->datas_.size();
    run_data->input_dim_size[frame_idx] = box_num;
    std::vector<int> valid_box(box_num, 1);
    run_data->mxnet_output[frame_idx].resize(box_num);
    run_data->norm_rois[frame_idx].resize(box_num);

    auto &norm_rois = run_data->norm_rois[frame_idx];

    std::vector<BPUBBox> boxes;
    uint32_t handle_num =
        max_handle_num_ < 0 ? box_num : std::min(max_handle_num_, box_num);
    for (uint32_t roi_idx = 0; roi_idx < box_num; roi_idx++) {
      auto &roi = rois->datas_[roi_idx];
      auto p_roi = std::static_pointer_cast<XRocData<BBox>>(roi);
      auto p_norm_roi = std::make_shared<XRocData<BBox>>();
      norm_rois[roi_idx] = std::static_pointer_cast<BaseData>(p_norm_roi);
      p_norm_roi->value = p_roi->value;
      if (p_roi->state_ != HobotXRoc::DataState::VALID
          || roi_idx >= handle_num) {
        valid_box[roi_idx] = 0;
      } else {
        boxes.push_back(BPUBBox{p_roi->value.x1,
                                p_roi->value.y1,
                                p_roi->value.x2,
                                p_roi->value.y2,
                                p_roi->value.score,
                                0,
                                true});
        LOGD << "box {" << p_roi->value.x1 << "," << p_roi->value.y1 << ","
             << p_roi->value.x2 << "," << p_roi->value.y2 << "}";
      }
    }
    ModelOutputBuffer bufs(model_info_, boxes.size());
    {
      RUN_PROCESS_TIME_PROFILER(model_name_ + "_runmodel")
      RUN_FPS_PROFILER(model_name_ + "_runmodel")
      int resizable_cnt = 0;
      ret = RunModelFromResizer(
          reinterpret_cast<BPUPyramidBuffer>(&(pyramid->img)),
          boxes.data(),
          boxes.size(),
          &resizable_cnt,
          bufs.out_bufs_.data(),
          boxes.size() * model_info_.output_layer_size_.size());
      if (ret == -1) {
        return;
      }
      for (uint32_t i = 0, bpu_box_idx = 0; i < box_num; i++) {
        if (valid_box[i]) {
          valid_box[i] = boxes[bpu_box_idx].resizable;
          if (valid_box[i]) {
            auto p_norm_roi =
                std::static_pointer_cast<XRocData<BBox>>(norm_rois[i]);
            p_norm_roi->value.x1 = boxes[bpu_box_idx].x1;
            p_norm_roi->value.y1 = boxes[bpu_box_idx].y1;
            p_norm_roi->value.x2 = boxes[bpu_box_idx].x2;
            p_norm_roi->value.y2 = boxes[bpu_box_idx].y2;
          }
          bpu_box_idx++;
        }
      }
    }

    {
      RUN_PROCESS_TIME_PROFILER(model_name_ + "_do_hbrt")
      RUN_FPS_PROFILER(model_name_ + "_do_hbrt")
      // change raw data to mxnet layout
      int layer_size = model_info_.output_layer_size_.size();
      for (uint32_t i = 0, mxnet_rlt_idx = 0; i < box_num; i++) {
        if (valid_box[i]) {
          auto &mxnet_rlt = run_data->mxnet_output[frame_idx][i];
          mxnet_rlt.resize(layer_size);
          for (int j = 0; j < layer_size; j++) {
            int raw_rlt_idx = mxnet_rlt_idx * layer_size + j;
            mxnet_rlt[j].resize(model_info_.mxnet_output_layer_size_[j]);
            hbrtConvertLayout(feature_bufs_[j].data(), LAYOUT_NHWC_NATIVE,
                              BPU_getRawBufferPtr(bufs.out_bufs_[raw_rlt_idx]),
                              model_info_.layout_flag_[j],
                              model_info_.element_type_[j],
                              model_info_.align_dim_[j],
                              model_info_.convert_endianness_[j]);
            ConvertOutputToMXNet(
                feature_bufs_[j].data(), mxnet_rlt[j].data(), j);
          }
          mxnet_rlt_idx++;
        }
      }
    }
  }
}
}  // namespace HobotXRoc
