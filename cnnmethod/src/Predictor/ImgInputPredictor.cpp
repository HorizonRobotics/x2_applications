/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: ImgInputPredictor.cpp
 * @Brief: definition of the ImgInputPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-16 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-16 16:23:27
 */

#include "CNNMethod/Predictor/ImgInputPredictor.h"
#include <algorithm>
#include <memory>
#include <string>
#include "CNNMethod/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/image_tools.h"
#include "hobotxroc/profiler.h"
#include "horizon/vision_type/vision_type.hpp"
#include "horizon/vision_type/vision_type_common.h"
#include "opencv2/opencv.hpp"

using hobot::vision::BBox;
using hobot::vision::ImageFrame;
using hobot::vision::PymImageFrame;
typedef std::shared_ptr<ImageFrame> ImageFramePtr;

namespace HobotXRoc {

int32_t ImgInputPredictor::Init(std::shared_ptr<CNNMethodConfig> config) {
  Predictor::Init(config);
  expand_scale_ = config->GetFloatValue("expand_scale", 1.0f);
  std::string s_norm_method = config->GetSTDStringValue("norm_method");
  auto iter = g_norm_method_map.find(s_norm_method);
  HOBOT_CHECK(iter != g_norm_method_map.end())
      << "norm_method is unknown:" << s_norm_method;
  norm_method_ = iter->second;

  std::string s_filter_method =
      config->GetSTDStringValue("filter_method", "no_filter");
  auto filter_iter = g_filter_method_map.find(s_filter_method);
  HOBOT_CHECK(filter_iter != g_filter_method_map.end())
      << "filter_method is unknown:" << s_filter_method;
  filter_method_ = filter_iter->second;

  rotate_degree_ = config->GetIntValue("rotate_degree");
}

void ImgInputPredictor::UpdateParam(std::shared_ptr<CNNMethodConfig> config) {
  Predictor::UpdateParam(config);
  if (config->KeyExist("expand_scale")) {
    expand_scale_ = config->GetFloatValue("expand_scale");
  }
  if (config->KeyExist("norm_method")) {
    std::string s_norm_method = config->GetSTDStringValue("norm_method");
    auto iter = g_norm_method_map.find(s_norm_method);
    if (iter == g_norm_method_map.end()) {
      LOGD << "norm_method is unknown:" << s_norm_method;
    } else {
      norm_method_ = iter->second;
    }
  }
  if (config->KeyExist("filter_method")) {
    std::string s_filter_method = config->GetSTDStringValue("filter_method");
    auto filter_iter = g_filter_method_map.find(s_filter_method);
    if (filter_iter == g_filter_method_map.end()) {
      LOGD << "filter_method is unknown:" << s_filter_method;
    } else {
      filter_method_ = filter_iter->second;
    }
  }
  if (config->KeyExist("rotate_degree")) {
    rotate_degree_ = config->GetIntValue("rotate_degree");
  }
}

void ImgInputPredictor::Do(CNNMethodRunData *run_data) {
  int frame_size = run_data->input->size();
  run_data->mxnet_output.resize(frame_size);
  run_data->input_dim_size.resize(frame_size);
  run_data->norm_rois.resize(frame_size);
  run_data->real_nhwc = model_info_.real_nhwc_;
  run_data->elem_size = model_info_.elem_size_;

  for (int frame_idx = 0; frame_idx < frame_size; frame_idx++) {  // loop frame
    auto &input_data = (*(run_data->input))[frame_idx];
    auto rois = std::static_pointer_cast<BaseDataVector>(input_data[0]);
    auto xroc_pyramid =
        std::static_pointer_cast<XRocData<ImageFramePtr>>(input_data[1]);
    auto pyramid = std::static_pointer_cast<PymImageFrame>(xroc_pyramid->value);

    int box_num = rois->datas_.size();
    run_data->mxnet_output[frame_idx].resize(box_num);
    run_data->input_dim_size[frame_idx] = box_num;
    run_data->norm_rois[frame_idx].resize(box_num);

    auto &norm_rois = run_data->norm_rois[frame_idx];

    uint32_t w = pyramid->Width();
    uint32_t h = pyramid->Height();
    uint32_t dst_h = model_info_.input_nhwc_[1];
    uint32_t dst_w = model_info_.input_nhwc_[2];

    int layer_size = model_info_.output_layer_size_.size();
    uint32_t handle_num =
        max_handle_num_ < 0 ? box_num : std::min(max_handle_num_, box_num);
    for (uint32_t roi_idx = 0; roi_idx < handle_num; roi_idx++) {
      auto p_roi =
          std::static_pointer_cast<XRocData<BBox>>(rois->datas_[roi_idx]);
      auto p_norm_roi = std::make_shared<XRocData<BBox>>();
      norm_rois[roi_idx] = std::static_pointer_cast<BaseData>(p_norm_roi);
      if (p_roi->state_ != HobotXRoc::DataState::VALID) {
        p_norm_roi->value = p_roi->value;
        continue;
      }
      BBox *norm_box = &(p_norm_roi->value);
      if (NormalizeRoi(&p_roi->value,
                       norm_box,
                       expand_scale_,
                       norm_method_,
                       w,
                       h,
                       filter_method_)) {
        LOGD << "norm roi error, box: [" << p_roi->value.x1 << ", "
             << p_roi->value.y1 << ", " << p_roi->value.x2 << ", "
             << p_roi->value.y2 << "]";
        continue;
      }

      int ret = 0;
      uint8_t *tmp_src_data = nullptr, *tmp_dst_data = nullptr;
      int tmp_src_size = 0, tmp_dst_size = 0;
      int src_1_stride = 0, src_2_stride = 0, dst_1_stride = 0,
          dst_2_stride = 0;
      int tmp_src_w = 0, tmp_src_h = 0, tmp_dst_w = 0, tmp_dst_h = 0;
      {
        RUN_PROCESS_TIME_PROFILER(model_name_ + "_crop")
        RUN_FPS_PROFILER(model_name_ + "_crop")

        // crop
        tmp_src_data = reinterpret_cast<uint8_t *>(pyramid->Data());
        tmp_src_size = w * h * 3 / 2;
        tmp_src_w = w;
        tmp_src_h = h;
        src_1_stride = pyramid->Stride();
        src_2_stride = pyramid->Stride();
        int ret = HobotXRocCropImage(tmp_src_data,
                                     tmp_src_size,
                                     tmp_src_w,
                                     tmp_src_h,
                                     src_1_stride,
                                     src_2_stride,
                                     IMAGE_TOOLS_RAW_YUV_NV12,
                                     norm_box->x1,
                                     norm_box->y1,
                                     norm_box->x2 - 1,
                                     norm_box->y2 - 1,
                                     &tmp_dst_data,
                                     &tmp_dst_size,
                                     &tmp_dst_w,
                                     &tmp_dst_h,
                                     &dst_1_stride,
                                     &dst_2_stride);
        HOBOT_CHECK(ret == 0)
            << "crop img failed"
            << ", src_size:" << tmp_src_size << ", src_w:" << tmp_src_w
            << ", src_h:" << tmp_src_h << ", src_1_stride:" << src_1_stride
            << ", src_2_stride:" << src_2_stride;

        tmp_src_data = tmp_dst_data;
        tmp_src_size = tmp_dst_size;
        tmp_src_w = tmp_dst_w;
        tmp_src_h = tmp_dst_h;
        src_1_stride = dst_1_stride;
        src_2_stride = dst_2_stride;
      }
      {
        RUN_PROCESS_TIME_PROFILER(model_name_ + "_resize")
        RUN_FPS_PROFILER(model_name_ + "_resize")
        // resize
        tmp_dst_w = (rotate_degree_ && rotate_degree_ != 180) ? dst_h : dst_w;
        tmp_dst_h = (rotate_degree_ && rotate_degree_ != 180) ? dst_w : dst_h;
        if (tmp_src_w != tmp_dst_w || tmp_src_h != tmp_dst_h) {
          struct HobotXRocImageToolsResizeInfo resize_info;
          ret = HobotXRocResizeImage(tmp_src_data,
                                     tmp_src_size,
                                     tmp_src_w,
                                     tmp_src_h,
                                     src_1_stride,
                                     src_2_stride,
                                     IMAGE_TOOLS_RAW_YUV_NV12,
                                     0,
                                     tmp_dst_w,
                                     tmp_dst_h,
                                     &tmp_dst_data,
                                     &tmp_dst_size,
                                     &dst_1_stride,
                                     &dst_2_stride,
                                     &resize_info);
          HOBOT_CHECK(ret == 0)
              << "resize img failed"
              << ", src_size:" << tmp_src_size << ", src_w:" << tmp_src_w
              << ", src_h:" << tmp_src_h << ", src_1_stride:" << src_1_stride
              << ", src_2_stride:" << src_2_stride;
          HobotXRocFreeImage(tmp_src_data);
          tmp_src_data = tmp_dst_data;
          tmp_src_size = tmp_dst_size;
          tmp_src_w = tmp_dst_w;
          tmp_src_h = tmp_dst_h;
          src_1_stride = dst_1_stride;
          src_2_stride = dst_2_stride;
        }
      }

      {
        RUN_PROCESS_TIME_PROFILER(model_name_ + "_rotate")
        RUN_FPS_PROFILER(model_name_ + "_rotate")

        // rotate
        if (rotate_degree_) {
          ret = HobotXRocRotateImage(tmp_src_data,
                                     tmp_src_size,
                                     tmp_src_w,
                                     tmp_src_h,
                                     src_1_stride,
                                     src_2_stride,
                                     IMAGE_TOOLS_RAW_YUV_NV12,
                                     rotate_degree_,
                                     &tmp_dst_data,
                                     &tmp_dst_size,
                                     &tmp_dst_w,
                                     &tmp_dst_h,
                                     &dst_1_stride,
                                     &dst_2_stride);
          HOBOT_CHECK(ret == 0)
              << "rotate img failed"
              << ", src_size:" << tmp_src_size << ", src_w:" << tmp_src_w
              << ", src_h:" << tmp_src_h << ", src_1_stride:" << src_1_stride
              << ", src_2_stride:" << src_2_stride;
          HobotXRocFreeImage(tmp_src_data);
          tmp_src_data = tmp_dst_data;
          tmp_src_size = tmp_dst_size;
          tmp_src_w = tmp_dst_w;
          tmp_src_h = tmp_dst_h;
          src_1_stride = dst_1_stride;
          src_2_stride = dst_2_stride;
        }
      }
      ModelOutputBuffer bufs(model_info_, 1);
      {
#if 0
        static int data_idx = 0;
        DumpBinaryFile(tmp_src_data,
                       tmp_src_size,
                       std::to_string(data_idx++) + ".nv12");
#endif
        RUN_PROCESS_TIME_PROFILER(model_name_ + "_runmodel")
        RUN_FPS_PROFILER(model_name_ + "_runmodel")
        int ret = RunModelFromImage(
            tmp_src_data, tmp_src_size, bufs.out_bufs_.data(), layer_size);
        if (ret == -1) {
          HobotXRocFreeImage(tmp_src_data);
          continue;
        }
      }
      LOGD << "RunModelFromImage success";
      HobotXRocFreeImage(tmp_src_data);

      auto &one_tgt_mxnet = run_data->mxnet_output[frame_idx][roi_idx];
      one_tgt_mxnet.resize(layer_size);
      // change raw data to mxnet layout
      {
        RUN_PROCESS_TIME_PROFILER(model_name_ + "_do_hbrt")
        RUN_FPS_PROFILER(model_name_ + "_do_hbrt")
        for (int j = 0; j < layer_size; j++) {
          uint32_t feature_size = model_info_.mxnet_output_layer_size_[j];
          one_tgt_mxnet[j].resize(feature_size);
          hbrtConvertLayout(feature_bufs_[j].data(), LAYOUT_NHWC_NATIVE,
                              BPU_getRawBufferPtr(bufs.out_bufs_[j]),
                              model_info_.layout_flag_[j],
                              model_info_.element_type_[j],
                              model_info_.align_dim_[j],
                              model_info_.convert_endianness_[j]);
          ConvertOutputToMXNet(
              feature_bufs_[j].data(), one_tgt_mxnet[j].data(), j);
        }
      }
      LOGD << "do hbrt success";
    }
  }
}

}  // namespace HobotXRoc
