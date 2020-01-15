/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: Predictor.cpp
 * @Brief: definition of the Predictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-16 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-16 16:23:27
 */

#include "CNNMethod/Predictor/Predictor.h"
#include <stdint.h>
#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "CNNMethod/util/util.h"
#include "hb_vio_interface.h"
#include "hbdk/hbdk_layout.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/profiler.h"
#include "horizon/vision_type/vision_type.hpp"
#include "horizon/vision_type/vision_type_common.h"
#include "hobot_vision/bpumodel_manager.hpp"

namespace HobotXRoc {
int32_t Predictor::Init(std::shared_ptr<CNNMethodConfig> config) {
  model_name_ = config->GetSTDStringValue("model_name");
  model_version_ = config->GetSTDStringValue("model_version", "unknown");
  HOBOT_CHECK(model_name_.size() > 0) << "must set model_name";

  std::string parent_path = config->GetSTDStringValue("parent_path");
  model_path_ = config->GetSTDStringValue("model_file_path");
  std::string bpu_cfg_path = config->GetSTDStringValue("bpu_config_path");
  max_handle_num_ = config->GetIntValue("max_handle_num", -1);
  HOBOT_CHECK(model_path_.size() > 0) << "must set model_file_path";
  HOBOT_CHECK(bpu_cfg_path.size() > 0) << "must set bpu_config_cfg";

  model_path_ = parent_path + model_path_;
  bpu_cfg_path = parent_path + bpu_cfg_path;
  LOGD << "model_file_path:" << model_path_ << std::endl
       << "bpu_config_path:" << bpu_cfg_path
       << "parent path" << parent_path;
  bpu_handle_ = hobot::vision::BPUModelManager::Get().GetBpuHandle(
    model_path_, bpu_cfg_path);
  LOGI << "BPU version:" << BPU_getVersion(bpu_handle_);

  BPUModelInfo output_model_info, input_model_info;
  int ret = BPU_getModelOutputInfo(
      bpu_handle_, model_name_.c_str(), &output_model_info);
  HOBOT_CHECK(ret == 0) << "get model output info failed, model:"
                        << model_name_;
  ret = BPU_getModelInputInfo(
      bpu_handle_, model_name_.c_str(), &input_model_info);
  HOBOT_CHECK(ret == 0) << "get model input info failed, model:"
                        << model_name_;
  model_info_.Init(bpu_handle_, model_name_, &input_model_info,
                    &output_model_info);

  feature_bufs_.resize(model_info_.output_layer_size_.size());
  for (int i = 0; i < model_info_.output_layer_size_.size(); i++) {
    feature_bufs_[i].resize(model_info_.output_layer_size_[i]);
  }

  ret = BPU_createFakeImageHandle(model_info_.input_nhwc_[1],
                                  model_info_.input_nhwc_[2],
                                  &fake_img_handle_);
  HOBOT_CHECK(ret == 0) << "create fake image handle failed";
  return 0;
}

void Predictor::UpdateParam(std::shared_ptr<CNNMethodConfig> config) {
  if (config->KeyExist("max_handle_num")) {
    max_handle_num_ = config->GetIntValue("max_handle_num");
  }
}

void Predictor::Finalize() {
  if (fake_img_handle_) {
    BPU_releaseFakeImageHandle(fake_img_handle_);
  }
  if (bpu_handle_) {
    hobot::vision::BPUModelManager::Get().ReleaseBpuHandle(model_path_);
    LOGD << "cnn release model " << model_path_;
  }
}

int Predictor::RunModelFromImage(uint8_t *data,
                                 int data_size,
                                 BPU_Buffer_Handle *output_buf,
                                 int output_size) {
  BPUFakeImage *fake_img_ptr = nullptr;
  fake_img_ptr = BPU_getFakeImage(fake_img_handle_, data, data_size);
  if (fake_img_ptr == nullptr) {
    LOGE << "get fake image failed";
    return -1;
  }
  BPUModelHandle model_handle;
  int ret = BPU_runModelFromImage(bpu_handle_,
                                  model_name_.c_str(),
                                  fake_img_ptr,
                                  output_buf,
                                  output_size,
                                  &model_handle);
  if (ret != 0) {
    LOGE << "BPU_runModelFromImage failed:" << BPU_getLastError(bpu_handle_);
    BPU_releaseFakeImage(fake_img_handle_, fake_img_ptr);
    return -1;
  }
  ret = BPU_getModelOutput(bpu_handle_, model_handle);
  BPU_releaseFakeImage(fake_img_handle_, fake_img_ptr);

  if (ret != 0) {
    LOGE << "BPU_getModelOutput failed:" << BPU_getLastError(bpu_handle_);
    BPU_releaseModelHandle(bpu_handle_, model_handle);
    return -1;
  }
  BPU_releaseModelHandle(bpu_handle_, model_handle);
  return 0;
}

int Predictor::RunModelFromResizer(BPUPyramidBuffer input,
                                   BPUBBox *box,
                                   int box_num,
                                   int *resizable_cnt,
                                   BPU_Buffer_Handle *output_buf,
                                   int output_size) {
  BPUModelHandle model_handle;
  int ret = BPU_runModelFromResizer(bpu_handle_,
                                    model_name_.c_str(),
                                    input,
                                    box,
                                    box_num,
                                    resizable_cnt,
                                    output_buf,
                                    output_size,
                                    &model_handle);
  if (ret != 0 && *resizable_cnt == 0) {
    LOGI << "no box pass resizer";
    return -1;
  } else if (ret != 0) {
    LOGE << "BPU_runModelFromResizer failed:" << BPU_getLastError(bpu_handle_);
    return -1;
  }
  ret = BPU_getModelOutput(bpu_handle_, model_handle);

  if (ret != 0) {
    LOGE << "BPU_getModelOutput failed:" << BPU_getLastError(bpu_handle_);
    BPU_releaseModelHandle(bpu_handle_, model_handle);
    return -1;
  }
  BPU_releaseModelHandle(bpu_handle_, model_handle);
  LOGD << "resizeable box:" << *resizable_cnt;
  return 0;
}

void Predictor::ConvertOutputToMXNet(void *src_ptr,
                                     void *dest_ptr,
                                     int layer_idx) {
  auto &aligned_nhwc = model_info_.aligned_nhwc_[layer_idx];
  auto &real_nhwc = model_info_.real_nhwc_[layer_idx];
  auto elem_size = model_info_.elem_size_[layer_idx];
  auto &shift = model_info_.all_shift_[layer_idx];

  uint32_t dst_n_stride =
      real_nhwc[1] * real_nhwc[2] * real_nhwc[3] * elem_size;
  uint32_t dst_h_stride = real_nhwc[2] * real_nhwc[3] * elem_size;
  uint32_t dst_w_stride = real_nhwc[3] * elem_size;
  uint32_t src_n_stride =
      aligned_nhwc[1] * aligned_nhwc[2] * aligned_nhwc[3] * elem_size;
  uint32_t src_h_stride = aligned_nhwc[2] * aligned_nhwc[3] * elem_size;
  uint32_t src_w_stride = aligned_nhwc[3] * elem_size;
  float tmp_float_value;
  int32_t tmp_int32_value;
  for (uint32_t nn = 0; nn < real_nhwc[0]; nn++) {
    void *cur_n_dst = reinterpret_cast<int8_t *>(dest_ptr) + nn * dst_n_stride;
    void *cur_n_src = reinterpret_cast<int8_t *>(src_ptr) + nn * src_n_stride;
    for (uint32_t hh = 0; hh < real_nhwc[1]; hh++) {
      void *cur_h_dst =
          reinterpret_cast<int8_t *>(cur_n_dst) + hh * dst_h_stride;
      void *cur_h_src =
          reinterpret_cast<int8_t *>(cur_n_src) + hh * src_h_stride;
      for (uint32_t ww = 0; ww < real_nhwc[2]; ww++) {
        void *cur_w_dst =
            reinterpret_cast<int8_t *>(cur_h_dst) + ww * dst_w_stride;
        void *cur_w_src =
            reinterpret_cast<int8_t *>(cur_h_src) + ww * src_w_stride;
        for (uint32_t cc = 0; cc < real_nhwc[3]; cc++) {
          void *cur_c_dst =
              reinterpret_cast<int8_t *>(cur_w_dst) + cc * elem_size;
          void *cur_c_src =
              reinterpret_cast<int8_t *>(cur_w_src) + cc * elem_size;
          if (elem_size == 4) {
            tmp_int32_value = *(reinterpret_cast<int32_t *>(cur_c_src));
            tmp_float_value = GetFloatByInt(tmp_int32_value, shift[cc]);
            *(reinterpret_cast<float *>(cur_c_dst)) = tmp_float_value;
          } else {
            *(reinterpret_cast<int8_t *>(cur_c_dst)) =
                *(reinterpret_cast<int8_t *>(cur_c_src));
          }
        }
      }
    }
  }
}

int Predictor::NormalizeRoi(hobot::vision::BBox *src,
                            hobot::vision::BBox *dst,
                            float norm_ratio,
                            NormMethod norm_method,
                            uint32_t total_w,
                            uint32_t total_h,
                            FilterMethod filter_method) {
  *dst = *src;
  float box_w = dst->x2 - dst->x1;
  float box_h = dst->y2 - dst->y1;
  float center_x = (dst->x1 + dst->x2) / 2.0f;
  float center_y = (dst->y1 + dst->y2) / 2.0f;
  float w_new = box_w;
  float h_new = box_h;
  switch (norm_method) {
    case NormMethod::BPU_MODEL_NORM_BY_WIDTH_LENGTH: {
      w_new = box_w * norm_ratio;
      h_new = box_h + w_new - box_w;
      if (h_new <= 0) return -1;
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_WIDTH_RATIO:
    case NormMethod::BPU_MODEL_NORM_BY_HEIGHT_RATIO:
    case NormMethod::BPU_MODEL_NORM_BY_LSIDE_RATIO: {
      h_new = box_h * norm_ratio;
      w_new = box_w * norm_ratio;
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_HEIGHT_LENGTH: {
      h_new = box_h * norm_ratio;
      w_new = box_w + h_new - box_h;
      if (w_new <= 0) return -1;
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_LSIDE_LENGTH: {
      if (box_w > box_h) {
        w_new = box_w * norm_ratio;
        h_new = box_h + w_new - box_w;
        if (h_new <= 0) return -1;
      } else {
        h_new = box_h * norm_ratio;
        w_new = box_w + h_new - box_h;
        if (w_new <= 0) return -1;
      }
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_LSIDE_SQUARE: {
      if (box_w > box_h) {
        w_new = box_w * norm_ratio;
        h_new = w_new;
      } else {
        h_new = box_h * norm_ratio;
        w_new = h_new;
      }
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_DIAGONAL_SQUARE: {
      float diagonal = sqrt(pow(box_w, 2.0) + pow(box_h, 2.0));
      w_new = h_new = diagonal * norm_ratio;
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_NOTHING: break;
    default: return 0;
  }
  dst->x1 = center_x - w_new / 2;
  dst->x2 = center_x + w_new / 2;
  dst->y1 = center_y - h_new / 2;
  dst->y2 = center_y + h_new / 2;

  if (FilterRoi(src, dst, total_w, total_h, filter_method)) {
    *dst = *src;
    return -1;
  }

  dst->x1 = dst->x1 < 0 ? 0.0f : dst->x1;
  dst->y1 = dst->y1 < 0 ? 0.0f : dst->y1;
  dst->x2 = dst->x2 > total_w ? total_w : dst->x2;
  dst->y2 = dst->y2 > total_h ? total_h : dst->y2;
  LOGD << "norm roi[x1, y1, x2, y2]: [" << dst->x1 << ", " << dst->y1 << ", "
       << dst->x2 << ", " << dst->y2 << "]";
  return 0;
}

int Predictor::FilterRoi(hobot::vision::BBox *src,
                         hobot::vision::BBox *dst,
                         int src_w,
                         int src_h,
                         FilterMethod filter_method) {
  switch (filter_method) {
    case FilterMethod::OUT_OF_RANGE: {
      if (dst->x1 < 0 || dst->y1 < 0 || dst->x2 > src_w || dst->y2 > src_h)
        return -1;
    } break;
  }
  return 0;
}
}  // namespace HobotXRoc
