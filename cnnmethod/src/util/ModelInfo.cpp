/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: ModelInfo.cpp
 * @Brief: definition of the ModelInfo
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-05-12 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-05-12 16:17:08
 */

#include "CNNMethod/util/ModelInfo.h"
#include "bpu_predict/bpu_internal.h"
#include "hobotlog/hobotlog.hpp"

namespace HobotXRoc {

void ModelInfo::Init(BPUHandle bpuHandler,
                     std::string model_name,
                     BPUModelInfo *input_model_info,
                     BPUModelInfo *output_model_info) {
  model_name_ = model_name;
//  model_file_path_ = model_file_path;
  aligned_nhwc_.resize(output_model_info->num);
  real_nhwc_.resize(output_model_info->num);
  output_layer_size_.resize(output_model_info->num);
  mxnet_output_layer_size_.resize(output_model_info->num);

  // get info: aligned_nhwc_ and output_layer_size_
  for (int i = 0; i < output_model_info->num; ++i) {
    int model_out_size = 1;
    for (int j = output_model_info->ndim_array[i];
         j < output_model_info->ndim_array[i + 1];
         ++j) {
      model_out_size *= output_model_info->aligned_shape_array[j];
      aligned_nhwc_[i].push_back(output_model_info->aligned_shape_array[j]);
    }
    int out_type_size = sizeof(int8_t);
    if (output_model_info->dtype_array[i] == BPU_DTYPE_FLOAT32) {
      out_type_size = sizeof(float);
    }
    output_layer_size_[i] = out_type_size * model_out_size;
  }

  // get info: input_nhwc_
  for (int i = input_model_info->ndim_array[0];
       i < input_model_info->ndim_array[1];
       i++) {
    input_nhwc_.push_back(input_model_info->valid_shape_array[i]);
  }

  hbrt_hbm_handle_t hbm_handle;
  int ret = BPU_getHBMhandleFromBPUhandle(bpuHandler, &hbm_handle.handle);
  HOBOT_CHECK(ret == 0) << "Load bpu model failed: "
                          << BPU_getLastError(bpuHandler);
  hbrt_model_handle_t model_handle_;
  CHECK_HBRT_ERROR(hbrtGetModelHandle(&model_handle_,
                                    hbm_handle, model_name_.c_str()));
  uint32_t num_out = 0;
  CHECK_HBRT_ERROR(hbrtGetOutputFeatureNumber(&num_out, model_handle_));
  const hbrt_feature_handle_t *feature_info;
  CHECK_HBRT_ERROR(hbrtGetOutputFeatureHandles(&feature_info, model_handle_));
  all_shift_.resize(num_out);
  layout_flag_.resize(num_out);
  element_type_.resize(num_out);
  align_dim_.resize(num_out);
  convert_endianness_.resize(num_out);
  for (int i = 0; i < num_out; i++) {
    hbrt_layout_type_t layout;
    CHECK_HBRT_ERROR(hbrtGetFeatureLayoutType(&layout, feature_info[i]));
    layout_flag_[i] = layout;
    hbrt_dimension_t valid_dim;
    CHECK_HBRT_ERROR(hbrtGetFeatureValidDimension(&valid_dim, feature_info[i]));
    real_nhwc_[i].push_back(valid_dim.n);
    real_nhwc_[i].push_back(valid_dim.h);
    real_nhwc_[i].push_back(valid_dim.w);
    real_nhwc_[i].push_back(valid_dim.c);

    hbrt_dimension_t aligned_dim;
    CHECK_HBRT_ERROR(hbrtGetFeatureAlignedDimension(&aligned_dim,
                      feature_info[i]));
    align_dim_[i] = aligned_dim;

    bool is_big_endian;
    CHECK_HBRT_ERROR(hbrtFeatureIsBigEndian(&is_big_endian, feature_info[i]));
    convert_endianness_[i] = is_big_endian;

    hbrt_element_type_t element_type;
    CHECK_HBRT_ERROR(hbrtGetFeatureElementType(&element_type, feature_info[i]));
    element_type_[i] = element_type;
    uint32_t size;
    CHECK_HBRT_ERROR(hbrtGetElementSize(&size, element_type));
    elem_size_.push_back(size);

    const uint8_t *shift_value;
    CHECK_HBRT_ERROR(hbrtGetFeatureShiftValues(&shift_value, feature_info[i]));
    int channel_num = real_nhwc_[i][3];
    for (int j = 0; j < channel_num; j++) {
      all_shift_[i].push_back(static_cast<uint32_t>(shift_value[j]));
    }

    uint32_t valid_byte_size;
    CHECK_HBRT_ERROR(hbrtGetFeatureValidTotalByteSize(&valid_byte_size,
                     feature_info[i]));
    mxnet_output_layer_size_[i] = static_cast<int>(valid_byte_size);
  }
}

std::ostream &operator<<(std::ostream &os, const ModelInfo &info) {
  os << "model name:" << info.model_name_ << std::endl;
  // output_layer_size
  os << "output layer size:" << std::endl;
  for (auto &o_size : info.output_layer_size_) {
    os << o_size << " ";
  }
  os << std::endl;
  os << "mxnet output layer size:" << std::endl;
  for (auto &o_size : info.mxnet_output_layer_size_) {
    os << o_size << " ";
  }
  os << std::endl;
  // aligned_nhwc
  os << "aligned nhwc:" << std::endl;
  for (auto &layer_nhwc : info.aligned_nhwc_) {
    for (auto &nhwc : layer_nhwc) {
      os << nhwc << " ";
    }
    os << std::endl;
  }

  // real nhwc
  os << "real nhwc:" << std::endl;
  for (auto &layer_nhwc : info.real_nhwc_) {
    for (auto &nhwc : layer_nhwc) {
      os << nhwc << " ";
    }
    os << std::endl;
  }
  // layput flag
  os << "layout flag:" << std::endl;
  for (auto &flag : info.layout_flag_) {
    os << flag << " ";
  }
  os << std::endl;
  // shift
  os << "shift:" << std::endl;
  for (auto &layout_shift : info.all_shift_) {
    for (auto &shift : layout_shift) {
      os << shift << " ";
    }
    os << std::endl;
  }
  // elem_size
  os << "elem_size:" << std::endl;
  for (auto &elem_size : info.elem_size_) {
    os << elem_size << " ";
  }
  os << std::endl;

  // input hnwc
  os << "input_nhwc:" << std::endl;
  for (auto v : info.input_nhwc_) {
    os << v << " ";
  }

    // aligned dim
  os << "output align dim:" << std::endl;
  for (auto& data : info.align_dim_) {
    os << data.n << " ," << data.h << ","
       << data.w << "," << data.c << std::endl;
  }
  os << std::endl;
}

}  // namespace HobotXRoc
