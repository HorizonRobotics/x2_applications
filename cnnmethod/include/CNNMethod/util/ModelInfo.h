/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: ModelInfo.h
 * @Brief: declaration of the ModelInfo
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-05-12 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-05-12 16:16:58
 */

#ifndef INCLUDE_CNNMETHOD_UTIL_MODELINFO_H_
#define INCLUDE_CNNMETHOD_UTIL_MODELINFO_H_

#include <iostream>
#include <string>
#include <vector>
#include "bpu_predict/bpu_io.h"
#include "bpu_predict/bpu_predict.h"
#include "hbdk/hbdk_layout.h"
#include "hbdk/hbdk_hbrt.h"
namespace HobotXRoc {

class ModelInfo {
 public:
  void Init(BPUHandle bpuHandler,
            std::string model_name,
            BPUModelInfo *input_model,
            BPUModelInfo *output_model);
  friend std::ostream &operator<<(std::ostream &o, const ModelInfo &info);

 public:
  std::string model_name_;
  std::string model_file_path_;

  // output info
  std::vector<int> output_layer_size_;               // include sizeof(ele)
  std::vector<int> mxnet_output_layer_size_;         // include sizeof(ele)
  std::vector<std::vector<uint32_t>> aligned_nhwc_;  // bpu output nhwc
  std::vector<std::vector<uint32_t>> real_nhwc_;     // mxnet output nhwc
  std::vector<hbrt_layout_type_t> layout_flag_;
  std::vector<uint32_t> elem_size_;
  std::vector<std::vector<uint32_t>> all_shift_;
  // input info
  std::vector<int> input_nhwc_;
  std::vector<hbrt_element_type_t> element_type_;
  std::vector<hbrt_dimension_t> align_dim_;
  std::vector<int> convert_endianness_;
};

}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_UTIL_MODELINFO_H_
