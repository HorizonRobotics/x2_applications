/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CNNMethodData.h
 * @Brief: declaration of the CNNMethodData
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 16:16:58
 */

#ifndef INCLUDE_CNNMETHOD_UTIL_CNNMETHODDATA_H_
#define INCLUDE_CNNMETHOD_UTIL_CNNMETHODDATA_H_

#include <vector>
#include "bpu_predict/bpu_predict.h"
#include "hobotxsdk/xroc_data.h"
#include "horizon/vision_type/vision_type.hpp"

namespace HobotXRoc {

struct CNNMethodRunData {
  const std::vector<std::vector<BaseDataPtr>> *input;
  const std::vector<HobotXRoc::InputParamPtr> *param;

  std::vector<std::vector<uint32_t>> real_nhwc;

  std::vector<std::vector<BaseDataPtr>> norm_rois;

  std::vector<int> input_dim_size;
  std::vector<uint32_t> elem_size;

  // mxnet_output_[i] : frame i
  // mxnet_output_[i][j] : frame i, object j
  // mxnet_output_[i][j][k]: frame i, object j, layer k
  std::vector<std::vector<std::vector<std::vector<int8_t>>>> mxnet_output;

  std::vector<std::vector<BaseDataPtr>> output;
};
}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_UTIL_CNNMETHODDATA_H_
