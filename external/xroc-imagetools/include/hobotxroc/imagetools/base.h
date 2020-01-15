/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     图像处理类的基类定义
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.14
 */

#ifndef INCLUDE_HOBOTXROC_IMAGETOOLS_BASE_H_
#define INCLUDE_HOBOTXROC_IMAGETOOLS_BASE_H_

#include "hobotxroc/imagetools/common.h"

namespace HobotXRoc {

class ImageBase {
 public:
  inline void GetInputImageInfo(const ImageToolsFormatData &input) {
    for (int i = 0; i < 3; ++i) {
      input_data_[i] = input.data_[i];
      input_data_size_[i] = input.data_size_[i];
    }
    input_array_type_ = input.array_type_;
    input_fmt_ = input.format_;
    input_width_ = input.width_;
    input_height_ = input.height_;
    input_first_stride_ = input.first_stride_;
    input_second_stride_ = input.second_stride_;
  }

  inline void SetOutputImageInfo(ImageToolsFormatData &output) {
    output.data_[0] = output_data_;
    output.data_size_[0] = output_data_size_;
    output.array_type_ = output_array_type_;
    output.format_ = output_fmt_;
    output.width_ = output_width_;
    output.height_ = output_height_;
    output.first_stride_ = output_first_stride_;
    output.second_stride_ = output_second_stride_;
  }

 protected:
  const uint8_t *input_data_[3] = {nullptr, nullptr, nullptr};
  int input_data_size_[3] = {0, 0, 0};
  FotmatDataArrayType input_array_type_;
  HobotXRocImageToolsPixelFormat input_fmt_;
  int input_first_stride_ = 0;
  int input_second_stride_ = 0;
  int input_width_ = 0;
  int input_height_ = 0;
  int output_first_stride_ = 0;
  int output_second_stride_ = 0;
  // output only support kContinueType mode
  uint8_t *output_data_ = nullptr;
  int output_data_size_ = 0;
  FotmatDataArrayType output_array_type_ = FotmatDataArrayType::kContinueType;
  int output_width_ = 0;
  int output_height_ = 0;
  HobotXRocImageToolsPixelFormat output_fmt_;
};

}  // namespace HobotXRoc

#endif  // INCLUDE_HOBOTXROC_IMAGETOOLS_BASE_H_
