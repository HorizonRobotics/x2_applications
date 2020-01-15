/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides common types of xroc image tools interface
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.10
 */
#ifndef INCLUDE_HOBOTXROC_IMAGETOOLS_COMMON_H_
#define INCLUDE_HOBOTXROC_IMAGETOOLS_COMMON_H_
#include <assert.h>
#include "hobotxroc/image_tools.h"

namespace HobotXRoc {

enum class FotmatDataArrayType {
  kContinueType,  // for BGR/RGB/GRAY, and for Y U V store continue
  kSeperateType  // for Y U V store seperately, e.g. vio returned NV12
};

struct ImageToolsFormatData {
  FotmatDataArrayType array_type_ = FotmatDataArrayType::kContinueType;
  // kContinueType only use first element.
  // kSeperateType can use two or three elements.
  // BGR/RGB/GRAY only support kContinueType
  // kSeperateType NV12 only use first two elements.
  uint8_t *data_[3] = {nullptr, nullptr, nullptr};
  int data_size_[3] = {0, 0, 0};
  int width_ = 0;
  int height_ = 0;
  int first_stride_ = 0;  // y/rgb/bgr/gray stride
  int second_stride_ = 0;  // uv stride
  HobotXRocImageToolsPixelFormat format_;
  inline bool Valid() const {
    if (width_ <= 0 || height_ <= 0 || first_stride_ <= 0) {
      return false;
    }

    int data_max_index = 1;
    if (FotmatDataArrayType::kContinueType == array_type_) {
      data_max_index = 1;
    } else if (IMAGE_TOOLS_RAW_YUV_NV21 == format_
               || IMAGE_TOOLS_RAW_YUV_NV12 == format_) {
      data_max_index = 2;
    } else if (IMAGE_TOOLS_RAW_YUV_I420 == format_) {
      data_max_index = 3;
    } else {
      return false;
    }
    for (int i = 0; i < data_max_index; ++i) {
      if (nullptr == data_[i] || data_size_[i] <= 0) {
        return false;
      }
    }
    if (first_stride_ < width_) {
      return false;
    }
    bool ret = true;
    if (IMAGE_TOOLS_RAW_RGB == format_
        || IMAGE_TOOLS_RAW_BGR == format_) {
      ret = (first_stride_ >= width_ * 3)
             && (data_size_[0] >=  width_ * height_ * 3);
    } else if (IMAGE_TOOLS_RAW_GRAY == format_) {
      ret = (first_stride_ >= width_)
             && (data_size_[0] >= width_ * height_);
    } else if (IMAGE_TOOLS_RAW_YUV_I420 == format_) {
      ret = (first_stride_ >= width_)
             && (second_stride_ >= width_ / 2);
      if (FotmatDataArrayType::kContinueType == array_type_) {
        ret = ret && (data_size_[0] >= width_ * height_ * 3 / 2);
      } else {
        ret = ret && (data_size_[0] >= width_ * height_)
                  && (data_size_[1] >= (width_ * height_ >> 2))
                  && (data_size_[2] >= (width_ * height_ >> 2));
      }
    } else {  // nv12 or nv21
      ret = (first_stride_ >= width_)
             && (second_stride_ >= width_);
      if (FotmatDataArrayType::kContinueType == array_type_) {
        ret = ret && (data_size_[0] >= width_ * height_ * 3 / 2);
      } else {
        ret = ret && (data_size_[0] >= width_ * height_)
                  && (data_size_[1] >= (width_ * height_ >> 1));
      }
    }
    return ret;
  }
};
#pragma GCC push_options
#pragma GCC optimize("fp-contract=off")
inline void BilinearInterpolation(const int src_w,
                                  const int src_h,
                                  const int element_size,
                                  const int input_stride,
                                  const uint8_t *src,
                                  const int dst_w,
                                  const int dst_h,
                                  const int output_stride,
                                  uint8_t *dst) {
  if (nullptr == src || nullptr == dst) {
    return;
  }
  uint8_t *output = dst;
  assert(dst_h > 0);
  assert(dst_w > 0);
  for (int h = 0; h < dst_h; ++h) {
    for (int w = 0; w < dst_w; ++w) {
      float x = 1.0 * w * src_w / static_cast<float>(dst_w);
      float y = 1.0 * h * src_h / static_cast<float>(dst_h);
      int x1 = static_cast<int>(x);
      int x2 = x1 + 1;
      int y1 = static_cast<int>(y);
      int y2 = y1 + 1;
      if (x2 >= src_w) {
        x2 = src_w - 1;
      }
      if (y2 >= src_h) {
        y2 = src_h - 1;
      }
      assert(x1 >= 0 && y1 >= 0);
      assert(x2 < src_w && y2 < src_h);

      for (int element = 0; element < element_size; ++element) {
        uint8_t x1y1_val = src[y1 * input_stride
                               + x1 * element_size + element];
        uint8_t x1y2_val = src[y2 * input_stride
                               + x1 * element_size + element];
        uint8_t x2y1_val = src[y1 * input_stride
                               + x2 * element_size + element];
        uint8_t x2y2_val = src[y2 * input_stride
                               + x2 * element_size + element];
        float top_inter = (x - x1) * x2y1_val + (x2 - x) * x1y1_val;
        float bottom_inter = (x - x1) * x2y2_val + (x2 - x) * x1y2_val;
        float inter = (y - y1) * bottom_inter + (y2 - y) * top_inter;
        output[w * element_size + element] = static_cast<uint8_t>(inter);
      }
    }
    output += output_stride;
  }
}
#pragma GCC pop_options

}  // namespace HobotXRoc
#endif  // INCLUDE_HOBOTXROC_IMAGETOOLS_COMMON_H_
