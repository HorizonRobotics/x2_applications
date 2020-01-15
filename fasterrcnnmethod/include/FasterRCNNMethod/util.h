/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     Util
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#ifndef INCLUDE_FASTERRCNNMETHOD_UTIL_H_
#define INCLUDE_FASTERRCNNMETHOD_UTIL_H_

#include <string.h>
#include <string>
#include <memory>
#include <numeric>

#include "hobotxsdk/xroc_data.h"
#include "horizon/vision_type/vision_type.hpp"

#include "result.h"

using hobot::vision::Feature;

namespace faster_rcnn_method {

inline void MemZero(void* p, size_t n)
{
  memset(p, 0, n);
}

inline void l2_norm(Feature &feature) {
  float sum = 0.0;
  sum = std::inner_product(feature.values.begin(), feature.values.end(), feature.values.begin(), sum);
  float eps = 1e-10;
  sum = sqrt(sum) + eps;
  for (auto &value : feature.values) {
    value = value / sum;
  }
  return;
}

inline float SigMoid(const float &input) {
  return 1 / (1 + std::exp(-1 * input));
}

inline float GetFloatByInt(int32_t value, uint32_t shift) {
  return (static_cast<float>(value)) / (static_cast<float>(1 << shift));
}

// coordinate transform.
// fasterrcnn model's input size maybe not eqaul to origin image size,
// needs coordinate transform for detection result.

void CoordinateTransform(FasterRCNNOutMsg &det_result,
                         int src_image_width, int src_image_height,
                         int model_input_width, int model_input_hight) {
  for (auto &boxes : det_result.boxes) {
    for (auto &box : boxes.second) {
      box.x1 = box.x1 * src_image_width / model_input_width;
      box.y1 = box.y1 * src_image_height / model_input_hight;
      box.x2 = box.x2 * src_image_width / model_input_width;
      box.y2 = box.y2 * src_image_height / model_input_hight;
    }
  }

  for (auto &landmarks : det_result.landmarks) {
    for (auto &landmark : landmarks.second) {
      for (auto &point : landmark.values) {
        point.x = point.x * src_image_width / model_input_width;
        point.y = point.y * src_image_height / model_input_hight;
      }
    }
  }
}

std::string GetParentPath(const std::string &path) {
  auto pos = path.rfind('/');
  if (std::string::npos != pos) {
    auto parent = path.substr(0, pos);
    return parent + "/";
  } else {
    return std::string("./");
  }
}

}  // namespace faster_rcnn_method

#endif  // INCLUDE_FASTERRCNNMETHOD_UTIL_H_
