/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2020-01-01 18:52:54
 * @Version: v0.0.1
 * @Brief: BBox DataStruct definition
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2020-01-01 19:00:39
 */
#ifndef XROC_FRAMEWORK_EXAMPLE_TUTORIALS_METHODS_BBOX_DATA_H_
#define XROC_FRAMEWORK_EXAMPLE_TUTORIALS_METHODS_BBOX_DATA_H_
#include <iostream>
#include <string>
#include "hobotxroc/method.h"

namespace hobot {
namespace vision {
template <typename Dtype>
struct BBox_ {
  inline BBox_() {}
  inline BBox_(Dtype x1_, Dtype y1_, Dtype x2_, Dtype y2_, float score_ = 0.0f,
               int32_t id_ = -1, const std::string &category_name_ = "") {
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
    id = id_;
    score = score_;
    category_name = category_name_;
  }
  inline Dtype Width() const { return (x2 - x1); }
  inline Dtype Height() const { return (y2 - y1); }
  inline Dtype CenterX() const { return (x1 + (x2 - x1) / 2); }
  inline Dtype CenterY() const { return (y1 + (y2 - y1) / 2); }

  inline friend std::ostream &operator<<(std::ostream &out, BBox_ &bbox) {
    out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2
        << " y2: " << bbox.y2 << " score: " << bbox.score << " )";
    return out;
  }

  inline friend std::ostream &operator<<(std::ostream &out, const BBox_ &bbox) {
    out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2
        << " y2: " << bbox.y2 << " score: " << bbox.score << " )";
    return out;
  }

  Dtype x1 = 0;
  Dtype y1 = 0;
  Dtype x2 = 0;
  Dtype y2 = 0;
  float score = 0.0;
  int32_t id = 0;
  std::string category_name = "";
};
typedef BBox_<float> BBox;
}  // namespace vision
}  // namespace hobot

namespace HobotXRoc {
typedef XRocData<hobot::vision::BBox> BBox;
}  // namespace HobotXRoc


#endif  // XROC_FRAMEWORK_EXAMPLE_TUTORIALS_METHODS_BBOX_DATA_H_