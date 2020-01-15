//
// Created by yaoyao.sun on 2019-04-23.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef INCLUDE_FASTERRCNNMETHOD_RESULT_H_
#define INCLUDE_FASTERRCNNMETHOD_RESULT_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hobotxsdk/xroc_data.h"
#include "horizon/vision_type/vision_type.hpp"

namespace faster_rcnn_method {

using hobot::vision::BBox;
using hobot::vision::Landmarks;
using hobot::vision::Feature;
using hobot::vision::Segmentation;
using hobot::vision::Pose3D;
using hobot::vision::Attribute;

struct FasterRCNNOutMsg {
  std::map<std::string, std::vector<BBox>> boxes;
  std::map<std::string, std::vector<Landmarks>> landmarks;
  std::map<std::string, std::vector<Feature>> features;
  std::map<std::string, std::vector<Segmentation>> segmentations;
  std::map<std::string, std::vector<Pose3D>> poses;
  std::map<std::string, std::vector<Attribute<int>>> attributes;
};

}  // namespace faster_rcnn_method

#endif  // FASTERRCNNMETHOD_RESULT_H_
