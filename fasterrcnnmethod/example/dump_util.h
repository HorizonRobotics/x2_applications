//
// Created by yaoyao.sun on 2019-08-13.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef EXAMPLE_DUMP_UTIL_H_
#define EXAMPLE_DUMP_UTIL_H_

#include <string>
#include <vector>

#include "FasterRCNNMethod/dump.h"
#include "hobotlog/hobotlog.hpp"

using faster_rcnn_method::DumpRects;
using faster_rcnn_method::DumpRectsWithInfo;
using faster_rcnn_method::DumpLandmark;
using faster_rcnn_method::DumpSkeleton;
using faster_rcnn_method::DumpMask;
using faster_rcnn_method::DumpPlateWithInfo;

inline void DumpFace(cv::Mat *img, const std::vector<BaseDataPtr> &det_result,
                     std::string image_name = "") {
  LOGI << "dump start.";
  DumpRects(*img, std::static_pointer_cast<BaseDataVector>(det_result[0]), 0);
  cv::imwrite(image_name, *img);
  LOGI << "dump end.";
}

inline void DumpFaceLmk(cv::Mat *img,
                        const std::vector<BaseDataPtr> &det_result,
                        std::string image_name = "") {
  LOGI << "dump start.";
  DumpRects(*img, std::static_pointer_cast<BaseDataVector>(det_result[0]), 0);
  DumpLandmark(*img, std::static_pointer_cast<BaseDataVector>(det_result[1]));
  cv::imwrite(image_name, *img);
  LOGI << "dump end.";
}

inline void DumpMultitask(cv::Mat *img,
                          const std::vector<BaseDataPtr> &det_result,
                          std::string image_name = "") {
  LOGI << "dump start.";
  DumpRects(*img, std::static_pointer_cast<BaseDataVector>(det_result[0]), 0);
  DumpRects(*img, std::static_pointer_cast<BaseDataVector>(det_result[1]), 1);
  DumpRects(*img, std::static_pointer_cast<BaseDataVector>(det_result[2]), 2);
  DumpSkeleton(*img, std::static_pointer_cast<BaseDataVector>(det_result[3]));
  DumpMask(*img, std::static_pointer_cast<BaseDataVector>(det_result[4]),
           std::static_pointer_cast<BaseDataVector>(det_result[2]));
  cv::imwrite(image_name, *img);
  LOGI << "dump end.";
}

inline void DumpVehicle(cv::Mat *img,
                        const std::vector<BaseDataPtr> &det_result,
                        std::string image_name = "") {
  LOGI << "dump start.";
  DumpRectsWithInfo(*img,
                    std::static_pointer_cast<BaseDataVector>(det_result[0]), 0,
                    image_name);
  DumpRectsWithInfo(*img,
                    std::static_pointer_cast<BaseDataVector>(det_result[1]), 1,
                    image_name);
  DumpRectsWithInfo(*img,
                    std::static_pointer_cast<BaseDataVector>(det_result[2]), 2,
                    image_name);
  DumpRectsWithInfo(*img,
                    std::static_pointer_cast<BaseDataVector>(det_result[3]), 3,
                    image_name);
  DumpRectsWithInfo(*img,
                    std::static_pointer_cast<BaseDataVector>(det_result[4]), 4,
                    image_name);
  DumpPlateWithInfo(
      *img, std::static_pointer_cast<BaseDataVector>(det_result[5]),
      std::static_pointer_cast<BaseDataVector>(det_result[6]), 0, image_name);

  cv::imwrite("./render/" + image_name, *img);
  LOGI << "dump end.";
}

#endif  // EXAMPLE_DUMP_UTIL_H_
