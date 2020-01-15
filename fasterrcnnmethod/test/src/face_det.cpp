//
// Created by yaoyao.sun on 2019-05-14.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <gtest/gtest.h>

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>

#include "yuv_utils.h"
#include "FasterRCNNMethod/dump.h"

#include "bpu_predict/bpu_io.h"
#include "horizon/vision_type/vision_type.hpp"

#include "hobotxsdk/xroc_sdk.h"
#include "opencv2/opencv.hpp"

#include "FasterRCNNMethod.h"
#include "bpu_predict/bpu_predict.h"

using HobotXRoc::BaseData;
using HobotXRoc::BaseDataPtr;
using HobotXRoc::BaseDataVector;
using HobotXRoc::InputData;
using HobotXRoc::InputDataPtr;
using HobotXRoc::XRocData;
using HobotXRoc::InputParamPtr;

using HobotXRoc::FasterRCNNMethod;
using hobot::vision::ImageFrame;
using hobot::vision::CVImageFrame;

TEST(FACE_DET_TEST, Basic) {

  FasterRCNNMethod faster_rcnn_method;
  std::string config_file = "./configs/face_pose_lmk_config.json";
  faster_rcnn_method.Init(config_file);

  std::string img_list = "./test/data/image.list";
  std::ifstream ifs(img_list);
  ASSERT_TRUE(ifs.is_open());

  std::string input_image;
  while (getline(ifs, input_image)) {
    auto img_bgr = cv::imread(input_image);
    std::cout << "origin image size, width: " << img_bgr.cols << ", height: " << img_bgr.rows << std::endl;
    auto cv_image_frame_ptr = std::make_shared<CVImageFrame>();

    cv_image_frame_ptr->img = img_bgr;
    cv_image_frame_ptr->pixel_format = HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR;
    auto xroc_img = std::make_shared<XRocData<std::shared_ptr<ImageFrame>>>();
    xroc_img->value = cv_image_frame_ptr;

    std::vector<std::vector<BaseDataPtr>> input;
    std::vector<HobotXRoc::InputParamPtr> param;
    input.resize(1);
    input[0].push_back(xroc_img);
    std::vector<std::vector<BaseDataPtr>> xroc_output = faster_rcnn_method.DoProcess(input, param);
    ASSERT_TRUE(xroc_output.size() == 1);
    auto faster_rcnn_out = xroc_output[0];
    auto rects = std::static_pointer_cast<BaseDataVector>(faster_rcnn_out[0]);
    ASSERT_TRUE(rects->datas_.size() == 3);
  }

  faster_rcnn_method.Finalize();
}
