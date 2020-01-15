/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "./dump_util.h"
#include "./stopwatch.h"
#include "FasterRCNNMethod/FasterRCNNMethod.h"
#include "FasterRCNNMethod/faster_rcnn_imp.h"
#include "FasterRCNNMethod/yuv_utils.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxsdk/xroc_sdk.h"
#include "input_util/vio.h"
#include "json/json.h"
#include "opencv2/opencv.hpp"
#include "horizon/vision_type/vision_type.hpp"

using HobotXRoc::BaseData;
using HobotXRoc::BaseDataPtr;
using HobotXRoc::BaseDataVector;
using HobotXRoc::InputData;
using HobotXRoc::InputDataPtr;
using HobotXRoc::XRocData;

using HobotXRoc::FasterRCNNMethod;
using hobot::vision::ImageFrame;
using hobot::vision::PymImageFrame;

static void Usage() {
  std::cout << "./FasterRCNNMethod_example dump_face_det_result image_list\n";
}

int DumpFaceDetResult(int argc, char **argv) {
  if (argc < 2) {
    Usage();
    return -1;
  }
  FasterRCNNMethod faster_rcnn_method;
  std::string image_list = argv[1];
  std::string model_config_file = "./configs/face_config.json";

  faster_rcnn_method.Init(model_config_file);

  std::ifstream ifs(image_list);
  if (!ifs.is_open()) {
    std::cout << "open image list file failed." << std::endl;
    return -1;
  }
  std::string input_image;

  VIO fb_vio;
  auto ret = fb_vio.Init("./configs/vio_onsemi0230_fb.json");
  HOBOT_CHECK(ret == 0) << "fb vio init failed!!!";
  fb_vio.Start();
  HOBOT_CHECK(ret == 0) << "fb vio start failed!!!";
  img_info_t data;

  std::ofstream ofs("face_det_result.txt");
  while (getline(ifs, input_image)) {
    cv::Mat bgr_img = cv::imread(input_image);
    int width = bgr_img.cols;
    int height = bgr_img.rows;
    cv::Mat img_nv12;
    bgr_to_nv12(bgr_img.data, height, width, img_nv12);
    ret = fb_vio.GetFBImage(&data, img_nv12.data, height * width * 3 / 2);
    HOBOT_CHECK(ret == 0) << "fb vio get image failed!!!";
    // vio_debug::print_info(data);

    auto py_image_frame_ptr = std::make_shared<PymImageFrame>();
    py_image_frame_ptr->img = data;

    auto xroc_pyramid =
        std::make_shared<XRocData<std::shared_ptr<ImageFrame>>>();
    xroc_pyramid->value = py_image_frame_ptr;

    std::vector<std::vector<BaseDataPtr>> input;
    std::vector<HobotXRoc::InputParamPtr> param;
    input.resize(1);
    input[0].push_back(xroc_pyramid);
    auto xroc_output = faster_rcnn_method.DoProcess(input, param);
    assert(xroc_output.size() == 1);
    auto faster_rcnn_out = xroc_output[0];
    static int frame_cnt = 0;
    ofs << input_image;
    for (auto &in_rect :
        std::static_pointer_cast<BaseDataVector>(faster_rcnn_out[0])->datas_) {
      auto rect = std::static_pointer_cast<XRocData<BBox>>(in_rect);
      ofs << " " << rect->value.x1 << " " << rect->value.y1 << " "
          << rect->value.x2 << " " << rect->value.y2 << " "
          << rect->value.score;
    }
    ofs << "\n";
    ret = fb_vio.Free(&data);
    HOBOT_CHECK(ret == 0) << "fb vio free image failed!!!";
    LOGD << "predict success: " << ++frame_cnt;
  }
  fb_vio.Stop();
  faster_rcnn_method.Finalize();
  return 0;
}
