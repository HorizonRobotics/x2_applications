/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <iostream>
#include <assert.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <string>
#include <fstream>

#include "yuv_utils.h"

#include "opencv2/opencv.hpp"

static void Usage() {
  std::cout << "./FasterRCNNMethod_example dump_nv12 image_list "
               "img_height img_width\n";
}

int TestDumpNV12(int argc, char **argv) {
  if (argc < 4) {
    Usage();
    return -1;
  }
  std::string image_list = argv[1];
  int img_height = atoi(argv[2]);
  int img_width = atoi(argv[3]);

  std::ifstream ifs(image_list);
  if (!ifs.is_open()) {
    std::cout << "open image list file failed." << std::endl;
    return -1;
  }
  std::string input_image;

  while (getline(ifs, input_image)) {
    auto img_bgr = cv::imread(input_image);
    cv::Mat img_nv12;
    uint8_t* bgr_data = img_bgr.ptr<uint8_t>();
    bgr_to_nv12(bgr_data, img_height, img_width, img_nv12);
    uint8_t* nv12_data = img_nv12.ptr<uint8_t>();
    std::cout << "origin image size, width: " << img_bgr.cols << ", height: " << img_bgr.rows << std::endl;

    std::string save_img_name = "./nv12_data/" + input_image + "_nv12.dat";
    FILE *fp = fopen(save_img_name.c_str(), "wb+");
    if (fp == nullptr) {
      std::cout << "open save file erro\n";
      return -1;
    }
    int nv12_len = img_width * img_height * 3 / 2;
    std::cout << "nv12 len: " << nv12_len << std::endl;
    fwrite(nv12_data, nv12_len, 1, fp);
    fclose(fp);
  }

  return 0;
}

