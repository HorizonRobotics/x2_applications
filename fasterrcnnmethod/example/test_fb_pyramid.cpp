/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "hobotlog/hobotlog.hpp"
#include "yuv_utils.h"
#include "input_util/vio.h"

static void Usage() {
  std::cout << "./FasterRCNNMethod_example fb_pyramid image_list\n";
}

int TestFBPyramid(int argc, char **argv) {
  if (argc < 2) {
    Usage();
    return -1;
  }

  std::string image_list = argv[1];
  while (true) {
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
    while (getline(ifs, input_image)) {
      cv::Mat bgr_img = cv::imread(input_image);
      int width = bgr_img.cols;
      int height = bgr_img.rows;
      cv::Mat img_nv12;
      bgr_to_nv12(bgr_img.data, height, width, img_nv12);

      ret = fb_vio.GetFBImage(&data, img_nv12.data, height * width * 3 / 2);
      HOBOT_CHECK(ret == 0) << "fb vio get image failed!!!";
      vio_debug::print_info(data);
#if 1
    int img_height = data.src_img.height;
    int img_witdh = data.src_img.width;
    std::cout << "img" <<": height: " << img_height << "width: "
              << img_witdh << std::endl;
    int img_y_len = img_height * img_witdh;
    int img_uv_len = img_height * img_witdh / 2;
    uint8_t *img_ptr = static_cast<uint8_t*>(malloc(img_y_len + img_uv_len));
    memcpy(img_ptr, reinterpret_cast<uint8_t *>(data.src_img.y_vaddr),
           img_y_len);
    memcpy(img_ptr + img_y_len,
           reinterpret_cast<uint8_t *>(data.src_img.c_vaddr), img_uv_len);
    cv::Mat bgr_mat;
    nv12_to_bgr(img_ptr, img_height, img_witdh, bgr_mat);
    free(img_ptr);
    cv::imwrite("pyramid_img_single.jpg", bgr_mat);
    for (int k = 0; k < 5; ++k) {
      int ds_img_height = data.down_scale[4*k].height;
      int ds_img_witdh = data.down_scale[4*k].step;

      std::cout << "ds_img" << std::to_string(k) <<": height: " << ds_img_height
                << " width: " << ds_img_witdh << std::endl;
      int ds_img_y_len = ds_img_height * ds_img_witdh;
      int ds_img_uv_len = ds_img_height * ds_img_witdh / 2;
      uint8_t *ds_img_ptr = static_cast<uint8_t*>(malloc(
          ds_img_y_len + ds_img_uv_len));
      memcpy(ds_img_ptr,
             reinterpret_cast<uint8_t *>(data.down_scale[4 * k].y_vaddr),
             ds_img_y_len);
      memcpy(ds_img_ptr + ds_img_y_len,
             reinterpret_cast<uint8_t *>(data.down_scale[4 * k].c_vaddr),
             ds_img_uv_len);
      cv::Mat ds_bgr_mat;
      nv12_to_bgr(ds_img_ptr, ds_img_height, ds_img_witdh, ds_bgr_mat);
      free(ds_img_ptr);
      cv::imwrite("ds_pyramid_img" + std::to_string(k) + ".jpg", ds_bgr_mat);
    }
#endif

      ret = fb_vio.Free(&data);
      HOBOT_CHECK(ret == 0) << "fb vio free image failed!!!";
    }
    fb_vio.Stop();
  }
  return 0;
}

