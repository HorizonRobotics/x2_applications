/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "hobotlog/hobotlog.hpp"
#include "yuv_utils.h"
#include "input_util/camera.h"

static void Usage() {
  std::cout << "./FasterRCNNMethod_example x2_dev_single_pyramid "
               "[get_vio_times]\n";
}

int TestX2DEVSinglePyramid(int argc, char **argv) {
  if (argc < 2) {
    Usage();
  }
  Camera single_camera(0, "./configs/hb_x2dev.json",
                       "./configs/vio_onsemi0230.json");
  img_info_t data;
  int get_vio_time = 1;
  if (argc > 1) {
    get_vio_time = atoi(argv[1]);
  }
  for (int i = 0; i < get_vio_time; ++i) {
    auto ret = single_camera.GetImage(&data);
    HOBOT_CHECK(ret == 0) << "single camera get image failed!!!";
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
    ret = single_camera.Free(&data);
    HOBOT_CHECK(ret == 0) << "single camera free image failed!!!";
  }
  return 0;
}




