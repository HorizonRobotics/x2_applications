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
#include <string>

#include "opencv2/opencv.hpp"
#include "yuv_utils.h"

#include "input_util/camera.h"

static void Usage() {
  std::cout << "./FasterRCNNMethod_example x2_dev_pyramid [get_vio_times]\n";
}

int TestX2DEVDualPyramid(int argc, char **argv) {
  if (argc < 2) {
    Usage();
  }
  Camera dual_camera(2, "./configs/hb_x2dev.json",
                     "./configs/vio_onsemi_dual.json");

  int get_vio_time = 1;
  if (argc > 1) {
    get_vio_time = atoi(argv[1]);
  }
  mult_img_info_t data;
  for (int i = 0; i < get_vio_time; ++i) {
    auto ret = dual_camera.GetMultiImage(&data);
    HOBOT_CHECK(ret == 0) << "dual camera get multi image failed!!!";
    std::string image_type;
    for (int j = 0; j < 2; ++j) {
      vio_debug::print_info(data.img_info[j]);
      auto img = data.img_info[j];
      if (j == 0) {
        image_type = "RGB";
        printf("rgb frame id: %d\n", img.frame_id);
      } else {
        image_type = "NIR";
        printf("nir frame id: %d\n", img.frame_id);
      }
      int img_height = img.src_img.height;
      int img_witdh = img.src_img.step;
      std::cout << "img" << std::to_string(j) <<": height: "
                << img_height << " width: " << img_witdh
                << " timestamp: " << data.img_info[j].timestamp << std::endl;
      int img_y_len = img_height * img_witdh;
      int img_uv_len = img_height * img_witdh / 2;
      uint8_t *img_ptr = static_cast<uint8_t*>(malloc(img_y_len + img_uv_len));
      memcpy(img_ptr, reinterpret_cast<uint8_t *>(img.src_img.y_vaddr),
             img_y_len);
      memcpy(img_ptr + img_y_len,
             reinterpret_cast<uint8_t *>(img.src_img.c_vaddr), img_uv_len);
      cv::Mat bgr_mat;
      nv12_to_bgr(img_ptr, img_height, img_witdh, bgr_mat);
      free(img_ptr);
      cv::imwrite("src_" + image_type + "_" + std::to_string(img.frame_id) +
          "_" + std::to_string(j) + ".jpg", bgr_mat);
      for (int k = 0; k < 5; ++k) {
        int ds_img_height = img.down_scale[4*k].height;
        int ds_img_witdh = img.down_scale[4*k].step;
        std::cout << "ds_img" << std::to_string(k) <<": height: "
                  << ds_img_height << " width: " << ds_img_witdh << std::endl;
        int ds_img_y_len = ds_img_height * ds_img_witdh;
        int ds_img_uv_len = ds_img_height * ds_img_witdh / 2;
        uint8_t *ds_img_ptr = static_cast<uint8_t*>(malloc(
            ds_img_y_len + ds_img_uv_len));
        memcpy(ds_img_ptr,
               reinterpret_cast<uint8_t *>(img.down_scale[4 * k].y_vaddr),
               ds_img_y_len);
        memcpy(ds_img_ptr + ds_img_y_len,
               reinterpret_cast<uint8_t *>(img.down_scale[4 * k].c_vaddr),
               ds_img_uv_len);
        cv::Mat ds_bgr_mat;
        nv12_to_bgr(ds_img_ptr, ds_img_height, ds_img_witdh, ds_bgr_mat);
        free(ds_img_ptr);
        cv::imwrite(std::to_string(img.frame_id) +
            "_ds_pyramid_img" + std::to_string(k) + ".jpg", ds_bgr_mat);
      }
    }
    ret = dual_camera.Free(&data);
    HOBOT_CHECK(ret == 0) << "dual camera free multi image failed!!!";
  }

  return 0;
}




