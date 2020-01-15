//
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include "yuv_utils.h"
#include <iostream>

void yuv420sp_to_yuv444(uint8_t* yuv420sp, int height, int width, uint8_t* yuv444_ptr) {
  uint8_t* y_ptr = yuv420sp;
  uint8_t* uv_ptr = yuv420sp + height * width;

  // convert to yuv444 (YUV YUV YUV ...)
  int loop = height / 2;
  uint8_t *dptr = yuv444_ptr;
  uint8_t* duvptr = uv_ptr;
  uint8_t* dyptr = y_ptr;
  for (int i = 0; i < loop; ++i) {
    for (int j = 0; j < width; j+=2) {
      *(dptr++) = *(dyptr++);
      *(dptr++) = *(duvptr+j);
      *(dptr++) = *(duvptr+j+1);
      *(dptr++) = *(dyptr++);
      *(dptr++) = *(duvptr+j);
      *(dptr++) = *(duvptr+j+1);
    }
    for (int j = 0; j < width; j+=2) {
      *(dptr++) = *(dyptr++);
      *(dptr++) = *(duvptr+j);
      *(dptr++) = *(duvptr+j+1);
      *(dptr++) = *(dyptr++);
      *(dptr++) = *(duvptr+j);
      *(dptr++) = *(duvptr+j+1);
    }
    duvptr += width;
  }

  return;
}

void nv12_to_bgr(uint8_t *yuv420sp, int height, int width, cv::Mat &img_bgr) {
  // convert by cv
  cv::Mat ynv12(height * 3 / 2, width, CV_8UC1, yuv420sp);
  cv::cvtColor(ynv12, img_bgr, cv::COLOR_YUV2BGR_NV12);
}

void bgr_to_nv12(uint8_t *bgr, int height, int width, cv::Mat &img_nv12) {
  cv::Mat bgr_mat(height, width, CV_8UC3, bgr);
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);

  uint8_t *yuv = yuv_mat.ptr<uint8_t>();
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  uint8_t *nv12 = img_nv12.ptr<uint8_t>();

  int uv_height = height / 2;
  int uv_width = width / 2;
  // copy y data
  int y_size = uv_height * uv_width * 4;
  memcpy(nv12, yuv, y_size);

  // copy uv data
  int uv_stride = uv_width * uv_height;
  uint8_t *uv_data = nv12 + y_size;
  for (int i = 0; i < uv_stride; ++i) {
    *(uv_data++) = *(yuv + y_size + i);
    *(uv_data++) = *(yuv + y_size + +uv_stride + i);
  }
}

