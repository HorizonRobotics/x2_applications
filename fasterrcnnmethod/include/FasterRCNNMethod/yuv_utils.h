//
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef INCLUDE_FASTERRCNNMETHOD_YUV_UTILS_H_
#define INCLUDE_FASTERRCNNMETHOD_YUV_UTILS_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

void yuv420sp_to_yuv444(uint8_t* yuv420sp, int height, int width, uint8_t* yuv444_ptr);

void nv12_to_bgr(uint8_t *yuv420sp, int height, int width, cv::Mat &img_bgr);

void bgr_to_nv12(uint8_t *bgr, int height, int width, cv::Mat &img_nv12);

#endif // INCLUDE_FASTERRCNNMETHOD_YUV_UTILS_H_
