/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: util.h
 * @Brief: declaration of the util
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-04 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-04 16:16:58
 */

#ifndef EXAMPLE_INCLUDE_UTIL_UTIL_H_
#define EXAMPLE_INCLUDE_UTIL_UTIL_H_

#include <opencv2/opencv.hpp>

inline void TransImage(cv::Mat *src_mat,
                       cv::Mat *dst_mat,
                       uint32_t dst_w,
                       uint32_t dst_h,
                       uint32_t *effective_w,
                       uint32_t *effective_h) {
  if (src_mat->rows == dst_h && src_mat->cols == dst_w) {
    *dst_mat = src_mat->clone();
    *effective_h = dst_h;
    *effective_w = dst_w;
  } else {
    *dst_mat = cv::Mat(dst_h, dst_w, CV_8UC3, cv::Scalar::all(0));
    int x2 = dst_w < src_mat->cols ? dst_w : src_mat->cols;
    int y2 = dst_h < src_mat->rows ? dst_h : src_mat->rows;
    (*src_mat)(cv::Rect(0, 0, x2, y2))
        .copyTo((*dst_mat)(cv::Rect(0, 0, x2, y2)));
    *effective_w = x2;
    *effective_h = y2;
  }
}

#endif  // EXAMPLE_INCLUDE_UTIL_UTIL_H_
