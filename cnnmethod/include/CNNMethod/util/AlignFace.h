/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: AlignFace.h
 * @Brief: method of the AlignFace
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-16 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 16:01:54
 */
#ifndef INCLUDE_CNNMETHOD_UTIL_ALIGNFACE_H_
#define INCLUDE_CNNMETHOD_UTIL_ALIGNFACE_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace HobotXRoc {
#ifndef ELLISION
#define ELLISION 1e-5
#endif  // ifndef ELLISION

#ifndef isValidCoord
#define IsValidCoord(a) (!((ABS(a - (-1))) < (ELLISION)))
#endif  // ifndef isValidCoord

int AlignFace(const std::vector<float> &lmks_pts,
              const cv::Mat &origImg,
              cv::Mat &dstImg,
              float fill_value,
              std::vector<float> coord5points);

int GetAffinePoints(std::vector<float> &pts_in,
                    cv::Mat &trans,
                    std::vector<float> &pts_out);  // 5x3 x2x3；

int Cp2tform(std::vector<float> &uv, std::vector<float> &xy, cv::Mat &trans);

int FindNonReflectiveSimilarity(std::vector<float> &uv,
                                std::vector<float> &xy,
                                cv::Mat &T,
                                cv::Mat &Tinv);

}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_UTIL_ALIGNFACE_H_
