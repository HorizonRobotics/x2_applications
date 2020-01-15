/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: AlignFace.cpp
 * @Brief: definition of the AlignFace
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-16 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 16:17:08
 */

#include "CNNMethod/util/AlignFace.h"

namespace HobotXRoc {

int AlignFace(const std::vector<float> &lmks_pts,
              const cv::Mat &origImg,
              cv::Mat &dstImg,
              float fill_value,
              std::vector<float> coord5points) {
  std::vector<float> trans_pts;
  std::vector<float> src;
  std::vector<float> dst;
  assert(lmks_pts.size() == coord5points.size());
  dst.resize(lmks_pts.size());
  src.resize(lmks_pts.size());
  for (int i = 0; i < lmks_pts.size(); ++i) {
    dst[i] = coord5points[i];
    src[i] = lmks_pts[i];
  }
  cv::Mat trans(3, 2, CV_32F);
  int ret = Cp2tform(src, dst, trans);
  if (ret == 0) return 0;
  int channel = dstImg.channels();
  float value = fill_value;
  if (channel == 3) {
    cv::warpAffine(origImg,
                   dstImg,
                   trans,
                   cv::Size(dstImg.cols, dstImg.rows),
                   cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(value, value, value));
  } else {
    cv::warpAffine(origImg,
                   dstImg,
                   trans,
                   cv::Size(dstImg.cols, dstImg.rows),
                   cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(value));
  }
  GetAffinePoints(src, trans, trans_pts);
  return 1;
}

int FindNonReflectiveSimilarity(std::vector<float> &uv,
                                std::vector<float> &xy,
                                cv::Mat &T,
                                cv::Mat &Tinv) {
  assert(uv.size() == xy.size());
  assert(uv.size() % 2 == 0);
  assert(uv.size() / 2 >= 3);
  int nPtNum = uv.size() / 2;
  cv::Mat X(nPtNum * 2, 4, CV_32F);
  for (int row = 0; row < nPtNum * 2; row++) {
    for (int col = 0; col < 4; col++) {
      if (row <= nPtNum - 1) {
        if (col <= 1) {
          X.at<float>(row, col) = xy[row * 2 + col];
        } else {
          X.at<float>(row, col) = 1 * (col - 1) % 2;
        }
      } else {
        if (col <= 1) {
          X.at<float>(row, col) =
              xy[(row - nPtNum) * 2 + 1 - col] * (1 - 2 * col);
        } else {
          X.at<float>(row, col) = 1 * (col % 2);
        }
      }
    }
  }
  int size = nPtNum * 2;
  // cv::Mat U(nPtNum * 2, CV_32F);
  cv::Mat U(1, &size, CV_32F);
  for (int row = 0; row < 2 * nPtNum; row++) {
    if (row < nPtNum) {
      U.at<float>(row) = uv[2 * row];
    } else {
      U.at<float>(row) = uv[2 * (row - nPtNum) + 1];
    }
  }

  // Least Squares Solution
  cv::Mat X_trans;
  cv::transpose(X, X_trans);
  cv::Mat X_trans_X = X_trans * X;
  cv::Mat inv_mat;
  double ret = cv::invert(X_trans_X, inv_mat);
  if (ret == 0.0) {
    return 0;
  } else {
    inv_mat = inv_mat * X_trans;
    inv_mat = inv_mat * U;
    cv::Mat &r = inv_mat;
    float sc = r.at<float>(0, 0);
    float ss = r.at<float>(1, 0);
    float tx = r.at<float>(2, 0);
    float ty = r.at<float>(3, 0);
    float trans_final[] = {sc, -1 * ss, 0, ss, sc, 0, tx, ty, 1};
    // cv::Mat Tinv(3, 3, CV_32F, trans_final);
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        Tinv.at<float>(row, col) = trans_final[row * 3 + col];
      }
    }

    double ret = cv::invert(Tinv, T);
    assert(ret != 0.0);
    T.at<float>(0, 2) = 0.0;
    T.at<float>(1, 2) = 0.0;
    T.at<float>(2, 2) = 1.0;
    return 1;
  }
}

int Cp2tform(std::vector<float> &uv, std::vector<float> &xy, cv::Mat &trans) {
  assert(uv.size() == xy.size() && uv.size() >= 3 * 2);
  cv::Mat T(3, 3, CV_32F), Tinv(3, 3, CV_32F);
  int ret = FindNonReflectiveSimilarity(uv, xy, T, Tinv);
  if (ret == 0) return 0;
  // cv::Mat trans(3, 2, CV_32F);
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 2; col++) {
      trans.at<float>(row, col) = T.at<float>(row, col);
    }
  }
  cv::transpose(trans, trans);
  return 1;
}

int GetAffinePoints(std::vector<float> &pts_in,
                    cv::Mat &trans,
                    std::vector<float> &pts_out) {
  pts_out.resize(pts_in.size());
  for (int i = 0; i < pts_in.size(); i += 2) {
    pts_out[i] = pts_in[i] * trans.at<float>(0, 0)
                 + pts_in[i + 1] * trans.at<float>(0, 1)
                 + trans.at<float>(0, 2);
    pts_out[i + 1] = pts_in[i] * trans.at<float>(1, 0)
                     + pts_in[i + 1] * trans.at<float>(1, 1)
                     + trans.at<float>(1, 2);
  }
  return 1;
}
}  // namespace HobotXRoc
