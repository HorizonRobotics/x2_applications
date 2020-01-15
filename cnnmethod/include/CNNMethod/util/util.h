/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @file      util.h
 * @author    hangjun.yang
 * @email     hangjun.yang@hobot.cc
 * @version   1.0.0.0
 * @date      2018.12.26
 */

#ifndef INCLUDE_CNNMETHOD_UTIL_UTIL_H_
#define INCLUDE_CNNMETHOD_UTIL_UTIL_H_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include "hobotlog/hobotlog.hpp"
#include "horizon/vision_type/vision_type.hpp"
#include "horizon/vision_type/vision_type_common.h"
#include "hb_vio_interface.h"

namespace HobotXRoc {

inline float SigMoid(const float input) {
  float ret = 1 / (1 + std::exp(-1 * input));
  return ret;
}

inline void SoftMax(std::vector<float> &inputs) {
  float sum = 0;
  for (auto &input : inputs) {
    sum += std::exp(input);
  }
  for (auto &input : inputs) {
    input = std::exp(input) / sum;
  }
}

template <typename T>
inline void Softmax(const std::vector<T> &energy, std::vector<T> *result) {
  if (nullptr == result || energy.empty() || result == &energy) {
    return;
  }
  result->resize(energy.size());
  T mmax = energy[0];
  for (int x = 1; x < energy.size(); ++x) {
    if (mmax < energy[x]) {
      mmax = energy[x];
    }
  }
  T sum = T(0.0f);
  for (int x = 0; x < result->size(); ++x) {
    (*result)[x] = std::exp(energy[x] - mmax);
    sum += (*result)[x];
  }
  for (int x = 0; x < result->size(); ++x) {
    (*result)[x] /= sum;
  }
}

// ToDo: 将bbox的x1，y1之类的修改掉
template <typename T>
void HNMS(std::vector<T> &candidates,
          std::vector<T> *result,
          const float overlap_ratio,
          const int top_N,
          const bool add_score,
          const float contain_ratio) {
  if (candidates.size() == 0) {
    return;
  }
  std::vector<bool> skip(candidates.size(), false);
  std::stable_sort(candidates.begin(), candidates.end(), T::greater);

  int count = 0;
  for (int i = 0; count < top_N && i < skip.size(); ++i) {
    if (skip[i]) continue;
    skip[i] = true;
    ++count;

    const float area_i = (candidates[i].x2_ - candidates[i].x1_)
                         * (candidates[i].y2_ - candidates[i].y1_);
    // suppress the significantly covered bbox
    for (int j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      // get intersections
      float xx1 = std::max(candidates[i].x1_, candidates[j].x1_);
      float yy1 = std::max(candidates[i].y1_, candidates[j].y1_);
      float xx2 = std::min(candidates[i].x2_, candidates[j].x2_);
      float yy2 = std::min(candidates[i].y2_, candidates[j].y2_);
      float area_intersection = (xx2 - xx1) * (yy2 - yy1);
      bool area_intersection_valid = (area_intersection > 0) && (xx2 - xx1 > 0);

      if (area_intersection_valid > 0) {
        // compute overlap
        float area_j = (candidates[j].x2_ - candidates[j].x1_)
                       * (candidates[j].y2_ - candidates[j].y1_);
        float o = area_intersection / area_j;
        if (o > overlap_ratio) {
          skip[j] = true;
          if (add_score) {
            candidates[i].conf_ += candidates[j].conf_;
          }
        }
        if (!skip[j] && contain_ratio < 1) {
          const float o = area_intersection / std::min(area_i, area_j);
          if (o >= contain_ratio) {
            skip[j] = true;
            if (add_score) {
              candidates[i].conf_ += candidates[j].conf_;
            }
          }
        }
      }
    }
    result->push_back(candidates[i]);
  }
  return;
}

template <typename T>
inline uint32_t ArgMax(const std::vector<T> &energy) {
  auto ele = std::max_element(energy.begin(), energy.end());
  return std::distance(energy.begin(), ele);
}

template <typename T>
T Mean(std::vector<T> v) {
  if (v.size() == 0) return 0;
  float tmp_x = 0;
  for (auto x : v) {
    tmp_x += x;
  }
  return tmp_x / v.size();
}

inline float GetFloatByInt(int32_t value, uint32_t shift) {
  float ret_x = value;
  if (value != 0) {
    int *ix = reinterpret_cast<int *>(&ret_x);
    (*ix) -= shift * 0x00800000;
  }
  return ret_x;
}

template <typename T>
void l2_norm(std::vector<T> &input, int length) {
  float sum = 0.0;
  float eps = 1e-10;
  for (int i = 0; i < length; ++i) {
    sum += input[i] * input[i];
  }
  sum = sqrt(sum) + eps;
  for (int i = 0; i < length; ++i) {
    input[i] = input[i] / sum;
  }
}

static void split_string(const std::string &s,
                         std::vector<std::string> &v,
                         const std::string &c) {
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;
  while (std::string::npos != pos2) {
    v.push_back(s.substr(pos1, pos2 - pos1));

    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if (pos1 != s.length()) {
    v.push_back(s.substr(pos1));
  }
}

static std::string get_parent_path(const std::string &path) {
  auto pos = path.rfind('/');
  if (std::string::npos != pos) {
    auto parent = path.substr(0, pos);
    return parent + "/";
  } else {
    return std::string("./");
  }
}

inline void DumpPyramid(img_info_t *pyramid, std::string f_name, int pyd_idx) {
  LOGD << "img size:";
  LOGD << "height:" << pyramid->down_scale[pyd_idx].height << " "
       << "width:" << pyramid->down_scale[pyd_idx].width << " "
       << "step:" << pyramid->down_scale[pyd_idx].step << std::endl;
  int img_size = pyramid->down_scale[pyd_idx].height
                 * pyramid->down_scale[pyd_idx].step * 3 / 2;
  int img_y_size =
      pyramid->down_scale[pyd_idx].height * pyramid->down_scale[pyd_idx].step;
  int img_uv_size = img_y_size / 2;
  std::ofstream fo(f_name, std::ios::binary);
  fo.write(reinterpret_cast<char *>(pyramid->down_scale[pyd_idx].y_vaddr),
           img_y_size);
  fo.write(reinterpret_cast<char *>(pyramid->down_scale[pyd_idx].c_vaddr),
           img_uv_size);
}

inline void DumpBinaryFile(void *data, int data_size, std::string file_name) {
  std::ofstream fo(file_name, std::ios::binary);
  fo.write(reinterpret_cast<char *>(data), data_size);
}

}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_UTIL_UTIL_H_
