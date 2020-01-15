/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     order vector base data type
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.19
 */

#ifndef HOBOTXROC_DATA_TYPES_ARRAY_H_
#define HOBOTXROC_DATA_TYPES_ARRAY_H_

#include <cstdint>
#include <cstdlib>
#include <vector>

#include "hobotxsdk/xroc_data.h"

namespace HobotXRoc {

/**
 * @brief 一位数组
 *
 */
struct Array : public HobotXRoc::BaseData {
  /**
   * @brief Construct a new Array object
   *
   * @param values_size 数据长度
   * @param values 如果不为null则拷贝该内存
   */
  explicit Array(size_t values_size = 0, float* values = nullptr);
  ~Array();

  uint32_t class_ = 0;         //< 分类信息
  float score_ = 0.0;          //< 置信度
  std::vector<float> values_;  //< 数据指针
};

}  // namespace HobotXRoc

#endif  // HOBOTXROC_DATA_TYPES_ARRAY_H_
