/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc framework C interface
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.4
 */

#ifndef HOBOTXROC_C_DATA_TYPES_ARRAY_H_
#define HOBOTXROC_C_DATA_TYPES_ARRAY_H_

#include <stdint.h>
#include <stdlib.h>
#include "hobotxsdk/xroc_capi_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct HobotXRocCapiArray_ {
  HobotXRocCapiBaseData parent_;
  uint32_t class_;      //< 分类信息
  float score_;         //< 置信度
  float* values_;       //< 数据指针
  size_t values_size_;  //< 数据长度
} HobotXRocCapiArray;

HOBOT_EXPORT
HobotXRocCapiArray* HobotXRocCapiArrayAlloc();

HOBOT_EXPORT
void HobotXRocCapiArrayFree(HobotXRocCapiArray** array);

#define XROC_CAPI_INHERIT_FROM_ARRAY(ChildType, size)                           \
  typedef HobotXRocCapiArray HobotXRocCapi##ChildType;                          \
  inline HobotXRocCapi##ChildType* HobotXRocCapi##ChildType##Alloc() {          \
    HobotXRocCapi##ChildType* c_data = HobotXRocCapiArrayAlloc();               \
    c_data->values_size_ = size;                                                \
    c_data->values_ = (float*)calloc(size, sizeof(float));                      \
    c_data->parent_.type_ = #ChildType;                                         \
    return c_data;                                                              \
  }                                                                             \
  inline void HobotXRocCapi##ChildType##Free(HobotXRocCapi##ChildType** data) { \
    free((*data)->values_);                                                     \
    HobotXRocCapiArrayFree(data);                                               \
  }

#ifdef __cplusplus
}
#endif

#endif  // HOBOTXROC_C_DATA_TYPES_ARRAY_H_
