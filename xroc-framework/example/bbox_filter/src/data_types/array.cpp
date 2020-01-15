/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc framework C interface
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.4
 */

#include <stdlib.h>
#include <memory>

#include "hobotxroc/c_data_types/array.h"
#include "hobotxroc/c_struct_transfer/array_transfer.h"
#include "hobotxsdk/xroc_capi_type_helper.h"

XROC_DEFINE_DATA_FREE_PARENT(Array)

HobotXRocCapiArray* HobotXRocCapiArrayAlloc() {
  XROC_CAPI_BASE_ALLOC(Array, array);
  return array;
}

void HobotXRocCapiArrayFree(HobotXRocCapiArray** array) {
  if (*array) {
    XROC_CAPI_BASE_FREE(Array, array);
    free(*array);
    *array = nullptr;
  }
}

namespace HobotXRoc {

Array::Array(size_t values_size, float* values) : values_(values_size) {
  type_ = "Array";
  XROC_BASE_CPP_CONTEXT_INIT(Array, c_data_);
  if (values)
    values_.assign(values, values + values_size);
}

Array::~Array() {
}

}  // namespace HobotXRoc
