/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc framework C interface
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.4
 */

#ifndef HOBOTXROC_C_STRUCT_TRANSFER_ARRAY_TRANSFER_H_
#define HOBOTXROC_C_STRUCT_TRANSFER_ARRAY_TRANSFER_H_

#include <memory>

#include "hobotxroc/c_data_types/array.h"
#include "hobotxroc/data_types/array.h"
#include "hobotxsdk/xroc_capi_type_helper.h"

namespace HobotXRoc {

XROC_DECLEAR_TRANSFER(Array);

#define XROC_ARRAY_CHILD_TRANSFER(ChildType)                                         \
  inline XROC_DEFINE_2C_TRANSFER_FUNC(ChildType, cpp_data) {                         \
    XROC_BASE_2C_TRANSFER_PROCESS(ChildType, cpp_data, c_data);                      \
                                                                                     \
    auto& in = cpp_data;                                                             \
    auto& out = c_data;                                                              \
    out->values_ = &(in->values_[0]);                                                \
    out->values_size_ = in->values_.size();                                          \
    out->class_ = in->class_;                                                        \
    out->score_ = in->score_;                                                        \
                                                                                     \
    return out;                                                                      \
  }                                                                                  \
                                                                                     \
  inline XROC_DEFINE_2CPP_TRANSFER_FUNC(ChildType, c) {                              \
    XROC_BASE_2CPP_TRANSFER_PROCESS(ChildType, c, cpp, c->values_size_, c->values_); \
    auto& in = c;                                                                    \
    auto& out = cpp;                                                                 \
    out->class_ = in->class_;                                                        \
    out->score_ = in->score_;                                                        \
                                                                                     \
    return out;                                                                      \
  }

XROC_ARRAY_CHILD_TRANSFER(Array)

}  // namespace HobotXRoc

#endif  // HOBOTXROC_C_STRUCT_TRANSFER_ARRAY_TRANSFER_H_
