/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc framework C interface
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.4
 */

#ifndef TEST_INCLUDE_HOBOTXROC_C_STRUCT_TRANSFER_BBOX_TRANSFER_H_
#define TEST_INCLUDE_HOBOTXROC_C_STRUCT_TRANSFER_BBOX_TRANSFER_H_

#include <memory>

#include "./array_transfer.h"
#include "hobotxroc/c_data_types/bbox.h"
#include "hobotxroc/data_types/bbox.h"
#include "hobotxsdk/xroc_capi_type_helper.h"

namespace HobotXRoc {

XROC_ARRAY_CHILD_TRANSFER(BBox);

}  // namespace HobotXRoc

#endif  // TEST_INCLUDE_HOBOTXROC_C_STRUCT_TRANSFER_BBOX_TRANSFER_H_
