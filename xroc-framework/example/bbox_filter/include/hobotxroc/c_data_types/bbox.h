/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc framework C interface
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.4
 */

#ifndef HOBOTXROC_C_DATA_TYPES_BBOX_H_
#define HOBOTXROC_C_DATA_TYPES_BBOX_H_

#include "array.h"
#include "hobotxsdk/xroc_capi_type.h"

#ifdef __cplusplus
extern "C" {
#endif

XROC_CAPI_INHERIT_FROM_ARRAY(BBox, 4)

#ifdef __cplusplus
}
#endif

#endif  // HOBOTXROC_C_DATA_TYPES_BBOX_H_
