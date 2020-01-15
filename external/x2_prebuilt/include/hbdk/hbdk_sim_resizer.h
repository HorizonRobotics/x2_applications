//===-------- hbdk_sim_resizer.h - resizer simulator interface ------*- C -*-===//
//
//                     The HBDK Simulator Infrastructure
//
// This file is subject to the terms and conditions defined in file
// 'LICENSE.txt', which is part of this source code package.
//
//===-----------------------------------------------------------------------===//

#pragma once

#include "hbdk_config.h"
#include "hbdk_error.h"
#include "hbdk_march.h"
#include "hbdk_type.h"

#ifdef __cplusplus
extern "C" {
#endif

HBDK_PUBLIC hbrt_error_t hbrtSimResizeYUV420(uint8_t* result, uint8_t* src_y, uint8_t* src_uv, uint32_t src_h,
                                             uint32_t src_w, uint32_t src_h_stride, int32_t roi_x_start,
                                             int32_t roi_y_start, uint32_t roi_x_size, uint32_t roi_y_size,
                                             uint32_t dst_w, uint32_t dst_h, MARCH march, bool ignore_uv,
                                             bool padding_neighbor);

#ifdef __cplusplus
}
#endif