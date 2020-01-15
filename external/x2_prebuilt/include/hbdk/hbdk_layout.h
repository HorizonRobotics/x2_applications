//===- hbdk_layout.h - Layout definition and helper function ------*- C -*-===//
//
//                     The HBDK Compiler Infrastructure
//
// This file is subject to the terms and conditions defined in file
// 'LICENSE.txt', which is part of this source code package.
//
//===----------------------------------------------------------------------===//
/// \file
/// The naive way to place a tensor on the memory storage is looping through its
/// NHWC dimension one by one, as shown by following C code.
///
///     for (int n = 0; n < N; ++n)
///       for (int h = 0; h < H; ++h)
///         for (int w = 0; w < W; ++w)
///           for (int c = 0; c < C; ++c)
///
/// In order to maximize MAC utilization, the tensor would be layout in a few
/// specific orders, based on what loop tiling and stride are choosed. The layout
/// could be represented by format string, X_Y_S_P.
///
///  - X means in which order we loop through NHWC.
///  - Y means the tiling size of each dimension, also means the order we loop
///    through them.
///  - S means the stride.
///  - P means the PE number and extend direction.
///
/// For example, NHCW_1N1H8W4C_S1 could be represented by C code below.
///
///     for (int n = 0; n < ceil(N/1); ++n)
///       for (int h = 0; h < ceil(H/1); ++h)
///         for (int c = 0; c < ceil(C/4); ++c)
///           for (int w = 0; w < ceil(W/8); ++w)
///             for (int nn = 0; nn < 1; ++nn)
///               for (int hh = 0; hh < 1; ++hh)
///                 for (int ww = 0; ww < 8; ++ww)
///                   for (int cc = 0; cc < 4; ++cc)
///
/// Refer to the following links for more information.
///
///   1. http://wiki.hobot.cc/pages/viewpage.action?pageId=19499317
///   2. http://wiki.hobot.cc/pages/viewpage.action?pageId=30846061
///
//===----------------------------------------------------------------------===//
#pragma once

#include "hbdk_config.h"
#include "hbdk_type.h"
#include <assert.h>   // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)
#include <stdbool.h>  // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)
#include <stdint.h>   // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)
#include <stdlib.h>   // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  LAYOUT_ORDER_NHWC = 0,  ///< NHWC
  LAYOUT_ORDER_NHCW,      ///< HNCW
  LAYOUT_ORDER_NCHW,      ///< NCHW
  LAYOUT_ORDER_NCWH,      ///< NCWH
} hbrt_layout_order_t;

/**
 * This enum class represents the layout for (convolution) stride 2 (s2).
 * For target supporting s2, the layout could be one of the following,
 *
 *   - LAYOUT_STRIDE2_MODE_W_MAPPING
 *
 *   - LAYOUT_STRIDE2_MODE_DOUBLE_BLOCK
 */
typedef enum {
  LAYOUT_STRIDE2_MODE_W_MAPPING = 0,  ///< stride 2
  LAYOUT_STRIDE2_MODE_DOUBLE_BLOCK,   ///< stride 2
} layout_stride2_mode_t;

static const uint32_t LAYOUT_NAME_MAX = 32;  ///< The maximum length of the name of layout
/**
 * This enum class is corresponding to the layout format string.
 * Must be in the same order of hbdk_layout.def
 */
typedef enum {
  LAYOUT_NHWC_NATIVE = 0,
  LAYOUT_NHCW_NATIVE,
  LAYOUT_NCHW_NATIVE,

  /**
   * P1
   */
  LAYOUT_NCHW_2N32C,       ///< side buffer for 1 PE
  LAYOUT_NCHW_2N32C_2PEN,  ///< side buffer for 2 PE
  LAYOUT_NCHW_2N32C_4PEN,  ///< side buffer for 4 PE

  LAYOUT_NHWC_4W4C_2PEC,   ///< p1 feature (normal & dws)
  LAYOUT_NHWC_16C_2PEC,    ///< p1 feature (fc)
  LAYOUT_NHWC_4C_2PEC,     ///< p1 int32_t feature
  LAYOUT_NHWC_4N4C_2PEN,   ///< p1 weight (normal)
  LAYOUT_NHWC_8N_2PEN,     ///< p1 weight (dws)
  LAYOUT_NHWC_4N16C_2PEN,  ///< p1 weight (fc)
  LAYOUT_NHWC_4N_2PEN,     ///< p1 bias

  /**
   * X2
   */
  LAYOUT_NHCW_32C,         ///< x2 and x3 feature (fc)
  LAYOUT_NHCW_8W4C,        ///< x2 feature (normal)
  LAYOUT_NHCW_8W4C_S2D,    ///< x2 feature (normal)
  LAYOUT_NHCW_16W16C,      ///< x2 feature (dws)
  LAYOUT_NHCW_16W16C_S2D,  ///< x2 feature (dws)
  LAYOUT_NCHW_8C,          ///< x2 feature (int32_t)
  LAYOUT_NCHW_8N4C,        ///< x2 weight (normal)
  LAYOUT_NCHW_16N,         ///< x2 weight (dws)
  LAYOUT_NCHW_8N32C,       ///< x2 weight (fc 2)
  LAYOUT_NCHW_8N8W4C,      ///< x2 weight (fc 1)
  LAYOUT_NHWC_4N,          ///< x2 bias

  /**
   * X3
   */

  // warping Y/UV layout, must be prior to feature layouts
  LAYOUT_NHCW_64W,         ///< x3 warping (Y mode), input Y, 8 slice, for co-operation mode (all PEs do one Y)
  LAYOUT_NHCW_64W_2PEN,    ///< x3 warping (Y mode), input Y, 8 slice, for 2 PE batch mode
  LAYOUT_NHCW_64W_4PEN,    ///< x3 warping (Y mode), input Y, 8 slice, for 4 PE batch mode
  LAYOUT_NHCW_64W_8PEN,    ///< x3 warping (Y mode), input Y, 8 slice, for 8 PE batch mode
  LAYOUT_NHCW_32W2C,       ///< x3 warping (UV mode), input UV, 8 slice, for co-operation mode (all PEs do one UV)
  LAYOUT_NHCW_32W2C_2PEN,  ///< x3 warping (UV mode), input UV, 8 slice, for 2 PE batch mode
  LAYOUT_NHCW_32W2C_4PEN,  ///< x3 warping (UV mode), input UV, 8 slice, for 4 PE batch mode
  LAYOUT_NHCW_32W2C_8PEN,  ///< x3 warping (UV mode), input UV, 8 slice, for 8 PE batch mode

  LAYOUT_NHCW_2H4W,       ///< x3 warping (Y mode), output Y, 1 slice
  LAYOUT_NHCW_2H4W_2PEW,  ///< x3 warping (Y mode), output Y, 1 slice, for co-operation mode (all PEs do one Y)
  LAYOUT_NHCW_2H4W_4PEW,  ///< x3 warping (Y mode), output Y, 1 slice, for co-operation mode (all PEs do one Y)
  LAYOUT_NHCW_2H4W_8PEW,  ///< x3 warping (Y mode), output Y, 1 slice, for co-operation mode (all PEs do one Y)
  LAYOUT_NHCW_2H4W_2PEN,  ///< x3 warping (Y mode), output Y, 1 slice, for 2 PE batch mode
  LAYOUT_NHCW_2H4W_4PEN,  ///< x3 warping (Y mode), output Y, 1 slice, for 4 PE batch mode
  LAYOUT_NHCW_2H4W_8PEN,  ///< x3 warping (Y mode), output Y, 1 slice, for 8 PE batch mode

  LAYOUT_NHCW_2H2W2C,       ///< x3 warping (UV mode), output UV, 1 slice
  LAYOUT_NHCW_2H2W2C_2PEW,  ///< x3 warping (UV mode), output UV, 1 slice, for co-operation mode (all PEs do one UV)
  LAYOUT_NHCW_2H2W2C_4PEW,  ///< x3 warping (UV mode), output UV, 1 slice, for co-operation mode (all PEs do one UV)
  LAYOUT_NHCW_2H2W2C_8PEW,  ///< x3 warping (UV mode), output UV, 1 slice, for co-operation mode (all PEs do one UV)
  LAYOUT_NHCW_2H2W2C_2PEN,  ///< x3 warping (UV mode), output UV, 1 slice, for 2 PE batch mode
  LAYOUT_NHCW_2H2W2C_4PEN,  ///< x3 warping (UV mode), output UV, 1 slice, for 4 PE batch mode
  LAYOUT_NHCW_2H2W2C_8PEN,  ///< x3 warping (UV mode), output UV, 1 slice, for 8 PE batch mode

  LAYOUT_NHCW_4H4W8C,       ///< x3 feature 1PE
  LAYOUT_NHCW_4H4W8C_2PEN,  ///< x3 feature 2PE, each PE process its own feature
  LAYOUT_NHCW_4H4W8C_4PEN,  ///< x3 feature 4PE, each PE process its own feature
  LAYOUT_NHCW_4H4W8C_8PEN,  ///< x3 feature 8PE, each PE process its own feature
  LAYOUT_NHCW_4H4W8C_2PEW,  ///< x3 feature 2PE, for channel-dependent operations (e.g., convolution)
  LAYOUT_NHCW_4H4W8C_4PEW,  ///< x3 feature 4PE, for channel-dependent operations (e.g., convolution)
  LAYOUT_NHCW_4H4W8C_8PEW,  ///< x3 feature 8PE, for channel-dependent operations (e.g., convolution)
  LAYOUT_NHCW_4H4W8C_2PEC,  ///< x3 feature 2PE, for channel-independent operations (e.g., pooling, roi resize)
  LAYOUT_NHCW_4H4W8C_4PEC,  ///< x3 feature 4PE, for channel-independent operations (e.g., pooling, roi resize)
  LAYOUT_NHCW_4H4W8C_8PEC,  ///< x3 feature 8PE, for channel-independent operations (e.g., pooling, roi resize)

  // LAYOUT_NHCW_32C,       ///< x3 FC convolution's feature (1x1) for 1 PE, same as X2
  LAYOUT_NHCW_32C_2PEC,  ///< x3 FC convolution's feature (1x1) for 2 PE
  LAYOUT_NHCW_32C_4PEC,  ///< x3 FC convolution's feature (1x1) for 4 PE
  LAYOUT_NHCW_32C_8PEC,  ///< x3 FC convolution's feature (1x1) for 8 PE
  LAYOUT_NHCW_32C_2PEN,  ///< x3 FC convolution's feature (1x1), separate feature, 2PE
  LAYOUT_NHCW_32C_4PEN,  ///< x3 FC convolution's feature (1x1), separate feature, 4PE
  LAYOUT_NHCW_32C_8PEN,  ///< x3 FC convolution's feature (1x1), separate feature, 8PE

  LAYOUT_NHCW_4W8C,       ///< x3 FC convolution's input (when H or W is not 1) for 1 PE,
  LAYOUT_NHCW_4W8C_2PEW,  ///< x3 FC convolution's input (when H or W is not 1) for 2 PE,
  LAYOUT_NHCW_4W8C_4PEW,  ///< x3 FC convolution's input (when H or W is not 1) for 4 PE,
  LAYOUT_NHCW_4W8C_8PEW,  ///< x3 FC convolution's input (when H or W is not 1) for 8 PE,
  LAYOUT_NHCW_4W8C_2PEN,  ///< x3 FC convolution's input (when H or W is not 1), separate feature, 2PE
  LAYOUT_NHCW_4W8C_4PEN,  ///< x3 FC convolution's input (when H or W is not 1), separate feature, 4PE
  LAYOUT_NHCW_4W8C_8PEN,  ///< x3 FC convolution's input (when H or W is not 1), separate feature, 8PE

  LAYOUT_NHCW_4W8C_2PEC,  ///< x3 global average pooling's partial sum for 2 PE,
  LAYOUT_NHCW_4W8C_4PEC,  ///< x3 global average pooling's partial sum for 4 PE,
  LAYOUT_NHCW_4W8C_8PEC,  ///< x3 global average pooling's partial sum for 8 PE,

  LAYOUT_NHCW_8W8C,       ///< x3 global average pooling's 16bit output for 1 PE,
  LAYOUT_NHCW_8W8C_2PEC,  ///< x3 global average pooling's 16bit output for 2 PE,
  LAYOUT_NHCW_8W8C_4PEC,  ///< x3 global average pooling's 16bit output for 4 PE,
  LAYOUT_NHCW_8W8C_8PEC,  ///< x3 global average pooling's 16bit output for 8 PE,
  LAYOUT_NHCW_8W8C_2PEN,  ///< x3 global average pooling's 16bit output for 2 PE,
  LAYOUT_NHCW_8W8C_4PEN,  ///< x3 global average pooling's 16bit output for 4 PE,
  LAYOUT_NHCW_8W8C_8PEN,  ///< x3 global average pooling's 16bit output for 8 PE,

  LAYOUT_NCWH_ZHW_8N8C,       ///< x3 normal convolution's weight for 1 PE
  LAYOUT_NCWH_ZHW_4N8C_2PEN,  ///< x3 normal convolution's weight for 2 PE
  LAYOUT_NCWH_ZHW_2N8C_4PEN,  ///< x3 normal convolution's weight for 4 PE
  LAYOUT_NCWH_ZHW_8C_8PEN,    ///< x3 normal convolution's weight for 8 PE

  LAYOUT_NCWH_ZHW_64N,       ///< x3 depth-wise convolution's weight for 1 PE
  LAYOUT_NCWH_ZHW_32N_2PEN,  ///< x3 depth-wise convolution's weight for 2 PE
  LAYOUT_NCWH_ZHW_16N_4PEN,  ///< x3 depth-wise convolution's weight for 4 PE
  LAYOUT_NCWH_ZHW_8N_8PEN,   ///< x3 depth-wise convolution's weight for 8 PE

  LAYOUT_NCWH_ZHW_16N4C,      ///< x3 folded depth-wise convolution's weight for 1 PE
  LAYOUT_NCWH_ZHW_8N4C_2PEN,  ///< x3 folded depth-wise convolution's weight for 2 PE
  LAYOUT_NCWH_ZHW_4N4C_4PEN,  ///< x3 folded depth-wise convolution's weight for 4 PE
  LAYOUT_NCWH_ZHW_2N4C_8PEN,  ///< x3 folded depth-wise convolution's weight for 8 PE

  LAYOUT_NCHW_32N16C,       ///< x3 FC convolutions's weight (1x1) for 1 PE
  LAYOUT_NCHW_32N16C_2PEN,  ///< x3 FC convolutions's weight (1x1) for 2 PE
  LAYOUT_NCHW_32N16C_4PEN,  ///< x3 FC convolutions's weight (1x1) for 4 PE
  LAYOUT_NCHW_32N16C_8PEN,  ///< x3 FC convolutions's weight (1x1) for 8 PE

  LAYOUT_NCHW_32N2W8C,       ///< x3 FC convolutions's weight (when H or W is not 1) for 1 PE
  LAYOUT_NCHW_32N2W8C_2PEN,  ///< x3 FC convolutions's weight (when H or W is not 1) for 2 PE
  LAYOUT_NCHW_32N2W8C_4PEN,  ///< x3 FC convolutions's weight (when H or W is not 1) for 4 PE
  LAYOUT_NCHW_32N2W8C_8PEN,  ///< x3 FC convolutions's weight (when H or W is not 1) for 8 PE

  LAYOUT_NCHW_8N,  ///< x3 bias, duplicate in all PEs

  LAYOUT_NCWH_ZHW_8N,       ///< x3 weight mask for 1 PE
  LAYOUT_NCWH_ZHW_4N_2PEN,  ///< x3 weight mask for 2 PE
  LAYOUT_NCWH_ZHW_2N_4PEN,  ///< x3 weight mask for 4 PE
  LAYOUT_NCWH_ZHW_2C_8PEN,  ///< x3 weight mask for 8 PE, different formula from others

  /**
   * other layout types
   */
  LAYOUT_NCHW_8W8C,  ///< apluscnn goldenc conv

  LAYOUT_NHCW_8C,       ///< x3 FC convolution's feature for 1 PE, 32bit
  LAYOUT_NHCW_8C_2PEC,  ///< x3 FC convolution's feature for 2 PE, 32bit
  LAYOUT_NHCW_8C_4PEC,  ///< x3 FC convolution's feature for 4 PE, 32bit
  LAYOUT_NHCW_8C_8PEC,  ///< x3 FC convolution's feature for 8 PE, 32bit

  LAYOUT_NHCW_16C,       ///< x3 FC convolution's feature for 1 PE, 16bit
  LAYOUT_NHCW_16C_2PEC,  ///< x3 FC convolution's feature for 2 PE, 16bit
  LAYOUT_NHCW_16C_4PEC,  ///< x3 FC convolution's feature for 4 PE, 16bit
  LAYOUT_NHCW_16C_8PEC,  ///< x3 FC convolution's feature for 8 PE, 16bit

  LAYOUT_NHCW_4H4W2C,       ///< x3 feature 1PE, 32bit
  LAYOUT_NHCW_4H4W2C_2PEN,  ///< x3 feature 2PE, each PE process its own feature, 32bit
  LAYOUT_NHCW_4H4W2C_4PEN,  ///< x3 feature 4PE, each PE process its own feature, 32bit
  LAYOUT_NHCW_4H4W2C_8PEN,  ///< x3 feature 8PE, each PE process its own feature, 32bit
  LAYOUT_NHCW_4H4W2C_2PEW,  ///< x3 feature 2PE, for channel-dependent operations (e.g., convolution), 32bit
  LAYOUT_NHCW_4H4W2C_4PEW,  ///< x3 feature 4PE, for channel-dependent operations (e.g., convolution), 32bit
  LAYOUT_NHCW_4H4W2C_8PEW,  ///< x3 feature 8PE, for channel-dependent operations (e.g., convolution), 32bit

  LAYOUT_NHCW_4H4W4C,       ///< x3 feature 1PE, 16bit
  LAYOUT_NHCW_4H4W4C_2PEN,  ///< x3 feature 2PE, each PE process its own feature, 16bit
  LAYOUT_NHCW_4H4W4C_4PEN,  ///< x3 feature 4PE, each PE process its own feature, 16bit
  LAYOUT_NHCW_4H4W4C_8PEN,  ///< x3 feature 8PE, each PE process its own feature, 16bit
  LAYOUT_NHCW_4H4W4C_2PEW,  ///< x3 feature 2PE, for channel-dependent operations (e.g., convolution), 16bit
  LAYOUT_NHCW_4H4W4C_4PEW,  ///< x3 feature 4PE, for channel-dependent operations (e.g., convolution), 16bit
  LAYOUT_NHCW_4H4W4C_8PEW,  ///< x3 feature 8PE, for channel-dependent operations (e.g., convolution), 16bit

  LAYOUT_NHCW_4H4W4C_2PEC,  ///< x3 feature 2PE, for channel-independent operations (e.g., pooling, roi resize), 16bit
  LAYOUT_NHCW_4H4W4C_4PEC,  ///< x3 feature 4PE, for channel-independent operations (e.g., pooling, roi resize), 16bit
  LAYOUT_NHCW_4H4W4C_8PEC,  ///< x3 feature 8PE, for channel-independent operations (e.g., pooling, roi resize), 16bit

  LAYOUT_NHWC_32W2C,  ///< x2 ParsingPostProcess(channelmax) output

  LAYOUT_NHCW_4C_8PEC,  ///< x3 elementwise 16bit literal input, 8PE
  LAYOUT_NHCW_4C_4PEC,  ///< x3 elementwise 16bit literal input, 4PE
  LAYOUT_NHCW_4C_2PEC,  ///< x3 elementwise 16bit literal input, 2PE

  LAYOUT_NHWC_4W8C,  ///< x2/x3 detectionpostprocess output 1PE
  LAYOUT_NHWC_4W16C,

  LAYOUT_NHWC_32C,  ///< x2/x3 detectionpostprocess intermediate result/encoded table.

  LAYOUT_NHWC_8C_4PEW,    ///< x3 detectionpostprocess output 4PE
  LAYOUT_NHWC_2W8C_2PEW,  ///< x3 detectionpostprocess output 2PE

  LAYOUT_NHWC_4C,  ///< x3 anchor table

  LAYOUT_NHCW_4C,       ///< x3 elementwise shift, 1PE
  LAYOUT_NHCW_2C,       ///< x3 gap partial sum, 1PE
  LAYOUT_NHCW_2C_2PEC,  ///< x3 elementwise shift or gap partial sum, 2PE
  LAYOUT_NHCW_2C_4PEC,  ///< x3 elementwise shift or gap partial sum, 4PE
  LAYOUT_NHCW_2C_8PEC,  ///< x3 elementwise shift or gap partial sum, 8PE

  LAYOUT_NHWC_8C_4PEN,    ///< x3 dpp output, separate feature, 4PE
  LAYOUT_NHWC_2W8C_2PEN,  ///< x3 dpp output, separate feature, 2PE

  LAYOUT_NHCW_2C_2PEN,  ///< x3 partial sum for int8_t output, 2 PE separate
  LAYOUT_NHCW_2C_4PEN,  ///< x3 partial sum for int8_t output, 4 PE separate
  LAYOUT_NHCW_2C_8PEN,  ///< x3 partial sum for int8_t output, 8 PE separate

  LAYOUT_NHCW_4C_2PEN,  ///< x3 partial sum for int16_t output, 2 PE separate
  LAYOUT_NHCW_4C_4PEN,  ///< x3 partial sum for int16_t output, 4 PE separate
  LAYOUT_NHCW_4C_8PEN,  ///< x3 partial sum for int16_t output, 8 PE separate

  LAYOUT_NHCW_8C_2PEN,  ///< x3 elementwise shift, 2PE, separate feature
  LAYOUT_NHCW_8C_4PEN,  ///< x3 elementwise shift, 4PE, separate feature
  LAYOUT_NHCW_8C_8PEN,  ///< x3 elementwise shift, 8PE, separate feature

  LAYOUT_NHWC_4W8C_2PEN,  ///< x3 dpp intermediate, 4 features per inst
  LAYOUT_NHWC_4W8C_4PEN,  ///< x3 dpp intermediate, 4 features per inst

  LAYOUT_NHWC_4W8C_2PEW,  ///< x3 dpp output, 4 features per inst
  LAYOUT_NHWC_4W8C_4PEW,  ///< x3 dpp output, 4 features per inst

  LAYOUT_NHWC_4096W8C,
  LAYOUT_NHWC_2048W8C_2PEW,
  LAYOUT_NHWC_1024W8C_4PEW,
  LAYOUT_NHWC_4096W8C_2PEN,
  LAYOUT_NHWC_4096W8C_4PEN,

  LAYOUT_NHCW_8C_2PEW,  ///< x3 filter output, 2PE
  LAYOUT_NHCW_8C_4PEW,  ///< x3 filter output, 4PE
  LAYOUT_NHCW_8C_8PEW,  ///< x3 filter output, 8PE

  LAYOUT_NCHW_32N,       ///< x3 FC bias, 1PE
  LAYOUT_NCHW_32N_2PEN,  ///< x3 FC bias, 2PE
  LAYOUT_NCHW_32N_4PEN,  ///< x3 FC bias, 4PE
  LAYOUT_NCHW_32N_8PEN,  ///< x3 FC bias, 8PE

  LAYOUT_NUM,

} hbrt_layout_type_t;

/**
 * Get layout name from enum
 * @param name name of the layout
 * @param layout layout enum
 * @return ::hbrtSuccess, ::hbrtErrorIllegalLayout
 */
HBDK_PUBLIC extern hbrt_error_t hbrtGetLayoutName(const char **name, hbrt_layout_type_t layout);

/**
 * Convert data layout
 * @param to_data the converted data will be written to this address
 * @param to_layout_type target layout type
 * @param from_data the address of source data
 * @param from_layout_type source layout type
 * @param element_type element type of the data
 * @param aligned_dim the dimensions of source data. should be 4-element uint32 array.
 * @param convert_endianness if true, the endianness of the data will also be converted.
 * @return ::hbrtSuccess, ::hbrtErrorIllegalLayout, ::hbrtErrorIllegalElementType
 * @note aligned_dim should take padding into account.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtConvertLayout(void *to_data, hbrt_layout_type_t to_layout_type,
                                                  const void *from_data, hbrt_layout_type_t from_layout_type,
                                                  hbrt_element_type_t element_type, hbrt_dimension_t aligned_dim,
                                                  bool convert_endianness);

/**
 * Similar to hbrtConvertLayout, but only data in the ROI will be converted
 * @param to_data the converted data will be written to this address
 * @param to_layout_type target layout type
 * @param from_data the address of source data
 * @param from_layout_type source layout type
 * @param element_type element type of the data
 * @param aligned_dim the dimensions of source data. should be 4-element uint32 array.
 * @param convert_endianness if true, the endianness of the data will also be converted.
 * @param roi_coord the coordinates of the start point of roi. inclusive
 * @param roi_size the size of the roi. exclusive
 * @return ::hbrtSuccess, ::hbrtErrorIllegalLayout, ::hbrtErrorIllegalElementType
 * @note aligned_dim should take padding into account
 */
HBDK_PUBLIC extern hbrt_error_t hbrtConvertLayoutRoi(void *to_data, hbrt_layout_type_t to_layout_type,
                                                     const void *from_data, hbrt_layout_type_t from_layout_type,
                                                     hbrt_element_type_t element_type, hbrt_dimension_t aligned_dim,
                                                     bool convert_endianness, hbrt_roi_t roi);

/**
 * Similar to hbrtConvertLayout, but only data in the ROI ({n_index, 0, 0, c_index}, {1, H, W, 1}) will be converted
 * @param to_data the converted data will be written to this address
 * @param to_layout_type target layout type
 * @param from_data the address of source data
 * @param from_layout_type source layout type
 * @param element_type element type of the data
 * @param aligned_dim the dimensions of source data. should be 4-element uint32 array.
 * @param convert_endianness if true, the endianness of the data will also be converted.
 * @param n_index index of N of data to convert
 * @param c_index index of C of data to convert
 * @return ::hbrtSuccess, ::hbrtErrorIllegalLayout, ::hbrtErrorIllegalElementType
 * @note aligned_dim should take padding into account
 */
HBDK_PUBLIC extern hbrt_error_t hbrtConvertLayoutToNative1HW1(void *to_data, const void *from_data,
                                                              hbrt_layout_type_t from_layout_type,
                                                              hbrt_element_type_t element_type,
                                                              hbrt_dimension_t aligned_dim, bool convert_endianness,
                                                              uint32_t n_index, uint32_t c_index);

/**
 * Similar to hbrtConvertLayout, but only data in the ROI ({n_index, h_index, w_index, 0}, {1, 1, 1, C}) will be
 * converted
 * @param to_data the converted data will be written to this address
 * @param to_layout_type target layout type
 * @param from_data the address of source data
 * @param from_layout_type source layout type
 * @param element_type element type of the data
 * @param aligned_dim the dimensions of source data. should be 4-element uint32 array.
 * @param convert_endianness if true, the endianness of the data will also be converted.
 * @param n_index index of N of data to convert
 * @param h_index index of H of data to convert
 * @param w_index index of W of data to convert
 * @return ::hbrtSuccess, ::hbrtErrorIllegalLayout, ::hbrtErrorIllegalElementType
 * @note aligned_dim should take padding into account
 */
HBDK_PUBLIC extern hbrt_error_t hbrtConvertLayoutToNative111C(void *to_data, const void *from_data,
                                                              hbrt_layout_type_t from_layout_type,
                                                              hbrt_element_type_t element_type,
                                                              hbrt_dimension_t aligned_dim, bool convert_endianness,
                                                              uint32_t n_index, uint32_t h_index, uint32_t w_index);

/**
 * Similar to hbrtConvertLayout, but only one point will be converted
 * @param to_data the converted data will be written to this address
 * @param to_layout_type target layout type
 * @param from_data the address of source data
 * @param from_layout_type source layout type
 * @param element_type element type of the data
 * @param aligned_dim the dimensions of source data. should be 4-element uint32 array.
 * @param convert_endianness if true, the endianness of the data will also be converted.
 * @param n_index index of N of data to convert
 * @param h_index index of H of data to convert
 * @param w_index index of W of data to convert
 * @param c_index index of C of data to convert
 * @return ::hbrtSuccess, ::hbrtErrorIllegalLayout, ::hbrtErrorIllegalElementType
 * @note aligned_dim should take padding into account
 */
HBDK_PUBLIC extern hbrt_error_t hbrtConvertLayoutToNative1111(void *to_data, const void *from_data,
                                                              hbrt_layout_type_t from_layout_type,
                                                              hbrt_element_type_t element_type,
                                                              hbrt_dimension_t aligned_dim, bool convert_endianness,
                                                              hbrt_dimension_t coord);

/**
 * Add padding to data
 * @param data_with_padding data with padding will be written to this address
 * @param dim_with_padding dimensions of data with padding.  should be 4-element uint32 array.
 * @param data_wo_padding source data without padding
 * @param dim_wo_padding dimensions of data without padding.  should be 4-element uint32 array.
 * @param element_type data element type
 * @return ::hbrtSuccess, ::hbrtErrorIllegalLayout, ::hbrtErrorIllegalElementType, ::hbrtErrorMemoryOverflow
 */
HBDK_PUBLIC extern hbrt_error_t hbrtAddPadding(void *data_with_padding, hbrt_dimension_t dim_with_padding,
                                               const void *data_wo_padding, hbrt_dimension_t dim_wo_padding,
                                               hbrt_element_type_t element_type);

/**
 * Remove padding from data
 * @param data_wo_padding data without padding will be written to this address
 * @param dim_wo_padding dimensions of data without padding.  should be 4-element uint32 array.
 * @param data_with_padding source data with padding
 * @param dim_with_padding dimensions of data with padding.  should be 4-element uint32 array.
 * @param element_type data element type
 * @return ::hbrtSuccess, ::hbrtErrorIllegalLayout, ::hbrtErrorIllegalElementType, ::hbrtErrorMemoryOverflow
 */
HBDK_PUBLIC extern hbrt_error_t hbrtRemovePadding(void *data_wo_padding, hbrt_dimension_t dim_wo_padding,
                                                  const void *data_with_padding, hbrt_dimension_t dim_with_padding,
                                                  hbrt_element_type_t element_type);

/**
 * Convert the endianss in [input, input+size) and store in output.
 * @param output the result will be written to this address
 * @param input source data address
 * @param size byte size of source data
 * @return ::hbrtSuccess, ::hbrtErrorMemoryOverflow
 * @note Input and output cannot have overlap, unless they are the same address.
 */
HBDK_PUBLIC extern hbrt_error_t hbrtConvertEndianness(void *output, const void *input, size_t size);

#ifdef __cplusplus
}  // extern "C"
#endif
