/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file vision_type_common.h
 * \~English @brief this c header file defines the vision related data structure that are
 * used in IOT, including face, head
 * @date 2019/4/3
 */

#ifndef VISION_TYPE_VISION_TYPE_COMMON_H_
#define VISION_TYPE_VISION_TYPE_COMMON_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
/**
 * \~Chinese @brief 图片编码方式
 */
typedef enum HorizonVisionPixelFormat_ {
  /// \~Chinese 普通图片文件的二进制流，默认BGR, 需要与二进制流文件长度一起配套使用
      kHorizonVisionPixelFormatNone = 0,
  /// RGB
      kHorizonVisionPixelFormatRawRGB,
  /// RGB565
      kHorizonVisionPixelFormatRawRGB565,
  /// BGR
      kHorizonVisionPixelFormatRawBGR,
  /// \~Chinese 单通道灰度图
      kHorizonVisionPixelFormatRawGRAY,
  /// YUV420SP:NV21
      kHorizonVisionPixelFormatRawNV21,
  /// YUV420SP:NV12
      kHorizonVisionPixelFormatRawNV12,
  /// YUV420P:I420
      kHorizonVisionPixelFormatRawI420,
  /// X2:SRC
      kHorizonVisionPixelFormatX2SRC,
  /// X2:PYM
      kHorizonVisionPixelFormatX2PYM,
  /// \~Chinese 图片标准格式 ，比如jpeg
      kHorizonVisionPixelFormatImageContainer,
  /// RGBA
      kHorizonVisionPixelFormatRawRGBA,
  /// BGRA
      kHorizonVisionPixelFormatRawBGRA,
} HorizonVisionPixelFormat;

/**
 * \~Chinese @brief 朝向
 */
typedef enum HorizonVisionOrientation_ {
  /// \~Chinese 未知
      Unknown = 0,
  /// \~Chinese 前
      Front = 1,
  /// \~Chinese 后
      Back = 2,
  /// \~Chinese 左
      Left = 3,
  /// \~Chinese 右
      Right = 4
} HorizonVisionOrientation;

#ifdef __cplusplus
}
#endif
#endif  // VISION_TYPE_VISION_TYPE_COMMON_H_
