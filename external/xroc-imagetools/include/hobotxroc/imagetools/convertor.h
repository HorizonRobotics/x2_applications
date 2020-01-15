/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc image Convertor
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.10
 */

#ifndef INCLUDE_HOBOTXROC_IMAGETOOLS_CONVERTOR_H_
#define INCLUDE_HOBOTXROC_IMAGETOOLS_CONVERTOR_H_
#include "hobotxroc/imagetools/common.h"
#include "hobotxroc/imagetools/base.h"
namespace HobotXRoc {

class ImageConvertor : public ImageBase {
 public:
  bool Convert(const ImageToolsFormatData &input,
               const HobotXRocImageToolsPixelFormat dst_fmt,
               ImageToolsFormatData &output);

 private:
  // convert all(except gray) to rgb/bgr
  bool ConvertToRGBOrBGR();

  // convert all(except gray) to i420
  bool ConvertToI420();

  // convert RGB/BGR to gray
  bool ConvertRGBorBGRToGray();

  // convert I420/NV12/NV21 to gray
  bool ConvertYUV420ToGray();

  // convert I420 to NV12/NV21
  bool ConvertI420ToNV();

  // convert NV12 to NV21  or convert NV21 to NV12
  bool ConvertBetweenNV();

  // convert RGB/BGR to NV12/nv21
  bool ConvertRGBOrBGRToNV();

  // convert nv12 to yuv444
  bool ConvertNV12ToYUV444();

 private:
  int width_;
  int height_;
};

}  // namespace HobotXRoc

#endif  // INCLUDE_HOBOTXROC_IMAGETOOLS_CONVERTOR_H_
