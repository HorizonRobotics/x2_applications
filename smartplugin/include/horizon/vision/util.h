/**
* Copyright (c) 2019 Horizon Robotics. All rights reserved.
* @file util.h
* @brief image conversion to xroc struct declaration
* @author ruoting.ding
* @email ruoting.ding@horizon.ai
* @date 2019/4/29
*/

#ifndef INCLUDE_HORIZON_VISION_UTIL_H_
#define INCLUDE_HORIZON_VISION_UTIL_H_
#include <cstdint>
#include <memory>

#include "horizon/vision_type/vision_type.h"
#include "horizon/vision_type/vision_type.hpp"
#include "hobotxsdk/xroc_sdk.h"

namespace horizon {
namespace vision {
namespace util {

using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
using XRocImageFramePtr = HobotXRoc::XRocData<ImageFramePtr>;
using XRocBaseDataVectorPtr = std::shared_ptr<HobotXRoc::BaseDataVector>;

HorizonVisionImage *ImageConversion(uint8_t *data,
                                    int data_size,
                                    int width,
                                    int height,
                                    int stride,
                                    int stride_uv,
                                    HorizonVisionPixelFormat format);

HorizonVisionImage *ImageConversion(const ImageFramePtr &cpp_img);
XRocImageFramePtr *ImageConversion(const HorizonVisionImage &c_img);
XRocImageFramePtr *ImageFrameConversion(const HorizonVisionImageFrame *);

}  // namespace util
}  // namespace vision
}  // namespace horizon

#endif  //  INCLUDE_HORIZON_VISION_UTIL_H_
