/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     image_utils header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.16
 * @date      2019.04.22
 */

#ifndef SNAPSHOTMETHOD_IMAGE_UTILS_IMAGE_UTILS_HPP_
#define SNAPSHOTMETHOD_IMAGE_UTILS_IMAGE_UTILS_HPP_

#include "horizon/vision_type/vision_type.hpp"
#include "hobotxroc/image_tools.h"

namespace HobotXRoc {

using hobot::vision::Point;
using hobot::vision::Points;
using hobot::vision::BBox;
typedef std::shared_ptr<hobot::vision::ImageFrame> ImageFramePtr;

class ImageUtils {
 public:
  static BBox AdjustSnapRect(const uint32_t &frame_width,
                             const uint32_t &frame_height,
                             const BBox &in_bbox,
                             const float &scale);

  static int Data2CVImage(const uint32_t &height,
                          const uint32_t &width,
                          const HorizonVisionPixelFormat &format,
                          uint8_t *data,
                          cv::Mat &cv_img);

  static ImageFramePtr DoFaceCrop(const ImageFramePtr &frame,
                                  const BBox &crop_rect,
                                  const uint32_t &output_width,
                                  const uint32_t &output_height,
                                  const bool &need_resize = true);
};
} // namespace HobotXRoc

#endif // SNAPSHOTMETHOD_IMAGE_UTILS_IMAGE_UTILS_HPP_
