/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     provides xroc image Rotater
 * @author    zhengzheng.ge
 * @email     zhengzheng.ge@horizon.ai
 * @version   0.0.0.1
 * @date      2019.11.15
 */

#ifndef HOBOTXROC_IMAGETOOLS_ROTATER_H_
#define HOBOTXROC_IMAGETOOLS_ROTATER_H_

#include <set>
#include "hobotxroc/imagetools/common.h"
#include "hobotxroc/imagetools/base.h"

namespace HobotXRoc {

class ImageRotater : public ImageBase {
 public:
  bool Rotate(const ImageToolsFormatData &input,
              const int degree,
              ImageToolsFormatData &output);
 private:
  bool RotateI420();
  bool RotateNV();
  bool RotateGray();
  bool RotateRGB();

 private:
  int degree_ = 0;
  static const std::set<int> degree_set_;
};
}  // namespace HobotXRoc

#endif  // HOBOTXROC_IMAGETOOLS_ROTATER_H_
