/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc image Decoder
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.10
 */
#ifndef INCLUDE_HOBOTXROC_IMAGETOOLS_DECODER_H_
#define INCLUDE_HOBOTXROC_IMAGETOOLS_DECODER_H_

#include <turbojpeg.h>
#include "hobotxroc/imagetools/common.h"
#include "hobotxroc/image_tools.h"

namespace HobotXRoc {

class ImageDecoder {
 public:
  bool Decode(const uint8_t *input,
              const int input_size,
              const HobotXRocImageToolsPixelFormat dst_fmt,
              ImageToolsFormatData &output);
 private:
  // decode to rgb/bgr/gray
  bool DecodeByTj(TJPF eformat);

 private:
  const uint8_t *input_data_ = nullptr;
  int input_data_size_ = 0;
  HobotXRocImageToolsPixelFormat dst_fmt_;
  int width_ = 0;
  int height_ = 0;
  uint8_t *output_data_ = nullptr;
  int output_data_size_ = 0;
  int first_stride_ = 0;
  int second_stride_ = 0;
};
}  // namespace HobotXRoc

#endif  // INCLUDE_HOBOTXROC_IMAGETOOLS_DECODER_H_
