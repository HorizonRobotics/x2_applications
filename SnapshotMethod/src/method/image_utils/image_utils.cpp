/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     image_utils implementation
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.10
 * @date      2019.04.22
 */

#include <memory>
#include <chrono>

#include "SnapShotMethod/image_utils/image_utils.hpp"
#include "SnapShotMethod/error_code.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxsdk/xroc_data.h"

namespace HobotXRoc {

using std::chrono::high_resolution_clock;
using std::chrono::duration;
typedef std::shared_ptr<hobot::vision::CVImageFrame> CVImageFramePtr;

int ImageUtils::Data2CVImage(const uint32_t &height,
                             const uint32_t &width,
                             const HorizonVisionPixelFormat &format,
                             uint8_t *data,
                             cv::Mat &cv_img) {
  if (!data) {
    return XROC_SNAPSHOT_ERR_PARAM;
  }
  switch (format) {
    case kHorizonVisionPixelFormatNone:
    case kHorizonVisionPixelFormatImageContainer:
    case kHorizonVisionPixelFormatRawRGB565: {
      return XROC_SNAPSHOT_ERR_PARAM;
    }
    case kHorizonVisionPixelFormatRawRGB:
    case kHorizonVisionPixelFormatRawBGR: {
      cv_img = cv::Mat(height, width, CV_8UC3);
      memcpy(cv_img.data, data, height * width * 3);
      break;
    }
    case kHorizonVisionPixelFormatRawGRAY: {
      cv_img = cv::Mat(height, width, CV_8UC1);
      memcpy(cv_img.data, data, height * width);
      break;
    }
    case kHorizonVisionPixelFormatX2SRC:
    case kHorizonVisionPixelFormatX2PYM:
    case kHorizonVisionPixelFormatRawNV21:
    case kHorizonVisionPixelFormatRawNV12:
    case kHorizonVisionPixelFormatRawI420: {
      cv_img = cv::Mat(height * 3 / 2, width, CV_8UC1);
      memcpy(cv_img.data, data, height * width * 3 / 2);
      break;
    }
    default:
      return XROC_SNAPSHOT_ERR_PARAM;
  }
  return XROC_SNAPSHOT_OK;
}

#ifndef ALIGN
#define ALIGN(x, a) (((x) + (a) - 1) & ~(a - 1))
#endif
BBox ImageUtils::AdjustSnapRect(const uint32_t &frame_width,
                                const uint32_t &frame_height,
                                const BBox &in_bbox,
                                const float &scale) {
  auto s32LeftTopX = static_cast<int>(in_bbox.x1);
  auto s32LeftTopY = static_cast<int>(in_bbox.y1);
  auto s32Width = static_cast<int>(in_bbox.Width());
  auto s32Height = static_cast<int>(in_bbox.Height());

  auto s32CenterX = s32LeftTopX + (s32Width >> 1);
  auto s32CenterY = s32LeftTopY + (s32Height >> 1);
  s32CenterX = ALIGN(s32CenterX, 2);
  s32CenterY = ALIGN(s32CenterY, 2);

  s32Width = static_cast<int>(s32Width * scale);
  s32Height = static_cast<int>(s32Height * scale);

  s32Width = ALIGN(s32Width, 4);
  s32Height = ALIGN(s32Height, 4);

  if (s32Width >= frame_width && s32Height >= frame_height) {
    s32Width = s32Height = std::min(frame_width, frame_height) - 8;
  }
  if (s32Width > frame_width) {
    s32Width = frame_width - 8;
  }
  if (s32Height > frame_height) {
    s32Height = frame_height - 8;
  }
  if (s32Width < s32Height) {
    s32Width = s32Height;
  }
  if (s32Height < s32Width) {
    s32Height = s32Width;
  }

  s32LeftTopX = s32CenterX - (s32Width >> 1);
  s32LeftTopY = s32CenterY - (s32Height >> 1);
  auto s32RightBotX = s32CenterX + (s32Width >> 1);
  auto s32RightBotY = s32CenterY + (s32Height >> 1);

  if (s32LeftTopX < 0 || s32LeftTopY < 0 || s32RightBotX >= frame_width ||
      s32RightBotY >= frame_height) {
    if (s32LeftTopX < 0) {
      s32LeftTopX = 0;
    }
    if (s32LeftTopY < 0) {
      s32LeftTopY = 0;
    }
    if (s32RightBotX >= frame_width) {
      s32LeftTopX = s32LeftTopX - (s32RightBotX - frame_width);
    }
    if (s32RightBotY >= frame_height) {
      s32LeftTopY = s32LeftTopY - (s32RightBotY - frame_height);
    }
  }

  BBox out_bbox;
  out_bbox.x1 = s32LeftTopX;
  out_bbox.y1 = s32LeftTopY;
  out_bbox.x2 = s32LeftTopX + s32Width - 1;
  out_bbox.y2 = s32LeftTopY + s32Height - 1;
  return out_bbox;
}

ImageFramePtr ImageUtils::DoFaceCrop(const ImageFramePtr &frame,
                                     const BBox &crop_rect,
                                     const uint32_t &output_width,
                                     const uint32_t &output_height,
                                     const bool &need_resize) {
  if (!frame->Data())
    return nullptr;

  auto u32Width = static_cast<unsigned>(crop_rect.Width() + 1);
  auto u32Height = static_cast<unsigned>(crop_rect.Height() + 1);

  assert(u32Width == u32Height);
  bool bNeedScale = (u32Width != output_width)
      || (u32Height != output_height);
  int width = 0;
  int height = 0;
  int first_stride = 0;
  int second_stride = 0;
  int crop_data_size = 0;

  unsigned char *pCropBuf = nullptr;
  auto *output_format = new HobotXRocImageToolsPixelFormat();

  int s32Ret = HobotXRocCropImageFrameWithPaddingBlack(\
                       frame.get(),
                       static_cast<const int>(crop_rect.x1),
                       static_cast<const int>(crop_rect.y1),
                       static_cast<const int>(crop_rect.x2),
                       static_cast<const int>(crop_rect.y2),
                       output_format,
                       &pCropBuf, &crop_data_size,
                       &width, &height,
                       &first_stride, &second_stride);

  if (s32Ret < 0) {
    LOGE << "crop failed!\n";
    if (bNeedScale) {
      std::free(pCropBuf);
    }
    return nullptr;
  }
  int out_data_size, out_stride, out_stride_uv;
  CVImageFramePtr snap_frame(new hobot::vision::CVImageFrame());

  if (bNeedScale && need_resize) {
    HobotXRocImageToolsResizeInfo resize_info {};
    uint8_t *scale_data;

    auto resize_start_time = high_resolution_clock::now();
    s32Ret = HobotXRocResizeImage(pCropBuf, crop_data_size,
                                  width, height,
                                  first_stride, second_stride,
                                  *output_format,
                                  1,
                                  output_width,
                                  output_height,
                                  &scale_data, &out_data_size,
                                  &out_stride,
                                  &out_stride_uv,
                                  &resize_info);

    int cv_ret = ImageUtils::Data2CVImage(output_height,
                                          output_width,
                                          frame->pixel_format,
                                          scale_data,
                                          snap_frame->img);

    if (cv_ret != XROC_SNAPSHOT_OK) {
      return nullptr;
    }

    if (s32Ret < 0) {
      LOGE << "hobot scale failed!\n";
      std::free(pCropBuf);
      return nullptr;
    }
    std::free(scale_data);
    std::free(pCropBuf);
  } else {
    int cv_ret = ImageUtils::Data2CVImage(height,
                                          width,
                                          frame->pixel_format,
                                          pCropBuf,
                                          snap_frame->img);
    if (cv_ret != XROC_SNAPSHOT_OK) {
      return nullptr;
    }
    std::free(pCropBuf);
  }

  delete output_format;

  if (snap_frame) {
    if (frame->pixel_format == kHorizonVisionPixelFormatX2PYM
        || frame->pixel_format == kHorizonVisionPixelFormatX2SRC) {
      snap_frame->pixel_format = kHorizonVisionPixelFormatRawNV12;
    } else {
      snap_frame->pixel_format = frame->pixel_format;
    }
    snap_frame->frame_id = frame->frame_id;
    snap_frame->time_stamp = frame->time_stamp;
  }

  return snap_frame;
}

} // namespace HobotXRoc
