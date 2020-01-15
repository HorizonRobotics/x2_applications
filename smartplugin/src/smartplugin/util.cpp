/**
* Copyright (c) 2019 Horizon Robotics. All rights reserved.
* @file util.h
* @brief convert common image to xroc struct.
* @author ruoting.ding
* @email ruoting.ding@horizon.ai
* @date 2019/4/29
*/

#include <cstdint>
#include <vector>
#include <memory>
#include <string>
#include "horizon/vision_type/vision_type.h"
#include "horizon/vision_type/vision_type.hpp"
#include "horizon/vision_type/vision_type_util.h"
#include "hobotlog/hobotlog.hpp"
#include "opencv2/opencv.hpp"
#include "horizon/vision/util.h"
#include "hobotxsdk/xroc_data.h"

namespace horizon {
namespace vision {
namespace util {
HorizonVisionImage *ImageConversion(uint8_t *data,
                                    int data_size,
                                    int width,
                                    int height,
                                    int stride,
                                    int stride_uv,
                                    HorizonVisionPixelFormat format) {
//  DumpImageFrame(cpp_img);
  HOBOT_CHECK(data) << "ImageConversion null input";
  HorizonVisionImage *c_img;
  HorizonVisionAllocImage(&c_img);
  c_img->pixel_format = format;

  c_img->data = static_cast<uint8_t *>(
      std::calloc(data_size, sizeof(uint8_t)));

  memcpy(c_img->data, data, data_size);
  c_img->data_size = data_size;
  c_img->width = width;
  c_img->height = height;
  c_img->stride = stride;
  c_img->stride_uv = stride_uv;
  return c_img;
}

HorizonVisionImage *ImageConversion(const ImageFramePtr &cpp_img) {
//  DumpImageFrame(cpp_img);
  HOBOT_CHECK(cpp_img) << "ImageConversion null input";
  HorizonVisionImage *c_img;
  HorizonVisionAllocImage(&c_img);
  c_img->pixel_format = cpp_img->pixel_format;

  c_img->data = static_cast<uint8_t *>(
      std::calloc(cpp_img->DataSize(), sizeof(uint8_t)));

  memcpy(c_img->data, cpp_img->Data(), cpp_img->DataSize());
  c_img->data_size = cpp_img->DataSize();
  c_img->width = cpp_img->Width();
  c_img->height = cpp_img->Height();
  c_img->stride = cpp_img->Stride();
  c_img->stride_uv = cpp_img->StrideUV();
  return c_img;
}
#ifdef X2
void check_pyramid(img_info_t *img_info) {
  for (auto i = 0; i < 5; ++i) {
    auto index = i * 4;
    HOBOT_CHECK(img_info->down_scale[index].height);
    HOBOT_CHECK(img_info->down_scale[index].step);
  }
}
#endif

namespace {
void GetImage(const HorizonVisionImage &image,
              const cv::Mat &bgr_img,
              int level = 0) {
  auto img_type = image.pixel_format;
  switch (img_type) {
    case kHorizonVisionPixelFormatX2PYM: {
#ifdef X2
      cv::Mat yuv420p;
      img_info_t *src_img = reinterpret_cast<img_info_t * >(image.data);
      int index = level * 4;
      auto height = src_img->down_scale[index].height;
      auto width = src_img->down_scale[index].width;
      auto y_addr = src_img->down_scale[index].y_vaddr;
      auto uv_addr = src_img->down_scale[index].c_vaddr;
      HOBOT_CHECK(height) << "width = " << width << ", height = " << height;
      auto img_y_size = src_img->down_scale[index].height *
          src_img->down_scale[index].step;
      auto img_uv_size = img_y_size / 2;

      yuv420p.create(height * 3 / 2, width, CV_8UC1);
      memcpy(yuv420p.data, y_addr, img_y_size);
      memcpy(yuv420p.data + height * width, uv_addr, img_uv_size);
      HOBOT_CHECK(!yuv420p.empty())
      << "width = " << width << ", height = " << height;
      cv::cvtColor(yuv420p, bgr_img, CV_YUV2BGR_NV12);

#endif
      return;
    }
    case kHorizonVisionPixelFormatRawNV12: {
      cv::Mat yuv420p(image.height * 3 / 2, image.width, CV_8UC1, image.data);
      cv::cvtColor(yuv420p, bgr_img, CV_YUV2BGR_NV12);
      return;
    }
    default: {
      return;
    }
  }
}
}  // end of namespace

XRocImageFramePtr *ImageConversion(const HorizonVisionImage &c_img) {
  auto xroc_img = new XRocImageFramePtr();
  xroc_img->type_ = "ImageFrame";
  auto image_type = c_img.pixel_format;
  switch (image_type) {
    case kHorizonVisionPixelFormatNone: {
      LOGI << "kHorizonVisionPixelFormatNone, data size is " << c_img.data_size;
      auto cv_img = std::make_shared<hobot::vision::CVImageFrame>();
      std::vector<unsigned char>
          buf(c_img.data, c_img.data + c_img.data_size);
      cv_img->img =
          cv::imdecode(buf, cv::IMREAD_COLOR);
      HOBOT_CHECK(!cv_img->img.empty())
      << "Invalid image , failed to create cvmat for"
         " kHorizonVisionPixelFormatNone type image";
      cv_img->pixel_format =
          HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR;
      xroc_img->value = cv_img;
      break;
    }
    case kHorizonVisionPixelFormatRawBGR: {
      auto cv_img = std::make_shared<hobot::vision::CVImageFrame>();
      cv_img->img =
          cv::Mat(c_img.height, c_img.width, CV_8UC3, c_img.data);
//      cv::imwrite("c_bgr_in.jpg", cv_img->img);
      HOBOT_CHECK(!cv_img->img.empty())
      << "Invalid image , failed to create cvmat for"
         " kHorizonVisionPixelFormatRawBGR type image";
      cv_img->pixel_format =
          HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR;
      xroc_img->value = cv_img;
      break;
    }
#ifdef X2
    case kHorizonVisionPixelFormatX2SRC: {
      LOGF << "No support for kHorizonVisionPixelFormatX2SRC";
      break;
    }
    case kHorizonVisionPixelFormatX2PYM: {
      if (getenv("dump_original_img")) {
        static int seq = 0;
        cv::Mat orig_mat;
        GetImage(c_img, orig_mat, 0);
        std::string img_path = "./data/input_" + std::to_string(seq++) + ".jpg";
        cv::imwrite(img_path, orig_mat);
      }
      auto pym_img = std::make_shared<hobot::vision::PymImageFrame>();
      auto img_info = reinterpret_cast<img_info_t *>(c_img.data);
      HOBOT_CHECK(img_info);
      pym_img->img = *img_info;
      xroc_img->value = pym_img;
      check_pyramid(img_info);
      break;
    }
#endif
    default: {
      LOGF << "No support image type " << image_type;
    }
  }
  return xroc_img;
}

XRocImageFramePtr *ImageFrameConversion(
    const HorizonVisionImageFrame *c_img_frame) {
  auto xroc_img_frame = ImageConversion(c_img_frame->image);
  xroc_img_frame->value->time_stamp = c_img_frame->time_stamp;
  xroc_img_frame->value->channel_id = c_img_frame->channel_id;
  xroc_img_frame->value->frame_id = c_img_frame->frame_id;
  return xroc_img_frame;
}

}  // namespace util
}  // namespace vision
}  // namespace horizon


