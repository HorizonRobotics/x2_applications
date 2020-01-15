/*
 * @Description: implement of vioplugin
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-08-26 16:17:25
 * @Author: songshan.gong@horizon.ai
 * @Date: 2019-09-26 16:17:25
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-09-26 17:21:53
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#include <ctime>
#include <fstream>

#include <unistd.h>

#include "vioplugin/vioprocess.h"

#include "opencv2/opencv.hpp"

#include "hobotlog/hobotlog.hpp"

int HorizonConvertImage(HorizonVisionImage* in_img, HorizonVisionImage* dst_img,
                        HorizonVisionPixelFormat dst_format) {
  if (!in_img || !dst_img) return kHorizonVisionErrorParam;
  dst_img->width = in_img->width;
  dst_img->height = in_img->height;
  dst_img->stride = in_img->stride;
  dst_img->stride_uv = in_img->stride_uv;
  dst_img->pixel_format = dst_format;
  if (dst_format != in_img->pixel_format) {
    switch (in_img->pixel_format) {
      case kHorizonVisionPixelFormatRawRGB: {
        cv::Mat rgb888(in_img->height, in_img->width, CV_8UC3, in_img->data);
        switch (dst_format) {
          case kHorizonVisionPixelFormatRawRGB565: {
            auto dst_data_size = in_img->data_size / 3 * 2;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat rgb565(in_img->height, in_img->width, CV_8UC2,
                           dst_img->data);
            cv::cvtColor(rgb888, rgb565, CV_BGR2BGR565);
          } break;
          case kHorizonVisionPixelFormatRawBGR: {
            dst_img->data_size = in_img->data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_img->data_size));
            cv::Mat bgr888(in_img->height, in_img->width, CV_8UC3,
                           dst_img->data);
            cv::cvtColor(rgb888, bgr888, CV_RGB2BGR);
          } break;
          default:
            return kHorizonVisionErrorNoImpl;
        }
      } break;
      case kHorizonVisionPixelFormatRawRGB565: {
        cv::Mat rgb565(in_img->height, in_img->width, CV_8UC2, in_img->data);
        switch (dst_format) {
          case kHorizonVisionPixelFormatRawRGB: {
            auto dst_data_size = (in_img->data_size >> 1) * 3;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat rgb888(in_img->height, in_img->width, CV_8UC3,
                           dst_img->data);
            cv::cvtColor(rgb565, rgb888, CV_BGR5652BGR);
          } break;
          case kHorizonVisionPixelFormatRawBGR: {
            auto dst_data_size = (in_img->data_size >> 1) * 3;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat bgr888(in_img->height, in_img->width, CV_8UC3,
                           dst_img->data);
            cv::cvtColor(rgb565, bgr888, CV_BGR5652RGB);
          } break;

          default:
            return kHorizonVisionErrorNoImpl;
        }
      } break;
      case kHorizonVisionPixelFormatRawBGR: {
        cv::Mat bgr888(in_img->height, in_img->width, CV_8UC3, in_img->data);
        switch (dst_format) {
          case kHorizonVisionPixelFormatRawRGB: {
            auto dst_data_size = in_img->data_size;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat rgb888(in_img->height, in_img->width, CV_8UC3,
                           dst_img->data);
            cv::cvtColor(bgr888, rgb888, CV_BGR2RGB);
          } break;
          case kHorizonVisionPixelFormatRawRGB565: {
            auto dst_data_size = in_img->data_size / 3 * 2;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat rgb565(in_img->height, in_img->width, CV_8UC2,
                           dst_img->data);
            cv::cvtColor(bgr888, rgb565, CV_RGB2BGR565);
          } break;
          case kHorizonVisionPixelFormatRawNV12: {
            auto dst_data_size = (in_img->height * in_img->width) * 3 / 2;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat yuv420_mat;
            cv::cvtColor(bgr888, yuv420_mat, cv::COLOR_BGR2YUV_I420);
            // copy y data
            int y_size = in_img->height * in_img->width;
            auto* yuv420_ptr = yuv420_mat.ptr<uint8_t>();
            memcpy(dst_img->data, yuv420_ptr, y_size);
            // copy uv data
            int uv_stride = in_img->width * in_img->height / 4;
            uint8_t* uv_data = dst_img->data + y_size;
            for (int i = 0; i < uv_stride; ++i) {
              *(uv_data++) = *(yuv420_ptr + y_size + i);
              *(uv_data++) = *(yuv420_ptr + y_size + uv_stride + i);
            }
          } break;
          default:
            return kHorizonVisionErrorNoImpl;
        }
      } break;
      case kHorizonVisionPixelFormatRawBGRA: {
        cv::Mat bgra(in_img->height, in_img->width, CV_8UC4, in_img->data);
        switch (dst_format) {
          case kHorizonVisionPixelFormatRawRGB: {
            auto dst_data_size = (in_img->height * in_img->width) * 3;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat rgb888(in_img->height, in_img->width, CV_8UC3,
                           dst_img->data);
            cv::cvtColor(bgra, rgb888, CV_BGRA2RGB);
          } break;
          case kHorizonVisionPixelFormatRawBGR: {
            auto dst_data_size = (in_img->height * in_img->width) * 3;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat bgr888(in_img->height, in_img->width, CV_8UC3,
                           dst_img->data);
            cv::cvtColor(bgra, bgr888, CV_BGRA2BGR);
          } break;
          default:
            return kHorizonVisionErrorNoImpl;
        }
      } break;
      case kHorizonVisionPixelFormatRawNV12: {
        cv::Mat nv12((in_img->height * 3) >> 1, in_img->width, CV_8UC1,
                     in_img->data);
        switch (dst_format) {
          case kHorizonVisionPixelFormatRawBGR: {
            auto dst_data_size = (in_img->height * in_img->width) * 3;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat bgr(in_img->height, in_img->width, CV_8UC3, dst_img->data);
            cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);
          } break;
          case kHorizonVisionPixelFormatRawBGRA: {
            auto dst_data_size = (in_img->height * in_img->width) << 2;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat bgra(in_img->height, in_img->width, CV_8UC4, dst_img->data);
            cv::cvtColor(nv12, bgra, CV_YUV2BGRA_NV12);
          } break;
          case kHorizonVisionPixelFormatRawRGBA: {
            auto dst_data_size = (in_img->height * in_img->width) << 2;
            dst_img->data_size = dst_data_size;
            dst_img->data =
                reinterpret_cast<uint8_t*>(std::malloc(dst_data_size));
            cv::Mat rgba(in_img->height, in_img->width, CV_8UC4, dst_img->data);
            cv::cvtColor(nv12, rgba, CV_YUV2RGBA_NV12);
          } break;
          default:
            return kHorizonVisionErrorNoImpl;
        }
      } break;
      default:
        return kHorizonVisionErrorNoImpl;
    }
  } else {
    dst_img->data_size = in_img->data_size;
    dst_img->data = reinterpret_cast<uint8_t*>(std::malloc(in_img->data_size));
    std::memcpy(dst_img->data, in_img->data, in_img->data_size);
  }
  return kHorizonVisionSuccess;
}

int HorizonSave2File(HorizonVisionImage* img, const char* file_path) {
  if (!img || !file_path) return kHorizonVisionErrorParam;
  if (0 == strlen(file_path)) return kHorizonVisionErrorParam;
  if (kHorizonVisionPixelFormatRawBGR == img->pixel_format) {
    cv::Mat bgr_img_mat(img->height, img->width, CV_8UC3, img->data);
    cv::imwrite(file_path, bgr_img_mat);
    return kHorizonVisionSuccess;
  }
  HorizonVisionImage* bgr_img;
  HorizonVisionAllocImage(&bgr_img);
  int ret = HorizonConvertImage(img, bgr_img, kHorizonVisionPixelFormatRawBGR);
  if (ret != kHorizonVisionSuccess) return ret;
  // save to file
  cv::Mat bgr_img_mat(bgr_img->height, bgr_img->width, CV_8UC3, bgr_img->data);
  cv::imwrite(file_path, bgr_img_mat);
  HorizonVisionFreeImage(bgr_img);
  return kHorizonVisionSuccess;
}

int HorizonFillFromFile(const char* file_path, HorizonVisionImage** ppimg) {
  if (!ppimg) {
    return kHorizonVisionErrorParam;
  }
  if (*ppimg || !file_path) return kHorizonVisionErrorParam;

  if (!strcmp(file_path, "")) {
    return kHorizonVisionErrorParam;
  }
  if (access(file_path, F_OK) != 0) {
    LOGE << "file not exist: " << file_path;
    return kHorizonVisionOpenFileFail;
  }

  try {
    auto bgr_mat = cv::imread(file_path);
    if (!bgr_mat.data) {
      LOGF << "Failed to call imread for " << file_path;
      return kHorizonVisionFailure;
    }
    HorizonVisionAllocImage(ppimg);
    auto& img = *ppimg;
    img->pixel_format = kHorizonVisionPixelFormatRawBGR;
    img->data_size = static_cast<uint32_t>(bgr_mat.total() * 3);
    // HorizonVisionFreeImage call std::free to free data
    img->data = reinterpret_cast<uint8_t*>(
        std::calloc(img->data_size, sizeof(uint8_t)));
    std::memcpy(img->data, bgr_mat.data, img->data_size);
    img->width = static_cast<uint32_t>(bgr_mat.cols);
    img->height = static_cast<uint32_t>(bgr_mat.rows);
    img->stride = static_cast<uint32_t>(bgr_mat.cols);
    img->stride_uv = static_cast<uint32_t>(bgr_mat.cols);
  } catch (const cv::Exception& e) {
    LOGF << "Exception to call imread for " << file_path;
    return kHorizonVisionOpenFileFail;
  }
  return kHorizonVisionSuccess;
}
