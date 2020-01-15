/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: hb_vio_fb_wrapper.h
 * @Brief: declaration of the hb_vio_fb_wrapper
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-05-23 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-05-23 16:16:58
 */

#ifndef EXAMPLE_INCLUDE_UTIL_HB_VIO_WRAPPER_H_
#define EXAMPLE_INCLUDE_UTIL_HB_VIO_WRAPPER_H_

#include <string>
#include "CNNMethod/util/util.h"
#include "hb_vio_common.h"
#include "hb_vio_interface.h"
#include "hobotxroc/image_tools.h"
#include "opencv2/opencv.hpp"
#include "util.h"
#include "x2_camera.h"

class HbVioWrapper {
 public:
  explicit HbVioWrapper(std::string hb_vio_cfg) : hb_vio_cfg_(hb_vio_cfg) {}

  virtual int Init() = 0;
  virtual int DeInit() = 0;

 protected:
  std::string hb_vio_cfg_;
  bool init_ = false;
};

class HbVioFbWrapper : public HbVioWrapper {
 public:
  explicit HbVioFbWrapper(std::string hb_vio_cfg) : HbVioWrapper(hb_vio_cfg) {}
  virtual ~HbVioFbWrapper() {
    if (init_) DeInit();
  }
  virtual int Init() {
    if (hb_vio_init(hb_vio_cfg_.c_str()) < 0) return -1;
    if (hb_vio_start() < 0) return -1;
    init_ = true;
  }

  virtual int DeInit() {
    if (hb_vio_stop() < 0) return -1;
    if (hb_vio_deinit() < 0) return -1;
    init_ = false;
  }

  void FreeImgInfo(img_info_t *fb_img) { hb_vio_free(fb_img); }

  int GetImgInfo(std::string rgb_file,
                 img_info_t *fb_img,
                 uint32_t *effective_w,
                 uint32_t *effective_h) {
    if (hb_vio_get_info(HB_VIO_FEEDBACK_SRC_INFO, &process_info_) < 0) {
      std::cout << "get fb src fail!!!" << std::endl;
      return -1;
    }
    cv::Mat img = cv::imread(rgb_file);
    cv::Mat img_1080p;
    int width = 1920;
    int height = 1080;
    TransImage(&img, &img_1080p, width, height, effective_w, effective_h);

    uint8_t *output_data = nullptr;
    int output_size, output_1_stride, output_2_stride;
    HobotXRocConvertImage(img_1080p.data,
                          height * width * 3,
                          width,
                          height,
                          width * 3,
                          0,
                          IMAGE_TOOLS_RAW_BGR,
                          IMAGE_TOOLS_RAW_YUV_NV12,
                          &output_data,
                          &output_size,
                          &output_1_stride,
                          &output_2_stride);

    memcpy(reinterpret_cast<uint8_t *>(process_info_.src_img.y_vaddr),
           output_data,
           output_size);
    HobotXRocFreeImage(output_data);
    hb_vio_set_info(HB_VIO_FEEDBACK_FLUSH, &process_info_);
    if (hb_vio_pym_process(&process_info_) < 0) {
      std::cout << "fb process fail!!!" << std::endl;
      return -1;
    }
    if (hb_vio_get_info(HB_VIO_PYM_INFO, fb_img) < 0) {
      std::cout << "hb_vio_get_info fail!!!" << std::endl;
      return -1;
    }
    return 0;
  }

  int GetImgInfo(uint8_t *nv12, int w, int h, img_info_t *fb_img) {
    if (hb_vio_get_info(HB_VIO_FEEDBACK_SRC_INFO, &process_info_) < 0) {
      std::cout << "get fb src fail!!!" << std::endl;
      return -1;
    }
    memcpy(reinterpret_cast<uint8_t *>(process_info_.src_img.y_vaddr),
           nv12,
           w * h * 3 / 2);
    hb_vio_set_info(HB_VIO_FEEDBACK_FLUSH, &process_info_);
    if (hb_vio_pym_process(&process_info_) < 0) {
      std::cout << "fb process fail!!!" << std::endl;
      return -1;
    }
    if (hb_vio_get_info(HB_VIO_PYM_INFO, fb_img) < 0) {
      std::cout << "hb_vio_get_info fail!!!" << std::endl;
      return -1;
    }
    return 0;
  }

 private:
  src_img_info_t process_info_;
};

class HbVioMonoCamera : public HbVioWrapper {
 public:
  explicit HbVioMonoCamera(std::string hb_vio_cfg, std::string camera_cfg)
      : HbVioWrapper(hb_vio_cfg), camera_cfg_(camera_cfg) {}
  virtual ~HbVioMonoCamera() {
    if (init_) DeInit();
  }
  virtual int Init() {
    if (hb_cam_init(camera_idx_, camera_cfg_.c_str()) < 0) return -1;
    if (hb_vio_init(hb_vio_cfg_.c_str()) < 0) return -1;
    if (hb_cam_start(camera_idx_) < 0) return -1;
    if (hb_vio_start() < 0) return -1;
    init_ = true;
  }
  virtual int DeInit() {
    if (hb_vio_stop() < 0) return -1;
    if (hb_cam_stop(camera_idx_) < 0) return -1;
    if (hb_vio_deinit() < 0) return -1;
    if (hb_cam_deinit(camera_idx_) < 0) return -1;
    init_ = false;
  }

  int GetImgInfo(img_info_t *pyd_img) {
    if (hb_vio_get_info(HB_VIO_PYM_INFO, pyd_img) < 0) {
      std::cout << "hb_vio_get_info fail!!!" << std::endl;
      return -1;
    }
    return 0;
  }

  void FreeImgInfo(img_info_t *pyd_img) { hb_vio_free(pyd_img); }

 private:
  std::string camera_cfg_;
  int camera_idx_ = 0;
};

class HbVioDualCamera : public HbVioWrapper {
 public:
  explicit HbVioDualCamera(std::string hb_vio_cfg, std::string camera_cfg)
      : HbVioWrapper(hb_vio_cfg), camera_cfg_(camera_cfg) {}
  virtual ~HbVioDualCamera() {
    if (init_) DeInit();
  }
  virtual int Init() {
    if (hb_cam_init(camera_idx_, camera_cfg_.c_str()) < 0) return -1;
    if (hb_vio_init(hb_vio_cfg_.c_str()) < 0) return -1;
    if (hb_cam_start(camera_idx_) < 0) return -1;
    if (hb_vio_start() < 0) return -1;
    init_ = true;
  }
  virtual int DeInit() {
    if (hb_vio_stop() < 0) return -1;
    if (hb_vio_deinit() < 0) return -1;
    if (hb_cam_stop(camera_idx_) < 0) return -1;
    if (hb_cam_deinit(camera_idx_) < 0) return -1;
    init_ = false;
  }

  int GetImgInfo(mult_img_info_t *pyd_img) {
    if (hb_vio_get_info(HB_VIO_PYM_MULT_INFO, pyd_img) < 0) {
      std::cout << "hb_vio_get_info fail!!!" << std::endl;
      return -1;
    }
    return 0;
  }

  void FreeImgInfo(mult_img_info_t *pyd_img) { hb_vio_mult_free(pyd_img); }

 private:
  std::string camera_cfg_;
  int camera_idx_ = 2;
};

#endif  // EXAMPLE_INCLUDE_UTIL_HB_VIO_WRAPPER_H_
