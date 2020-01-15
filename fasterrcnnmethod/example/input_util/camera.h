//
// Created by yaoyao.sun on 2019-08-13.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef EXAMPLE_INPUT_UTIL_CAMERA_H_
#define EXAMPLE_INPUT_UTIL_CAMERA_H_

#include <string>

#include "./x2_camera.h"
#include "hobotlog/hobotlog.hpp"
#include "./vio.h"

class Camera {
 public:
  Camera(uint32_t camera_index, const std::string &cam_cfg_file,
         const std::string &vio_cfg_file) : camera_index_(-1), vio_() {
    camera_index_ = camera_index;
    auto ret = hb_cam_init(camera_index_, cam_cfg_file.c_str());
    HOBOT_CHECK(ret == 0) << "camera init failed!!!";
    ret = vio_.Init(vio_cfg_file.c_str());
    HOBOT_CHECK(ret == 0) << "vio init failed!!!";
    ret = hb_cam_start(0);
    HOBOT_CHECK(ret == 0) << "camera start failed!!!";
    ret = hb_vio_start();
    HOBOT_CHECK(ret == 0) << "vio start failed!!!";
  }
  int GetImage(img_info_t* data) {
    return vio_.GetImage(data);
  }
  int GetMultiImage(mult_img_info_t* data) {
    return vio_.GetMultiImage(data);
  }
  int Free(img_info_t* data) {
    return vio_.Free(data);
  }

  int Free(mult_img_info_t* data) {
    return vio_.Free(data);
  }
  ~Camera() {
    vio_.Stop();
    hb_cam_stop(camera_index_);
    hb_cam_deinit(camera_index_);
  }

  Camera() = delete;
  Camera(const Camera& ) = delete;
  Camera& operator=(const Camera&) = delete;

 private:
  uint32_t camera_index_;
  VIO vio_;
};

#endif  // EXAMPLE_INPUT_UTIL_CAMERA_H_
