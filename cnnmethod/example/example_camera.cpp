/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <vector>
#include "hb_vio_common.h"
#include "hb_vio_interface.h"
#include "hobotxsdk/xroc_sdk.h"
#include "opencv2/opencv.hpp"
#include "horizon/vision_type/vision_type.hpp"
#include "util/hb_vio_wrapper.h"
#include "x2_camera.h"

using HobotXRoc::BaseData;
using HobotXRoc::BaseDataPtr;
using HobotXRoc::BaseDataVector;
using HobotXRoc::InputData;
using HobotXRoc::InputDataPtr;
using HobotXRoc::XRocData;

using hobot::vision::ImageFrame;
using hobot::vision::PymImageFrame;

void DumpAntiSpfDual(std::vector<HobotXRoc::BaseDataPtr> *result,
                     std::ostream &os,
                     std::string id);
static void Usage() {
  std::cout << "./example det_from_camera xroc_config vio_config_file "
               "camera_config_file output"
            << std::endl;
}

int DetFromCamera(int argc, char **argv) {
  if (argc < 5) {
    Usage();
    return -1;
  }
  const char *xroc_config = argv[1];
  const char *vio_cfg_file = argv[2];
  const char *cam_cfg_file = argv[3];
  const char *rlt_put_file = argv[4];

  HbVioDualCamera hd_vio(vio_cfg_file, cam_cfg_file);
  if (hd_vio.Init() < 0) {
    std::cout << "vio init failed" << std::endl;
    return -1;
  }

  HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
  flow->SetConfig("config_file", xroc_config);
  flow->Init();

  mult_img_info_t data;
  int mult_ret = 0;

  int frame_cnt = 1000;
  std::ofstream output(rlt_put_file, std::ios::out);
  for (int i = 0; i < frame_cnt; i++) {
    mult_ret = hd_vio.GetImgInfo(&data);
    if (mult_ret < 0) {
      std::cout << "hb_vio_get_info 6 err" << std::endl;
      return -1;
    }
    // 构造mat
    auto img_bgr = data.img_info[0];
    auto img_gray = data.img_info[1];
#if 0
    int bgr_height = img_bgr.src_img.height;
    int bgr_witdh = img_bgr.src_img.width;
    int bgr_y_len = bgr_height * bgr_witdh;
    int bgr_uv_len = bgr_height * bgr_witdh / 2;
    uint8_t *bgr_ptr = static_cast<uint8_t *>(malloc(bgr_y_len + bgr_uv_len));
    memcpy(bgr_ptr, img_bgr.src_img.y_vaddr, bgr_y_len);
    memcpy(bgr_ptr + bgr_y_len, img_bgr.src_img.c_vaddr, bgr_uv_len);

    int gray_height = img_gray.src_img.height;
    int gray_witdh = img_gray.src_img.width;
    int gray_y_len = gray_height * gray_witdh;
    int gray_uv_len = gray_height * gray_witdh / 2;
    uint8_t *gray_ptr =
        static_cast<uint8_t *>(malloc(gray_y_len + gray_uv_len));
    memcpy(gray_ptr, img_gray.src_img.y_vaddr, gray_y_len);
    memcpy(gray_ptr + gray_y_len, img_gray.src_img.c_vaddr, gray_uv_len);

    HobotXRoc::DumpBinaryFile(
        bgr_ptr, bgr_y_len + bgr_uv_len, std::to_string(i) + "_bgr.nv12");
    HobotXRoc::DumpBinaryFile(
        gray_ptr, gray_y_len + gray_uv_len, std::to_string(i) + "_gray.nv12");
#endif
    HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
    auto py_bgr = std::make_shared<PymImageFrame>();
    auto py_gray = std::make_shared<PymImageFrame>();
    py_bgr->img = img_bgr;
    py_gray->img = img_gray;

    auto xroc_pyramid_bgr =
        std::make_shared<XRocData<std::shared_ptr<ImageFrame>>>();
    xroc_pyramid_bgr->value = py_bgr;
    xroc_pyramid_bgr->name_ = "bgr_pyramid";

    auto xroc_pyramid_gray =
        std::make_shared<XRocData<std::shared_ptr<ImageFrame>>>();
    xroc_pyramid_gray->value = py_gray;
    xroc_pyramid_gray->name_ = "gray_pyramid";

    inputdata->datas_.push_back(
        std::static_pointer_cast<HobotXRoc::BaseData>(xroc_pyramid_bgr));
    inputdata->datas_.push_back(
        std::static_pointer_cast<HobotXRoc::BaseData>(xroc_pyramid_gray));
    auto out = flow->SyncPredict(inputdata);
    DumpAntiSpfDual(&(out->datas_), output, std::to_string(i));

    hd_vio.FreeImgInfo(&data);
  }
  delete flow;

  return 0;
}

void DumpAntiSpfDual(std::vector<HobotXRoc::BaseDataPtr> *result,
                     std::ostream &os,
                     std::string id) {
  os << id;
  auto anti_bgr_spfs =
      std::static_pointer_cast<HobotXRoc::BaseDataVector>((*result)[0]);
  auto anti_gray_spfs =
      std::static_pointer_cast<HobotXRoc::BaseDataVector>((*result)[1]);

  int target_size = anti_bgr_spfs->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto anti_spf = std::static_pointer_cast<
        HobotXRoc::XRocData<hobot::vision::Attribute<int>>>(
        anti_bgr_spfs->datas_[target_idx]);

    if (anti_spf->state_ == HobotXRoc::DataState::VALID) {
      os << " " << anti_spf->value.score;
    }
  }
  os << "  :";
  target_size = anti_gray_spfs->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto anti_spf = std::static_pointer_cast<
        HobotXRoc::XRocData<hobot::vision::Attribute<int>>>(
        anti_gray_spfs->datas_[target_idx]);
    if (anti_spf->state_ == HobotXRoc::DataState::VALID) {
      os << " " << anti_spf->value.score;
    }
  }
  os << std::endl;
}
