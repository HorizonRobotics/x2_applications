/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: example_do_pose_lmk.cpp
 * @Brief:
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 15:18:10
 */

#include <stdint.h>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include "CNNMethod/CNNMethod.h"
#include "CNNMethod/util/util.h"
#include "FasterRCNNMethod/FasterRCNNMethod.h"
#include "bpu_predict/bpu_io.h"
#include "bpu_predict/bpu_predict.h"
#include "hobotxroc/method.h"
#include "hobotxroc/method_factory.h"
#include "hobotxsdk/xroc_sdk.h"
#include "horizon/vision_type/vision_type.hpp"
#include "horizon/vision_type/vision_type_common.h"
#include "opencv2/opencv.hpp"
#include "util/hb_vio_wrapper.h"

typedef std::shared_ptr<hobot::vision::ImageFrame> ImageFramePtr;
using HobotXRoc::BaseDataPtr;

struct PyramidResult {
  img_info_t result_info;
};

static void Usage() {
  std::cout
      << "./example do_fb_rect_cnn [pose_lmk|age_gender|anti_spf|face_quality] "
         "xroc_cfg_file fb_cfg img_list out_file"
      << std::endl;
}

void DumpPoseLmk(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpAgeGender(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpAntiSpf(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpFaceQuality(std::vector<BaseDataPtr> &, std::ostream &, std::string);

int DoFbRectCnn(int argc, char **argv) {
  if (argc < 6) {
    Usage();
    return 1;
  }
  std::string model_name(argv[1]);
  std::string cfg_file(argv[2]);
  std::string fb_cfg(argv[3]);
  std::string img_list(argv[4]);
  std::string rlt_put_file = argv[5];

  HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
  flow->SetConfig("config_file", cfg_file.c_str());
  flow->SetConfig("profiler", "on");
  flow->SetConfig("profiler_file", "./profiler.txt");
  flow->Init();

  img_info_t feed_back_info;
  HbVioFbWrapper fb_handle(fb_cfg);
  fb_handle.Init();

  std::ifstream img_list_file(img_list);
  std::string img_path;
  std::string gt_line;
  std::ofstream output(rlt_put_file, std::ios::out);
  float x1, y1, x2, y2;
  while (getline(img_list_file, gt_line)) {
    std::istringstream gt(gt_line);
    gt >> img_path;
    gt >> x1 >> y1 >> x2 >> y2;
    if (x1 < 0 || x2 >= 1920 || y1 < 0 || y2 >= 1080) continue;

    auto xroc_rois = std::make_shared<HobotXRoc::BaseDataVector>();
    xroc_rois->name_ = "face_box";
    auto xroc_roi =
        std::make_shared<HobotXRoc::XRocData<hobot::vision::BBox>>();
    xroc_roi->value.x1 = x1;
    xroc_roi->value.y1 = y1;
    xroc_roi->value.x2 = x2;
    xroc_roi->value.y2 = y2;
    xroc_rois->datas_.push_back(xroc_roi);
    uint32_t effective_w, effective_h;
    fb_handle.GetImgInfo(img_path, &feed_back_info, &effective_w, &effective_h);

#if 0
  static int img_idx = 0;
  std::string yuv_string = "yuv_" + std::to_string(img_idx++) + ".dat";
  HobotXRoc::DumpPyramid(&feed_back_info, yuv_string, 0);
#endif

    HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());

    auto py_img = std::make_shared<hobot::vision::PymImageFrame>();
    py_img->img = feed_back_info;
    auto xroc_data = std::make_shared<HobotXRoc::XRocData<ImageFramePtr>>();
    xroc_data->value =
        std::static_pointer_cast<hobot::vision::ImageFrame>(py_img);
    xroc_data->name_ = "pyramid";

    inputdata->datas_.push_back(
        std::static_pointer_cast<HobotXRoc::BaseData>(xroc_rois));
    inputdata->datas_.push_back(
        std::static_pointer_cast<HobotXRoc::BaseData>(xroc_data));

    auto out = flow->SyncPredict(inputdata);
    if (model_name == "pose_lmk") {
      DumpPoseLmk(out->datas_, output, img_path);
    } else if (model_name == "age_gender") {
      DumpAgeGender(out->datas_, output, img_path);
    } else if (model_name == "anti_spf") {
      DumpAntiSpf(out->datas_, output, img_path);
    } else if (model_name == "face_quality") {
      DumpFaceQuality(out->datas_, output, img_path);
    }
    fb_handle.FreeImgInfo(&feed_back_info);
  }
  delete flow;
  return 0;
}

void DumpAntiSpf(std::vector<HobotXRoc::BaseDataPtr> &result,
                 std::ostream &os,
                 std::string id) {
  os << id;
  std::vector<float> rlts;
  auto anti_spfs =
      std::static_pointer_cast<HobotXRoc::BaseDataVector>(result[0]);
  int target_size = anti_spfs->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto anti_spf = std::static_pointer_cast<
        HobotXRoc::XRocData<hobot::vision::Attribute<int>>>(
        anti_spfs->datas_[target_idx]);
    if (anti_spf->state_ == HobotXRoc::DataState::VALID) {
      rlts.push_back(anti_spf->value.score);
    }
  }
  for (const auto &value : rlts) {
    os << " " << value;
  }
  os << std::endl;
}

void DumpAgeGender(std::vector<HobotXRoc::BaseDataPtr> &result,
                   std::ostream &os,
                   std::string id) {
  os << id;
  auto ages = std::static_pointer_cast<HobotXRoc::BaseDataVector>(result[0]);
  auto genders = std::static_pointer_cast<HobotXRoc::BaseDataVector>(result[1]);
  int target_size = ages->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto age =
        std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::Age>>(
            ages->datas_[target_idx]);
    auto gender =
        std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::Gender>>(
            genders->datas_[target_idx]);
    if (age->state_ != HobotXRoc::DataState::VALID) {
      os << " -1 -1 -1";
      continue;
    }
    os << " " << age->value.min << " " << age->value.max;
    os << " " << gender->value.value;
  }
  os << std::endl;
}

void DumpPoseLmk(std::vector<HobotXRoc::BaseDataPtr> &result,
                 std::ostream &os,
                 std::string id) {
  os << id;
  auto lmks = std::static_pointer_cast<HobotXRoc::BaseDataVector>(result[0]);
  auto poses = std::static_pointer_cast<HobotXRoc::BaseDataVector>(result[1]);
  int target_size = lmks->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto lmk =
        std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::Landmarks>>(
            lmks->datas_[target_idx]);
    auto pose =
        std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::Pose3D>>(
            poses->datas_[target_idx]);
    if (lmk->state_ != HobotXRoc::DataState::VALID) {
      continue;
    }
    for (auto &point : lmk->value.values) {
      os << " " << point.x << " " << point.y;
    }
    os << " " << pose->value.pitch << " " << pose->value.yaw << " "
       << pose->value.roll;
  }
  os << std::endl;
}

void DumpFaceQuality(std::vector<BaseDataPtr> &result,
                     std::ostream &os,
                     std::string id) {
  os << id;
  auto rois = std::static_pointer_cast<HobotXRoc::BaseDataVector>(result[0]);
  int target_size = rois->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto roi =
        std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::BBox>>(
            rois->datas_[target_idx]);
    os << " " << roi->value.x1 << " " << roi->value.y1 << " " << roi->value.x2
       << " " << roi->value.y2;
    for (int attr_idx = 1; attr_idx < result.size(); attr_idx++) {
      auto attrs =
          std::static_pointer_cast<HobotXRoc::BaseDataVector>(result[attr_idx]);
      auto attr = std::static_pointer_cast<
          HobotXRoc::XRocData<hobot::vision::Attribute<int>>>(
          attrs->datas_[target_idx]);
      if (attr->state_ == HobotXRoc::DataState::VALID) {
        if (attr_idx == 2) {
          os << " " << attr->value.value;
        } else {
          os << " " << attr->value.score;
        }
      } else {
        os << " -1.0";
      }
    }
  }
  os << std::endl;
}
