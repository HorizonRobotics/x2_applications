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
#include "opencv2/opencv.hpp"
#include "util/hb_vio_wrapper.h"
#include "horizon/vision_type/vision_type.hpp"
#include "horizon/vision_type/vision_type_common.h"

typedef std::shared_ptr<hobot::vision::ImageFrame> ImageFramePtr;
using hobot::vision::BBox;
using HobotXRoc::BaseDataPtr;
using HobotXRoc::BaseDataVector;
using HobotXRoc::XRocData;
static void Usage() {
  std::cout << "./example do_fb_det_cnn [pose_lmk|age_gender|anti_spf] "
               "xroc_cfg_file fb_cfg img_list out_file\n";
}
extern void
DumpAgeGender(std::vector<BaseDataPtr> &, std::ostream &, std::string);
extern void
DumpPoseLmk(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void PrintPoseLmk(std::vector<BaseDataPtr> &);
void DumpAntiSpfAndRoi(std::vector<BaseDataPtr> &, std::ostream &, std::string);

int DoFbDetCNN(int argc, char **argv) {
  if (argc < 6) {
    Usage();
    return 1;
  }
  std::string model_name(argv[1]);
  std::string cfg_file(argv[2]);
  std::string fb_cfg(argv[3]);
  std::string img_list(argv[4]);
  std::string output_file(argv[5]);

  HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
  flow->SetConfig("config_file", cfg_file.c_str());
  flow->Init();

  img_info_t feed_back_info;
  HbVioFbWrapper fb_handle(fb_cfg);
  fb_handle.Init();

  std::ifstream img_list_file(img_list);
  std::string img_path;
  std::ofstream output(output_file, std::ios::out);
  while (getline(img_list_file, img_path)) {
    std::vector<std::string> strs;
    HobotXRoc::split_string(img_path, strs, " ");
    uint32_t effective_w, effective_h;
    fb_handle.GetImgInfo(strs[0], &feed_back_info, &effective_w, &effective_h);
    HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());

    auto py_img = std::make_shared<hobot::vision::PymImageFrame>();
    py_img->img = feed_back_info;
    auto xroc_data = std::make_shared<HobotXRoc::XRocData<ImageFramePtr>>();
    xroc_data->value =
        std::static_pointer_cast<hobot::vision::ImageFrame>(py_img);
    xroc_data->name_ = "pyramid";

    inputdata->datas_.push_back(
        std::static_pointer_cast<HobotXRoc::BaseData>(xroc_data));

    auto out = flow->SyncPredict(inputdata);
    if (model_name == "pose_lmk") {
      DumpPoseLmk(out->datas_, output, strs[0]);
      PrintPoseLmk(out->datas_);
    } else if (model_name == "age_gender") {
      DumpAgeGender(out->datas_, output, strs[0]);
    } else if (model_name == "anti_spf") {
      DumpAntiSpfAndRoi(out->datas_, output, strs[0]);
    }
    fb_handle.FreeImgInfo(&feed_back_info);
  }
  delete flow;
  return 0;
}

void PrintPoseLmk(std::vector<HobotXRoc::BaseDataPtr> &result) {
  auto lmks = std::static_pointer_cast<HobotXRoc::BaseDataVector>(result[0]);
  auto poses = std::static_pointer_cast<HobotXRoc::BaseDataVector>(result[1]);
  auto xroc_pyramid =
      std::static_pointer_cast<HobotXRoc::XRocData<ImageFramePtr>>(result[2]);
  auto pyramid = std::static_pointer_cast<hobot::vision::PymImageFrame>(
      xroc_pyramid->value);
  auto rois = std::static_pointer_cast<HobotXRoc::BaseDataVector>(result[3]);

  static int dump_index = 0;
  auto &img_info = pyramid->img.src_img;
  auto height = img_info.height;
  auto width = img_info.width;
  auto y_addr = img_info.y_vaddr;
  auto uv_addr = img_info.c_vaddr;
  cv::Mat yuv420p;
  yuv420p.create(height * 3 / 2, width, CV_8UC1);
  memcpy(yuv420p.data, reinterpret_cast<uint8_t *>(y_addr), height * width);
  memcpy(yuv420p.data + height * width,
         reinterpret_cast<uint8_t *>(uv_addr),
         height * width / 2);
  cv::Mat img;
  cv::cvtColor(yuv420p, img, CV_YUV2BGR_NV12);
  int target_size = lmks->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto lmk =
        std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::Landmarks>>(
            lmks->datas_[target_idx]);
    auto pose =
        std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::Pose3D>>(
            poses->datas_[target_idx]);
    if (lmk->state_ != HobotXRoc::DataState::VALID) {
      std::cout << "lmk invalid";
      continue;
    }
    for (auto &point : lmk->value.values) {
      cv::rectangle(img,
                    cv::Point(point.x - 2, point.y - 2),
                    cv::Point(point.x + 2, point.y + 2),
                    CV_RGB(255, 0, 0),
                    2);
    }
  }
  for (int roi_idx = 0; roi_idx < rois->datas_.size(); roi_idx++) {
    auto roi =
        std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::BBox>>(
            rois->datas_[roi_idx]);
    cv::rectangle(img,
                  cv::Point(roi->value.x1, roi->value.y1),
                  cv::Point(roi->value.x2, roi->value.y2),
                  CV_RGB(0, 255, 0),
                  2);
  }
  cv::imwrite(std::to_string(dump_index++) + "_pose_lmk.jpg", img);
}

void DumpAntiSpfAndRoi(std::vector<HobotXRoc::BaseDataPtr> &result,
                       std::ostream &os,
                       std::string id) {
  os << id;
  auto anti_spfs = std::static_pointer_cast<BaseDataVector>(result[0]);
  auto rois = std::static_pointer_cast<BaseDataVector>(result[1]);
  int target_size = anti_spfs->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto anti_spf = std::static_pointer_cast<
        HobotXRoc::XRocData<hobot::vision::Attribute<int>>>(
        anti_spfs->datas_[target_idx]);
    auto p_roi =
        std::static_pointer_cast<XRocData<BBox>>(rois->datas_[target_idx]);
    if (anti_spf->state_ == HobotXRoc::DataState::VALID) {
      os << " " << p_roi->value.x1 << " " << p_roi->value.y1 << " "
         << p_roi->value.x2 << " " << p_roi->value.y2;
      os << " " << anti_spf->value.score;
    }
  }
  os << std::endl;
}
