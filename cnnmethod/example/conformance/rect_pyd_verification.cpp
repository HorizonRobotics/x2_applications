/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: rect_pyd_verification.cpp
 * @Brief: conformance validation of rect+pyramid
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-05-22 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-05-22 15:18:10
 */

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "3rd_party_lib/plat_cnn.h"
#include "CNNMethod/Predictor/Predictor.h"
#include "CNNMethod/Predictor/PredictorFactory.h"
#include "CNNMethod/util/CNNMethodConfig.h"
#include "CNNMethod/util/CNNMethodData.h"
#include "CNNMethod/util/util.h"
#include "bpu_predict/bpu_io.h"
#include "bpu_predict/bpu_predict.h"
#include "hbdk/hbdk_layout.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/method.h"
#include "hobotxsdk/xroc_data.h"
#include "hobotxsdk/xroc_sdk.h"
#include "util/hb_vio_wrapper.h"
#include "horizon/vision_type/vision_type.hpp"
#include "horizon/vision_type/vision_type_common.h"

typedef std::shared_ptr<hobot::vision::ImageFrame> ImageFramePtr;
void DumpInvalidCommon(const std::vector<std::vector<uint32_t>> &real_nhwc,
                       std::ostream &os);
void DumpValidCommon(std::vector<std::vector<int8_t>> &target_rlt,
                     const std::vector<std::vector<uint32_t>> &real_nhwc,
                     std::ostream &os);
void DumpLmk(std::vector<std::vector<int8_t>> &mxnet_outs,
             const hobot::vision::BBox &box,
             const std::vector<std::vector<uint32_t>> nhwc,
             std::ostream &os);
void DumpPose3D(std::vector<int8_t> &mxnet_outs, std::ostream &os);

static void Usage() {
  std::cout << "./example ver_rect_pyd method_cfg_file hb_vio_cfg_file gt.txt "
               "out_file"
            << std::endl;
}

void DumpMxnetOutput(HobotXRoc::CNNMethodRunData *run_data,
                     std::ostream &os,
                     const std::string &post_fn,
                     const std::string &img_path) {
  auto &result = run_data->mxnet_output;
  auto &elem_size = run_data->elem_size;
  auto &real_nhwc = run_data->real_nhwc;

  for (int frame_idx = 0; frame_idx < result.size(); frame_idx++) {
    auto &frame_rlt = result[frame_idx];
    int valid_target_idx = 0;
    for (int target_idx = 0; target_idx < frame_rlt.size(); target_idx++) {
      auto &target_rlt = frame_rlt[target_idx];
      if (target_rlt.size()) {
        os << img_path;
        if (post_fn == "lmk_pose") {
          auto boxes = std::static_pointer_cast<HobotXRoc::BaseDataVector>(
              (*(run_data->input))[frame_idx][0]);
          auto xroc_box = std::static_pointer_cast<
              HobotXRoc::XRocData<hobot::vision::BBox>>(
              boxes->datas_[target_idx]);
          DumpLmk(target_rlt, xroc_box->value, real_nhwc, os);
          DumpPose3D(target_rlt[3], os);
        } else {
          DumpValidCommon(target_rlt, real_nhwc, os);
        }
        os << std::endl;
      }
    }
  }
}
void DumpValidCommon(std::vector<std::vector<int8_t>> &target_rlt,
                     const std::vector<std::vector<uint32_t>> &real_nhwc,
                     std::ostream &os) {
  for (int layer_idx = 0; layer_idx < target_rlt.size(); layer_idx++) {
    auto &layer_nhwc = real_nhwc[layer_idx];
    int elem_num =
        layer_nhwc[0] * layer_nhwc[1] * layer_nhwc[2] * layer_nhwc[3];
    auto layer_rlt = reinterpret_cast<float *>(target_rlt[layer_idx].data());
    for (int elem_idx = 0; elem_idx < elem_num; elem_idx++) {
      os << " " << std::fixed << std::setprecision(5) << layer_rlt[elem_idx];
    }
  }
}

void DumpInvalidCommon(const std::vector<std::vector<uint32_t>> &real_nhwc,
                       std::ostream &os) {
  for (int layer_idx = 0; layer_idx < real_nhwc.size(); layer_idx++) {
    auto &layer_nhwc = real_nhwc[layer_idx];
    int elem_num =
        layer_nhwc[0] * layer_nhwc[1] * layer_nhwc[2] * layer_nhwc[3];
    for (int elem_idx = 0; elem_idx < elem_num; elem_idx++) {
      os << " " << std::fixed << std::setprecision(5) << -1.0f;
    }
  }
}

void DumpLmk(std::vector<std::vector<int8_t>> &mxnet_outs,
             const hobot::vision::BBox &box,
             const std::vector<std::vector<uint32_t>> nhwc,
             std::ostream &os) {
  static const float SCORE_THRESH = 0.0;
  static const float REGRESSION_RADIUS = 3.0;
  static const float STRIDE = 4.0;
  static const float num = 1;
  static const float height_m = 16;
  static const float width_m = 16;

  auto fl_scores = reinterpret_cast<float *>(mxnet_outs[0].data());
  auto fl_coords = reinterpret_cast<float *>(mxnet_outs[1].data());
  std::vector<std::vector<float>> points_score;
  std::vector<std::vector<float>> points_x;
  std::vector<std::vector<float>> points_y;
  points_score.resize(5);
  points_x.resize(5);
  points_y.resize(5);

  // nhwc, 1x16x16x5, 1x16x16x10
  for (int n = 0; n < num; ++n) {          // n
    for (int i = 0; i < height_m; ++i) {   // h
      for (int j = 0; j < width_m; ++j) {  // w
        int index_score = n * nhwc[0][1] * nhwc[0][2] * nhwc[0][3]
                          + i * nhwc[0][2] * nhwc[0][3] + j * nhwc[0][3];
        int index_coords = n * nhwc[1][1] * nhwc[1][2] * nhwc[0][3]
                           + i * nhwc[1][2] * nhwc[1][3] + j * nhwc[1][3];
        for (int k = 0; k < 5; ++k) {  // c
          auto score = fl_scores[index_score + k];
          if (score > SCORE_THRESH) {
            points_score[k].push_back(score);
            float x =
                (j + 0.5 - fl_coords[index_coords + 2 * k] * REGRESSION_RADIUS)
                * STRIDE;
            float y =
                (i + 0.5
                 - fl_coords[index_coords + 2 * k + 1] * REGRESSION_RADIUS)
                * STRIDE;
            x = std::min(std::max(x, 0.0f), width_m * STRIDE);
            y = std::min(std::max(y, 0.0f), height_m * STRIDE);
            points_x[k].push_back(x);
            points_y[k].push_back(y);
          }
        }
      }
    }
  }
  os << " " << box.x1 << " " << box.y1 << " " << box.x2 << " " << box.y2;
  std::vector<int> scores(5);
  for (int i = 0; i < 5; ++i) {
    float x = HobotXRoc::Mean(points_x[i]);
    float y = HobotXRoc::Mean(points_y[i]);
    x = box.x1 + x / 64 * (box.x2 - box.x1);
    y = box.y1 + y / 64 * (box.y2 - box.y1);
    scores[i] = static_cast<float>(points_score[i].size()) > 0.000001 ? 1 : 0;
    os << " " << std::fixed << std::setprecision(5) << x << " " << y;
  }
  for (int i = 0; i < 5; i++) {
    os << " " << scores[i];
  }
  /* lmks1 post process */
  std::vector<float> lmks1(10);
  auto reg_coords = reinterpret_cast<float *>(mxnet_outs[2].data());
  for (int i = 0; i < 5; i++) {
    lmks1[i << 1] = box.x1 + reg_coords[i << 1] * (box.x2 - box.x1);
    lmks1[(i << 1) + 1] = box.y1 + reg_coords[(i << 1) + 1] * (box.y2 - box.y1);
  }
  for (int i = 0; i < 10; i++) {
    os << " " << std::fixed << std::setprecision(5) << lmks1[i];
  }
}

void DumpPose3D(std::vector<int8_t> &mxnet_outs, std::ostream &os) {
  auto mxnet_out = reinterpret_cast<float *>(mxnet_outs.data());
  float yaw = mxnet_out[0] * 90.0;
  float pitch = mxnet_out[1] * 90.0;
  float roll = mxnet_out[2] * 90.0;
  os << " " << yaw << " " << pitch << " " << roll;
}

int DoVerRectPyd(int argc, char **argv) {
  if (argc < 5) {
    Usage();
    return -1;
  }
  std::string cfg_path = argv[1];
  std::string post_fn;
  std::ifstream infile(cfg_path.c_str());
  std::stringstream buffer;
  buffer << infile.rdbuf();
  auto config = std::shared_ptr<HobotXRoc::CNNMethodConfig>(
      new HobotXRoc::CNNMethodConfig(buffer.str()));
  config->SetSTDStringValue("parent_path",
                            HobotXRoc::get_parent_path(cfg_path));

  HobotXRoc::Predictor *predictor =
      HobotXRoc::PredictorFactory::GetPredictor(HobotXRoc::InputType::RECT);
  predictor->Init(config);
  post_fn = config->GetSTDStringValue("post_fn");

  std::string fb_cfg = argv[2];
  img_info_t feed_back_info;
  HbVioFbWrapper fb_handle(fb_cfg);
  fb_handle.Init();

  std::string gt_list = argv[3];
  std::string input_image;
  float x1, y1, x2, y2;

  std::ifstream f_rect(gt_list);
  std::string gt_line;
  std::string output_file = argv[4];
  std::ofstream output(output_file, std::ios::out);
  while (getline(f_rect, gt_line)) {
    std::istringstream gt(gt_line);
    gt >> input_image;
    // pyramid
    uint32_t effective_w, effective_h;
    fb_handle.GetImgInfo(
        input_image, &feed_back_info, &effective_w, &effective_h);

#if 0
  static int img_idx = 0;
  std::string yuv_string = "yuv_" + std::to_string(img_idx++) + ".dat";
  HobotXRoc::DumpPyramid(&feed_back_info, yuv_string, 0);
#endif

    gt >> x1 >> y1 >> x2 >> y2;
    if (x1 < 0 || x2 >= 1920 || y1 < 0 || y2 >= 1080) continue;
    // rect
    std::vector<std::vector<float>> rois;
    rois.resize(1);
    rois[0].push_back(x1);
    rois[0].push_back(y1);
    rois[0].push_back(x2);
    rois[0].push_back(y2);

    HobotXRoc::CNNMethodRunData run_data;
    std::vector<std::vector<HobotXRoc::BaseDataPtr>> input;
    input.resize(1);

    auto py_img = std::make_shared<hobot::vision::PymImageFrame>();
    py_img->img = feed_back_info;
    auto xroc_data = std::make_shared<HobotXRoc::XRocData<ImageFramePtr>>();
    xroc_data->value =
        std::static_pointer_cast<hobot::vision::ImageFrame>(py_img);

    auto xroc_rois = std::make_shared<HobotXRoc::BaseDataVector>();
    for (auto &roi : rois) {
      auto xroc_roi =
          std::make_shared<HobotXRoc::XRocData<hobot::vision::BBox>>();
      xroc_roi->value.x1 = roi[0];
      xroc_roi->value.y1 = roi[1];
      xroc_roi->value.x2 = roi[2];
      xroc_roi->value.y2 = roi[3];
      xroc_rois->datas_.push_back(xroc_roi);
    }
    input[0].push_back(
        std::static_pointer_cast<HobotXRoc::BaseData>(xroc_rois));
    input[0].push_back(
        std::static_pointer_cast<HobotXRoc::BaseData>(xroc_data));
    run_data.input = &input;
    predictor->Do(&run_data);
    DumpMxnetOutput(&run_data, output, post_fn, input_image);
    fb_handle.FreeImgInfo(&feed_back_info);
  }
  delete predictor;
}
