/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Method interface of xRoc framework
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.25
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <cassert>
#include <unistd.h>

#include "hobotxsdk/xroc_sdk.h"
#include "horizon/vision_type/vision_type.hpp"

typedef HobotXRoc::XRocData<hobot::vision::BBox> XRocBBox;
typedef HobotXRoc::XRocData<hobot::vision::Pose3D> XRocPose3D;
typedef HobotXRoc::XRocData<hobot::vision::Landmarks> XRocLandmarks;
typedef HobotXRoc::XRocData<hobot::vision::Quality> XRocQuality;
typedef HobotXRoc::XRocData<float> XRocFloat;

class Callback {
 public:
  Callback() = default;

  ~Callback() = default;

  void OnCallback(const HobotXRoc::OutputDataPtr &output) {
    using HobotXRoc::BaseDataVector;
    std::cout << "======================" << std::endl;
    std::cout << "seq: " << output->sequence_id_ << std::endl;
    std::cout << "error_code: " << output->error_code_ << std::endl;
    std::cout << "error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "datas_ size: " << output->datas_.size() << std::endl;
    for (const auto &data : output->datas_) {
      if (data->error_code_ < 0) {
        std::cout << "data error: " << data->error_code_ << std::endl;
        continue;
      }
      std::cout << "data type_name : " << data->type_ << " " << data->name_
                << std::endl;
      auto *pdata = dynamic_cast<BaseDataVector *>(data.get());
      std::cout << "pdata size: " << pdata->datas_.size() << std::endl;
      for (const auto &item : pdata->datas_) {
        assert("Number" == item->type_);
        //  get the item score
        auto select_score = std::static_pointer_cast<XRocFloat>(item);
        std::cout << "select_score:" << select_score->value
                  << std::endl;
      }
    }
  }
};

int main(int argc, char const *argv[]) {
  using HobotXRoc::BaseData;
  using HobotXRoc::BaseDataPtr;
  using HobotXRoc::BaseDataVector;
  using HobotXRoc::InputData;
  using HobotXRoc::InputDataPtr;

  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/grading.json");
  flow->Init();

  InputDataPtr inputdata(new InputData());

  // construct the box_list and put it into datas_[0]
  std::shared_ptr<BaseDataVector> box_list(new BaseDataVector());
  box_list->name_ = "box_list";

  std::shared_ptr<XRocBBox> bbox1(new XRocBBox());
  bbox1->type_ = "BBox";
  bbox1->value.x1 = 861;
  bbox1->value.y1 = 995;
  bbox1->value.x2 = 917;
  bbox1->value.y2 = 1063;
  bbox1->value.score = 1.084960;
  std::shared_ptr<XRocBBox> bbox2(new XRocBBox());
  bbox2->type_ = "BBox";
  bbox2->value.x1 = 1000;
  bbox2->value.y1 = 140;
  bbox2->value.x2 = 1061;
  bbox2->value.y2 = 202;
  bbox2->value.score = 0.9901622;
  box_list->datas_.push_back(bbox1);
  box_list->datas_.push_back(bbox2);
//  box_list->state_ = HobotXRoc::DataState::INVALID;
  inputdata->datas_.push_back(box_list);

  // construct the pose_3d_list and put it into datas_[1]
  std::shared_ptr<BaseDataVector> pose_3d_list(new BaseDataVector());
  pose_3d_list->name_ = "pose_3d_list";

  std::shared_ptr<XRocPose3D> pose1(new XRocPose3D());
  pose1->value.pitch = -14.5738907;
  pose1->value.yaw = 19.8864212;
  pose1->value.roll = -6.61847687;

  std::shared_ptr<XRocPose3D> pose2(new XRocPose3D());
  pose2->value.pitch = -15.0597382;
  pose2->value.yaw = 2.414505;
  pose2->value.roll = -1.443367;
  pose_3d_list->datas_.push_back(pose1);
  pose_3d_list->datas_.push_back(pose2);
//  pose_3d_list->state_ = HobotXRoc::DataState::INVALID;
  inputdata->datas_.push_back(pose_3d_list);

  // construct the land_mark_list and put it into datas_[2]
  std::shared_ptr<BaseDataVector> land_mark_list(new BaseDataVector());
  land_mark_list->name_ = "land_mark_list";

  std::shared_ptr<XRocLandmarks> lmk1(new XRocLandmarks());
  hobot::vision::Point point0(881, 1021, 13);
  hobot::vision::Point point1(907, 1079, 12);
  hobot::vision::Point point2(897, 1036, 13);
  hobot::vision::Point point3(884, 1047, 12);
  hobot::vision::Point point4(905, 1045, 13);
  lmk1->value.values = { point0, point1, point2, point3, point4 };

  std::shared_ptr<XRocLandmarks> lmk2(new XRocLandmarks());
  hobot::vision::Point point5(1020, 156, 9);
  hobot::vision::Point point6(1047, 159, 4);
  hobot::vision::Point point7(1034, 169, 13);
  hobot::vision::Point point8(1020, 180, 9);
  hobot::vision::Point point9(1041, 182, 9);
  lmk2->value.values = { point5, point6, point7, point8, point9 };

  land_mark_list->datas_.push_back(lmk1);
  land_mark_list->datas_.push_back(lmk2);
//  land_mark_list->state_ = HobotXRoc::DataState::INVALID;
  inputdata->datas_.push_back(land_mark_list);

  // construct the quality_list and put it into datas_[3]
  std::shared_ptr<BaseDataVector> quality_list(new BaseDataVector());
  quality_list->name_ = "quality_list";
  std::shared_ptr<XRocQuality> quality1(new XRocQuality());
  quality1->value.value = 0;
//  quality1->value.value = 75;
//  quality1->value.value = 0.3125f;

  std::shared_ptr<XRocQuality> quality2(new XRocQuality());
  quality2->value.value = 0;
//  quality2->value.value = 11;
//  quality2->value.value = 0.472499996f;

  quality_list->datas_.push_back(quality1);
  quality_list->datas_.push_back(quality2);
//  quality_list->state_ = HobotXRoc::DataState::INVALID;
  inputdata->datas_.push_back(quality_list);

  // sync GradingMethod
  auto out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);

  // async GradingMethod
  flow->AsyncPredict(inputdata);
  sleep(1);

  delete flow;
  return 0;
}
