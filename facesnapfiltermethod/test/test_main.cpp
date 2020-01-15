/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     FaceSnapFilterMethod test code
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.01.11
 */

#include <memory>
#include <utility>
#include <vector>
#include <string>
#include "gtest/gtest.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxsdk/xroc_sdk.h"
#include "FaceSnapFilterMethod/FaceSnapFilterMethod.h"
#include "horizon/vision_type/vision_type.hpp"

typedef HobotXRoc::XRocData<hobot::vision::BBox> XRocBBox;
typedef HobotXRoc::XRocData<hobot::vision::Pose3D> XRocPose3D;
typedef HobotXRoc::XRocData<hobot::vision::Landmarks> XRocLandmarks;
typedef HobotXRoc::XRocData<hobot::vision::Quality> XRocQuality;
typedef HobotXRoc::XRocData<hobot::vision::Age> XRocAge;
typedef HobotXRoc::XRocData<hobot::vision::Gender> XRocGender;
typedef HobotXRoc::XRocData<hobot::vision::Attribute<int32_t>> XRocAttribute;
typedef HobotXRoc::XRocData<int32_t> XRocFilterDescription;

namespace HobotXRoc {
class FaceSnapFilterMethodTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    config_path_ = "../../config/face_snap_filter_config.json";
    SetLogLevel(HOBOT_LOG_INFO);
    int size = 2;
    auto box = std::make_shared<XRocBBox>();
    box->type_ = "BBox";
    box->value.x1 = 10;
    box->value.y1 = 1;
    box->value.x2 = 1;
    box->value.y2 = 1;
    box->value.score = 0.92;
    face_box_.push_back(box);

    box = std::make_shared<XRocBBox>();
    box->type_ = "BBox";
    box->value.x1 = 500;
    box->value.y1 = 500;
    box->value.x2 = 800;
    box->value.y2 = 800;
    box->value.score = 0.99;
    face_box_.push_back(box);

    auto pose = std::make_shared<XRocPose3D>();
    pose->type_ = "Pose3D";
    pose->value.pitch = -50;
    pose->value.yaw = 0;
    pose->value.roll = 0;
    pose_.push_back(pose);
    pose = std::make_shared<XRocPose3D>();
    pose->type_ = "Pose3D";
    pose->value.pitch = 0;
    pose->value.yaw = 0;
    pose->value.roll = 0;
    pose_.push_back(pose);
    for (int i = 0; i < 2; ++i) {
      quality_.push_back(std::make_shared<XRocQuality>());
      quality_[i]->value.score = 0.3;
      quality_[i]->type_ = "Quality";
      landmark_.push_back(std::make_shared<XRocLandmarks>());
      hobot::vision::Point one_point;
      one_point.score = 5;
      landmark_[i]->value.values.push_back(one_point);
      landmark_[i]->type_ = "Landmark";
      age_.push_back(std::make_shared<XRocAge>());
      gender_.push_back(std::make_shared<XRocGender>());
      brightness_.push_back(std::make_shared<XRocAttribute>());
      brightness_[i]->value.score = 0.9;
      brightness_[i]->value.value = 0;
      brightness_[i]->type_ = "Attribute";
      occluded_attribute_.push_back(std::make_shared<XRocAttribute>());
      occluded_attribute_[i]->value.score = 0.3;
      occluded_attribute_[i]->type_ = "Attribute";
    }
  }

  virtual void TearDown() {
  }

  std::string config_path_;
  std::vector<std::shared_ptr<XRocBBox>> face_box_;
  std::vector<std::shared_ptr<XRocPose3D>> pose_;
  std::vector<std::shared_ptr<XRocQuality>> quality_;
  std::vector<std::shared_ptr<XRocLandmarks>> landmark_;
  std::vector<std::shared_ptr<XRocAge>> age_;
  std::vector<std::shared_ptr<XRocGender>> gender_;
  std::vector<std::shared_ptr<XRocAttribute>> occluded_attribute_;
  std::vector<std::shared_ptr<XRocAttribute>> brightness_;
};

class FaceSnapFilterMethodParam : public HobotXRoc::InputParam {
 public:
  FaceSnapFilterMethodParam(std::string method_name, std::string json_config)
      : InputParam(std::move(method_name)) {
    content_ = std::move(json_config);
    is_json_format_ = true;
  }
  std::string Format() override { return content_; };
 private:
  std::string content_ = "";
};

TEST_F(FaceSnapFilterMethodTest, default_config) {
  FaceSnapFilterMethod method;
  int ret = method.Init("");

  HobotXRoc::InputParamPtr method_param(
      new FaceSnapFilterMethodParam("filter_example",
                             "{\n"
                             " \"image_width\": 1920,\n"
                             " \"image_height\": 1080,\n"
                             " \"snap_size_thr\": 72,\n"
                             " \"frontal_pitch_thr\": 30,\n"
                             " \"frontal_yaw_thr\": 40,\n"
                             " \"frontal_roll_thr\": 0,\n"
                             " \"pv_thr\": 0.93,\n"
                             " \"quality_thr\": 0.0,\n"
                             " \"lmk_thr\": 1.0,\n"
                             " \"bound_thr_w\": 10,\n"
                             " \"bound_thr_h\": 10,\n"
                             " \"black_area_iou_thr\": 0.5,\n"
                             " \"black_area_list\": [],\n"
                             " \"brightness_min\": 0,\n"
                             " \"brightness_max\": 4,\n"
                             " \"occluded_thr\": 0.5,\n"
                             " \"abnormal_thr\": 1.0,\n"
                             " \"err_description\": {\n"
                             "    \"passed\": 0,\n"
                             "    \"snap_area\": -1,\n"
                             "    \"snap_size_thr\": -2,\n"
                             "    \"frontal_thr\": -3,\n"
                             "    \"pv_thr\": -4,\n"
                             "    \"quality_thr\": -5,\n"
                             "    \"lmk_thr\": -6,\n"
                             "    \"black_list\": -7,\n"
                             "    \"big_face\": -8,\n"
                             "    \"brightness\": -9,\n"
                             "    \"occluded_thr\": -10,\n"
                             "    \"abnormal_thr\": -11\n"
                             " }"
                             "}"));
  method.UpdateParameter(method_param);

  auto param_ptr = method.GetParameter();
  auto content = param_ptr->Format();
  auto param_gt = "{\n\t\"abnormal_thr\" : 1.0,"
                  "\n\t\"black_area_iou_thr\" : 0.5,"
                  "\n\t\"black_area_list\" : [],"
                  "\n\t\"bound_thr_h\" : 10,"
                  "\n\t\"bound_thr_w\" : 10,"
                  "\n\t\"brightness_max\" : 4,"
                  "\n\t\"brightness_min\" : 0,"
                  "\n\t\"err_description\" : "
                  "\n\t{"
                  "\n\t\t\"abnormal_thr\" : -11,"
                  "\n\t\t\"big_face\" : -8,"
                  "\n\t\t\"black_list\" : -7,"
                  "\n\t\t\"brightness\" : -9,"
                  "\n\t\t\"frontal_thr\" : -3,"
                  "\n\t\t\"lmk_thr\" : -6,"
                  "\n\t\t\"occluded_thr\" : -10,"
                  "\n\t\t\"passed\" : 0,"
                  "\n\t\t\"pv_thr\" : -4,"
                  "\n\t\t\"quality_thr\" : -5,"
                  "\n\t\t\"snap_area\" : -1,"
                  "\n\t\t\"snap_size_thr\" : -2"
                  "\n\t},"
                  "\n\t\"frontal_pitch_thr\" : 30,"
                  "\n\t\"frontal_roll_thr\" : 0,"
                  "\n\t\"frontal_yaw_thr\" : 40,"
                  "\n\t\"image_height\" : 1080,"
                  "\n\t\"image_width\" : 1920,"
                  "\n\t\"lmk_thr\" : 1.0,"
                  "\n\t\"occluded_thr\" : 0.5,"
                  "\n\t\"pv_thr\" : 0.93000000000000005,"
                  "\n\t\"quality_thr\" : 0.0,"
                  "\n\t\"snap_size_thr\" : 72"
                  "\n}\n";
  EXPECT_EQ(content, param_gt);
  EXPECT_EQ(ret, 0);
  std::vector<std::vector<BaseDataPtr>> input;
  std::vector<std::vector<BaseDataPtr>> output;
  std::vector<HobotXRoc::InputParamPtr> param;
  input.resize(1);
  param.resize(1);
  auto &frame_input = input[0];
  frame_input.resize(3);
  for (int i = 0; i < 3; ++i) {
    frame_input[i] = std::static_pointer_cast<BaseData>(\
                         std::make_shared<BaseDataVector>());
  }
  auto face_box = std::static_pointer_cast<BaseDataVector>(frame_input[0]);
  auto pose = std::static_pointer_cast<BaseDataVector>(frame_input[1]);
  auto landmark = std::static_pointer_cast<BaseDataVector>(frame_input[2]);
  for (int i = 0; i < face_box_.size(); ++i) {
    face_box->datas_.push_back(\
                std::static_pointer_cast<BaseData>(face_box_[i]));
    pose->datas_.push_back(std::static_pointer_cast<BaseData>(pose_[i]));
    landmark->datas_.push_back(\
                std::static_pointer_cast<BaseData>(landmark_[i]));
  }
  EXPECT_EQ(input[0].size(), 3);
  output = method.DoProcess(input, param);
  EXPECT_EQ(output.size(), 1);
  EXPECT_EQ(output[0].size(), 4);
  auto output_err_code =
      std::static_pointer_cast<BaseDataVector>(output[0][0]);
  auto output_box = std::static_pointer_cast<BaseDataVector>(output[0][1]);
  EXPECT_EQ(output_box->datas_.size(), 2);
  int actual_size = 0;
  for (size_t i = 0; i < output_box->datas_.size(); i++) {
    auto &data = output_box->datas_[i];
    if (data->state_ == DataState::VALID) {
      actual_size++;
    } else {
      auto err_code = std::static_pointer_cast<XRocFilterDescription>(
          output_err_code->datas_[i]);
      EXPECT_EQ(err_code->value, -4);
    }
  }
  EXPECT_EQ(actual_size, 1);
}

TEST_F(FaceSnapFilterMethodTest, basic) {
  FaceSnapFilterMethod method;
  int ret = method.Init(config_path_);

  EXPECT_EQ(ret, 0);
  std::vector<std::vector<BaseDataPtr>> input;
  std::vector<std::vector<BaseDataPtr>> output;
  std::vector<HobotXRoc::InputParamPtr> param;
  input.resize(1);
  param.resize(1);
  auto &frame_input = input[0];
  frame_input.resize(3);
  for (int i = 0; i < 3; ++i) {
    frame_input[i] = std::static_pointer_cast<BaseData>(\
                         std::make_shared<BaseDataVector>());
  }
  auto face_box = std::static_pointer_cast<BaseDataVector>(frame_input[0]);
  auto pose = std::static_pointer_cast<BaseDataVector>(frame_input[1]);
  auto landmark = std::static_pointer_cast<BaseDataVector>(frame_input[2]);
  for (int i = 0; i < face_box_.size(); ++i) {
    face_box->datas_.push_back(\
                std::static_pointer_cast<BaseData>(face_box_[i]));
    pose->datas_.push_back(std::static_pointer_cast<BaseData>(pose_[i]));
    landmark->datas_.push_back(\
                std::static_pointer_cast<BaseData>(landmark_[i]));
  }
  EXPECT_EQ(input[0].size(), 3);
  output = method.DoProcess(input, param);
  EXPECT_EQ(output.size(), 1);
  EXPECT_EQ(output[0].size(), 4);
  auto output_err_code =
      std::static_pointer_cast<BaseDataVector>(output[0][0]);
  auto output_box = std::static_pointer_cast<BaseDataVector>(output[0][1]);
  EXPECT_EQ(output_box->datas_.size(), 2);
  int actual_size = 0;
  for (size_t i = 0; i < output_box->datas_.size(); i++) {
    auto &data = output_box->datas_[i];
    if (data->state_ == DataState::VALID) {
      actual_size++;
    } else {
      auto err_code = std::static_pointer_cast<XRocFilterDescription>(
          output_err_code->datas_[i]);
      EXPECT_EQ(err_code->value, -5);
    }
  }
  EXPECT_EQ(actual_size, 1);
}

TEST_F(FaceSnapFilterMethodTest, normal) {
  FaceSnapFilterMethod method;
  int ret = method.Init(config_path_);
  EXPECT_EQ(ret, 0);
  std::vector<std::vector<BaseDataPtr>> input;
  std::vector<std::vector<BaseDataPtr>> output;
  std::vector<HobotXRoc::InputParamPtr> param;
  input.resize(1);
  param.resize(1);
  auto &frame_input = input[0];
  frame_input.resize(6);
  for (int i = 0; i < 6; ++i) {
    frame_input[i] = std::static_pointer_cast<BaseData>(\
                         std::make_shared<BaseDataVector>());
  }
  auto face_box = std::static_pointer_cast<BaseDataVector>(frame_input[0]);
  auto pose = std::static_pointer_cast<BaseDataVector>(frame_input[1]);
  auto landmark = std::static_pointer_cast<BaseDataVector>(frame_input[2]);
  auto quality = std::static_pointer_cast<BaseDataVector>(frame_input[3]);
  auto brightness = std::static_pointer_cast<BaseDataVector>(frame_input[4]);
  auto occluded_attribute =
      std::static_pointer_cast<BaseDataVector>(frame_input[5]);
  for (int i = 0; i < face_box_.size(); ++i) {
    face_box->datas_.push_back(\
                std::static_pointer_cast<BaseData>(face_box_[i]));
    pose->datas_.push_back(std::static_pointer_cast<BaseData>(pose_[i]));
    quality->datas_.push_back(std::static_pointer_cast<BaseData>(quality_[i]));
    landmark->datas_.push_back(\
                std::static_pointer_cast<BaseData>(landmark_[i]));
    brightness->datas_.push_back(\
        std::static_pointer_cast<BaseData>(brightness_[i]));
    brightness->datas_[i]->type_ = "Attribute";
    occluded_attribute->datas_.push_back(\
        std::static_pointer_cast<BaseData>(occluded_attribute_[i]));
    occluded_attribute->datas_[i]->type_ = "Attribute";
  }
  EXPECT_EQ(input[0].size(), 6);

  output = method.DoProcess(input, param);
  EXPECT_EQ(output.size(), 1);
  EXPECT_EQ(output[0].size(), 7);
  auto output_err_code =
      std::static_pointer_cast<BaseDataVector>(output[0][0]);
  auto output_box = std::static_pointer_cast<BaseDataVector>(output[0][1]);
  EXPECT_EQ(output_box->datas_.size(), 2);
  int actual_size = 0;
  for (size_t i = 0; i < output_box->datas_.size(); i++) {
    auto &data = output_box->datas_[i];
    if (data->state_ == DataState::VALID) {
      actual_size++;
    } else {
      auto err_code = std::static_pointer_cast<XRocFilterDescription>(
          output_err_code->datas_[i]);
      EXPECT_EQ(err_code->value, -5);
    }
  }
  EXPECT_EQ(actual_size, 1);
}

TEST_F(FaceSnapFilterMethodTest, big_face_mode) {
  FaceSnapFilterMethod method;
  int ret = method.Init("");

  HobotXRoc::InputParamPtr method_param(
      new FaceSnapFilterMethodParam("grading_example",
                                    "{\n"
                                    " \"image_width\": 1920,\n"
                                    " \"max_box_counts\": 1,\n"
                                    " \"image_height\": 1080,\n"
                                    " \"snap_size_thr\": -10,\n"
                                    " \"frontal_thr\": 0,\n"
                                    " \"pv_thr\": 0.00,\n"
                                    " \"quality_thr\": 0.0,\n"
                                    " \"lmk_thr\": 0.0,\n"
                                    " \"bound_thr_w\": 0,\n"
                                    " \"bound_thr_h\": 0,\n"
                                    " \"black_area_iou_thr\": 0.0,\n"
                                    " \"black_area_list\": [],\n"
                                    " \"err_description\": {\n"
                                    "    \"passed\": 0,\n"
                                    "    \"snap_area\": -1,\n"
                                    "    \"snap_size_thr\": -2,\n"
                                    "    \"frontal_thr\": -3,\n"
                                    "    \"pv_thr\": -4,\n"
                                    "    \"quality_thr\": -5,\n"
                                    "    \"lmk_thr\": -6,\n"
                                    "    \"black_list\": -7,\n"
                                    "    \"big_face\": -8,\n"
                                    "    \"brightness\": -9,\n"
                                    "    \"occluded_thr\": -10,\n"
                                    "    \"abnormal_thr\": -11\n"
                                    " }"
                                    "}"));
  ret = method.UpdateParameter(method_param);

  EXPECT_EQ(ret, 0);
  std::vector<std::vector<BaseDataPtr>> input;
  std::vector<std::vector<BaseDataPtr>> output;
  std::vector<HobotXRoc::InputParamPtr> param;
  input.resize(1);
  param.resize(1);
  auto &frame_input = input[0];
  frame_input.resize(1);
  for (int i = 0; i < 1; ++i) {
    frame_input[i] = std::static_pointer_cast<BaseData>(\
                         std::make_shared<BaseDataVector>());
  }
  auto face_box = std::static_pointer_cast<BaseDataVector>(frame_input[0]);

  for (int i = 0; i < face_box_.size(); ++i) {
    face_box->datas_.push_back(\
                std::static_pointer_cast<BaseData>(face_box_[i]));
  }
  EXPECT_EQ(input[0].size(), 1);
  output = method.DoProcess(input, param);
  EXPECT_EQ(output.size(), 1);
  EXPECT_EQ(output[0].size(), 2);
  auto output_err_code = std::static_pointer_cast<BaseDataVector>(output[0][0]);
  EXPECT_EQ(output_err_code->datas_.size(), 2);
  int actual_size = 0;
  for (const auto &data : output_err_code->datas_) {
    if (data->state_ == DataState::VALID) {
      actual_size++;
    }
  }
  EXPECT_EQ(actual_size, 1);
  auto err_code = std::static_pointer_cast<XRocFilterDescription>(
      output_err_code->datas_[0]);
  EXPECT_EQ(err_code->value, -8);
  err_code = std::static_pointer_cast<XRocFilterDescription>(
      output_err_code->datas_[1]);
  EXPECT_EQ(err_code->value, 0);
  auto output_box = std::static_pointer_cast<BaseDataVector>(output[0][1]);
  EXPECT_EQ(output_box->datas_.size(), 2);
  actual_size = 0;
  for (const auto &data : output_box->datas_) {
    if (data->state_ == DataState::VALID) {
      actual_size++;
    }
  }
  EXPECT_EQ(actual_size, 1);
}

TEST_F(FaceSnapFilterMethodTest, pass_through) {
  FaceSnapFilterMethod method;
  int ret = method.Init(config_path_);

  EXPECT_EQ(ret, 0);
  std::vector<std::vector<BaseDataPtr>> input;
  std::vector<std::vector<BaseDataPtr>> output;
  std::vector<HobotXRoc::InputParamPtr> param;
  input.resize(1);
  auto &frame_input = input[0];
  frame_input.resize(3);
  for (int i = 0; i < 3; ++i) {
    frame_input[i] = std::static_pointer_cast<BaseData>(\
                         std::make_shared<BaseDataVector>());
  }
  auto face_box = std::static_pointer_cast<BaseDataVector>(frame_input[0]);
  auto pose = std::static_pointer_cast<BaseDataVector>(frame_input[1]);
  auto landmark = std::static_pointer_cast<BaseDataVector>(frame_input[2]);
  for (int i = 0; i < face_box_.size(); ++i) {
    face_box->datas_.push_back(\
                std::static_pointer_cast<BaseData>(face_box_[i]));
    pose->datas_.push_back(std::static_pointer_cast<BaseData>(pose_[i]));
    landmark->datas_.push_back(\
                std::static_pointer_cast<BaseData>(landmark_[i]));
  }

  HobotXRoc::InputParamPtr pass_through_param(
      new FaceSnapFilterMethodParam("filter_example", "pass-through"));
  pass_through_param->is_json_format_ = false;
  param.emplace_back(pass_through_param);
  EXPECT_EQ(input[0].size(), 3);
  output = method.DoProcess(input, param);
  EXPECT_EQ(output.size(), 1);
  EXPECT_EQ(output[0].size(), 4);
  auto output_err_code =
      std::static_pointer_cast<BaseDataVector>(output[0][0]);
  auto output_box = std::static_pointer_cast<BaseDataVector>(output[0][1]);
  EXPECT_EQ(output_box->datas_.size(), 2);
  int actual_size = 0;
  for (auto & data : output_box->datas_) {
    if (data->state_ == DataState::VALID) {
      actual_size++;
    }
  }
  EXPECT_EQ(actual_size, 2);
}

}  // namespace HobotXRoc
