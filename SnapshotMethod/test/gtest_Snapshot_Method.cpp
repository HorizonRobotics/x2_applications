/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     gtest of Select Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.01.07
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <cassert>
#include <fstream>
#include <sstream>
#include <memory>

#include "gtest/gtest.h"
#include "hobotxsdk/xroc_sdk.h"
#include "test_support.hpp"
#include "hobotlog/hobotlog.hpp"
#include "SnapShotMethod/strategy/first_num_best.h"
#include "SnapShotMethod/snapshot_data_type/snapshot_data_type.hpp"

class XRocSelectMethodTest : public ::testing::Test {
 public:
  XRocSelectMethodTest() = default;

  class Callback {
   public:
    Callback() = default;
    ~Callback() = default;

    int SetImgSaveDir(const std::string &path) {
      save_dir_ = path;
      mkdir(save_dir_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    std::string GetLog() {
      return log_;
    }

    std::string GetSimpleLog() {
      return simple_log_;
    }

    void OnCallback(const HobotXRoc::OutputDataPtr &output) {
      using HobotXRoc::BaseDataVector;
      assert(output->datas_.size() >= 1);
      auto &data = output->datas_[0];
      if (data->error_code_ < 0) {
        std::cout << "data error: " << data->error_code_ << std::endl;
      }
      auto *psnap_data = dynamic_cast<BaseDataVector*>(data.get());
      if (!psnap_data->datas_.empty()) {
        for (const auto &item : psnap_data->datas_) {
          assert("BaseDataVector" == item->type_);
          //  get the item score
          auto one_target_snapshot_info =
              std::static_pointer_cast<BaseDataVector>(item);

          for (auto &snapshot_info : one_target_snapshot_info->datas_) {
            auto one_snap =
                std::static_pointer_cast<XRocSnapshotInfo>(snapshot_info);
            if (one_snap->value->snap) {
              DumpSnap(one_snap, save_dir_);
            }
            char s_log_tmp[256];
            snprintf(s_log_tmp, sizeof(s_log_tmp),
                    "track_id: %d, frame_id: %li\n",
                    one_snap->value->track_id,
                    one_snap->value->origin_image_frame->frame_id);
            simple_log_ += s_log_tmp;
            char log_tmp[256];
            snprintf(log_tmp, sizeof(log_tmp),
                     "track_id:%d frame_id:%li select_score:%f",
                     one_snap->value->track_id,
                     one_snap->value->origin_image_frame->frame_id,
                     one_snap->value->select_value);
            log_ += log_tmp;
            auto userdatas = one_snap->value->userdata;
            for (auto i = 0; i < userdatas.size(); i++) {
              auto pU = userdatas[i].get();
              if (userdatas[i]->state_ == HobotXRoc::DataState::VALID) {
                log_ += " userdata_";
                log_ += std::to_string(i);
                log_ += ":";
                log_ += userdatas[i]->name_;
              }
            }
            log_ += " ";
          }
        }
      }
      log_ += "\n";

      if (output->datas_.size() > 1) {
        auto &data = output->datas_[1];
        auto *psnap_data = dynamic_cast<BaseDataVector*>(data.get());
        if (!psnap_data->datas_.empty()) {
          for (const auto &item : psnap_data->datas_) {
            auto snap_state =
              std::static_pointer_cast<HobotXRoc::XRocSnapshotState>(item);
            auto &value = snap_state->value;
            LOGI << "snap id: " << value->id << " "
                      << "snap en: " << (value->snap_en ? 1 : 0) << " "
                      << "snap stop: " << (value->snap_stop ? 1 : 0) << " "
                      << "snap repeat: " << (value->snap_repeat ? 1 : 0) << " "
                      << "overall_quality: " << (value->overall_quality) << " "
                      << "select_index: " << (value->select_index) << " "
                      << "snap box:(" << value->box.x1 << ", "
                      << value->box.y1 << ", "
                      << value->box.x2 << ", "
                      << value->box.y2 << ")";
          }
        }
      }
    }

   private:
    std::string save_dir_ = "./";
    std::string log_;
    std::string simple_log_;
  };

 protected:
  void SetUp() override {}
  void TearDown() override {}
};

class SnapShotRunningParam : public HobotXRoc::InputParam {
 public:
  SnapShotRunningParam(const std::string &method_name,
      const std::string &json_config) : InputParam(method_name) {
    content_ = json_config;
    is_json_format_ = true;
  }
  std::string Format() override { return content_; };
 private:
  std::string content_ = "";
};

class SnapShotMethodParam : public HobotXRoc::InputParam {
 public:
  SnapShotMethodParam(const std::string &method_name,
      const std::string &json_config) : InputParam(method_name) {
    content_ = json_config;
    is_json_format_ = true;
  }
  std::string Format() override { return content_; };
 private:
  std::string content_ = "";
};

TEST_F(XRocSelectMethodTest, Basic) {
  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/snapshot.json");
  flow->Init();
  auto version = flow->GetVersion("snapshot_example");
  EXPECT_EQ(version, "0.0.34");
  delete flow;
}

TEST_F(XRocSelectMethodTest, ConfigUpdate) {
  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/snapshot.json");
  flow->Init();
  auto version = flow->GetVersion("snapshot_example");
  EXPECT_EQ(version, "0.0.34");
  auto ret = flow->GetConfig("snapshot_example");
  EXPECT_EQ(ret->is_json_format_, true);
  auto config_content = ret->Format();
  auto config_content_gt = "{"
                           "\n\t\"avg_crop_num_per_frame\" : 2,"
                           "\n\t\"begin_post_frame_thr\" : 1,"
                           "\n\t\"max_crop_num_per_frame\" : 4,"
                           "\n\t\"max_tracks\" : 256,"
                           "\n\t\"need_resize\" : false,"
                           "\n\t\"out_date_target_post_flag\" : false,"
                           "\n\t\"output_height\" : 128,"
                           "\n\t\"output_width\" : 128,"
                           "\n\t\"repeat_post_flag\" : false,"
                           "\n\t\"report_flushed_track_flag\" : true,"
                           "\n\t\"resnap_value\" : 0,"
                           "\n\t\"save_original_image_frame\" : false,"
                           "\n\t\"scale_rate\" : 1.6000000000000001,"
                           "\n\t\"smoothing_frame_range\" : 10,"
                           "\n\t\"snaps_per_track\" : 1,"
                           "\n\t\"snapshot_state_enable\" : true,"
                           "\n\t\"snapshot_type\" : \"first_num_best\","
                           "\n\t\"update_steps\" : 50"
                           "\n}\n";
  EXPECT_EQ(config_content, config_content_gt);

  HobotXRoc::InputParamPtr method_param(
      new SnapShotMethodParam("snapshot_example",
                              "{\n"
                              "  \"snapshot_type\": \"first_num_best\",\n"
                              "  \"scale_rate\": 1.0,\n"
                              "  \"need_resize\": true,\n"
                              "  \"out_date_target_post_flag\" : true,\n"
                              "  \"output_width\" : 200,\n"
                              "  \"output_height\" : 200,\n"
                              "  \"repeat_post_flag\" : true,\n"
                              "  \"report_flushed_track_flag\" : false,\n"
                              "  \"update_steps\": 10,\n"
                              "  \"snaps_per_track\": 3,\n"
                              "  \"max_tracks\": 512,\n"
                              "  \"max_crop_num_per_frame\": 8,\n"
                              "  \"smoothing_frame_range\": 21,\n"
                              "  \"avg_crop_num_per_frame\": 4,\n"
                              "  \"begin_post_frame_thr\": 500,\n"
                              "  \"resnap_value\": 600,\n"
                              "  \"snapshot_state_enable\" : true,\n"
                              "  \"save_original_image_frame\": true\n"
                              "}"));
  flow->UpdateConfig("snapshot_example", method_param);

  ret = flow->GetConfig("snapshot_example");
  EXPECT_EQ(ret->is_json_format_, true);
  config_content = ret->Format();
  config_content_gt = "{"
                      "\n\t\"avg_crop_num_per_frame\" : 4,"
                      "\n\t\"begin_post_frame_thr\" : 500,"
                      "\n\t\"max_crop_num_per_frame\" : 8,"
                      "\n\t\"max_tracks\" : 512,"
                      "\n\t\"need_resize\" : true,"
                      "\n\t\"out_date_target_post_flag\" : true,"
                      "\n\t\"output_height\" : 200,"
                      "\n\t\"output_width\" : 200,"
                      "\n\t\"repeat_post_flag\" : true,"
                      "\n\t\"report_flushed_track_flag\" : false,"
                      "\n\t\"resnap_value\" : 600,"
                      "\n\t\"save_original_image_frame\" : true,"
                      "\n\t\"scale_rate\" : 1.0,"
                      "\n\t\"smoothing_frame_range\" : 21,"
                      "\n\t\"snaps_per_track\" : 3,"
                      "\n\t\"snapshot_state_enable\" : true,"
                      "\n\t\"snapshot_type\" : \"first_num_best\","
                      "\n\t\"update_steps\" : 10"
                      "\n}\n";
  EXPECT_EQ(config_content, config_content_gt);
  delete flow;
}

TEST_F(XRocSelectMethodTest, CropGray) {
  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  callback.SetImgSaveDir("./gray_test_snaps/");
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/snapshot.json");
  flow->Init();

  HobotXRoc::InputParamPtr method_param(
      new SnapShotMethodParam("snapshot_example",
                              "{\n"
                              "  \"snapshot_type\": \"first_num_best\",\n"
                              "  \"scale_rate\": 1.6,\n"
                              "  \"need_resize\": false,\n"
                              "  \"out_date_target_post_flag\" : true,\n"
                              "  \"output_width\" : 128,\n"
                              "  \"output_height\" : 128,\n"
                              "  \"repeat_post_flag\" : false,\n"
                              "  \"update_steps\": 50,\n"
                              "  \"snaps_per_track\": 1,\n"
                              "  \"max_tracks\": 256,\n"
                              "  \"max_crop_num_per_frame\": 4,\n"
                              "  \"smoothing_frame_range\": 10,\n"
                              "  \"avg_crop_num_per_frame\": 2,\n"
                              "  \"begin_post_frame_thr\": 1,\n"
                              "  \"resnap_value\": 0,\n"
                              "  \"save_original_image_frame\": true,\n"
                              "  \"snapshot_state_enable\" : true,\n"
                              "  \"report_flushed_track_flag\" : true\n"
                              "}"));
  flow->UpdateConfig("snapshot_example", method_param);
  std::string img_format = "gray";
  auto img_path = "../../test/files/video-00000001";
  auto smart_frame = "0 20000000 "
                     "30 0 1594 496 1671 604 -1070 -13 68 -1 -136 64975 6 0 9 5 0 "
                     "31 0 620 236 705 338 1180 4 36 -3 -300 1115 5 13 13 10 9 "
                     "32 0 295 231 371 325 1930 7 -6 7 -153 1386 3 7 11 7 9 "
                     "33 0 1028 131 1066 176 0 0 0 0 0 65136 0 0 0 0 0 "
                     "34 0 507 115 533 156 0 0 0 0 0 65136 0 0 0 0 0 "
                     "35 0 763 57 787 91 0 0 0 0 0 65136 0 0 0 0 0 "
                     "30 1 1554 446 1672 603 0 0 0 0 0 0 0 0 0 0 0 "
                     "36 1 733 96 774 142 0 0 0 0 0 0 0 0 0 0 0 "
                     "31 1 601 219 712 352 0 0 0 0 0 0 0 0 0 0 0 "
                     "37 1 384 104 419 142 0 0 0 0 0 0 0 0 0 0 0 "
                     "34 1 505 103 555 158 0 0 0 0 0 0 0 0 0 0 0 "
                     "32 1 286 214 387 335 0 0 0 0 0 0 0 0 0 0 0 "
                     "33 1 1026 118 1069 175 0 0 0 0 0 0 0 0 0 0 0 "
                     "38 1 1760 584 1890 728 0 0 0 0 0 0 0 0 0 0 0 "
                     "39 1 1162 243 1245 338 0 0 0 0 0 0 0 0 0 0 0 "
                     "40 1 575 85 610 125 0 0 0 0 0 0 0 0 0 0 0 "
                     "41 1 651 98 692 148 0 0 0 0 0 0 0 0 0 0 0 "
                     "42 1 201 83 231 116 0 0 0 0 0 0 0 0 0 0 0 "
                     "43 1 1523 245 1614 353 0 0 0 0 0 0 0 0 0 0 0 "
                     "44 1 224 160 259 198 0 0 0 0 0 0 0 0 0 0 0 "
                     "45 1 599 106 639 155 0 0 0 0 0 0 0 0 0 0 0 "
                     "46 1 978 124 1026 182 0 0 0 0 0 0 0 0 0 0 0 "
                     "35 1 764 49 800 92 0 0 0 0 0 0 0 0 0 0 0 "
                     "47 1 984 0 1012 32 0 0 0 0 0 0 0 0 0 0 0 "
                     "48 1 1253 237 1332 327 0 0 0 0 0 0 0 0 0 0 0 "
                     "49 1 483 83 511 116 0 0 0 0 0 0 0 0 0 0 0 "
                     "50 1 425 108 473 170 0 0 0 0 0 0 0 0 0 0 0 "
                     "51 1 821 293 917 406 0 0 0 0 0 0 0 0 0 0 0 "
                     "52 1 178 119 240 193 0 0 0 0 0 0 0 0 0 0 0";
  HobotXRoc::InputDataPtr select_inputdata(new HobotXRoc::InputData());
  int ret = ConstructInput(smart_frame, img_path, select_inputdata, img_format, true);
  EXPECT_EQ(ret, 0);
  HobotXRoc::InputDataPtr crop_inputdata(new HobotXRoc::InputData());
  img_path = "../../test/files/video-00000002";
  smart_frame = "1 20040000 "
                "30 0 1591 496 1671 607 -1050 -15 67 0 -114 64980 8 0 9 3 0 "
                "53 0 147 162 179 204 0 0 0 0 0 65136 0 0 0 0 0 "
                "33 0 1029 130 1063 176 0 0 0 0 0 65136 0 0 0 0 0 "
                "50 0 425 127 454 168 0 0 0 0 0 65136 0 0 0 0 0 "
                "34 0 504 116 531 159 0 0 0 0 0 65136 0 0 0 0 0 "
                "35 0 759 59 782 92 0 0 0 0 0 65136 0 0 0 0 0 "
                "31 0 628 234 711 336 1320 6 32 -3 -254 1124 6 9 10 8 8 "
                "32 0 296 234 378 333 1910 9 -2 7 -11 1287 6 6 11 8 10 ";
  ret = ConstructInput(smart_frame, img_path, crop_inputdata, img_format, false);
  HobotXRoc::InputParamPtr input_param(
      new SnapShotRunningParam("snapshot_example",
                               "{\n"
                               "  \"snapshot_type\": \"crop\"\n"
                               "}"));
  crop_inputdata->params_.push_back(input_param);
  EXPECT_EQ(ret, 0);
  if (ret == 0) {
    // sync SnapShot
    auto out = flow->SyncPredict(select_inputdata);
    callback.OnCallback(out);
    out = flow->SyncPredict(crop_inputdata);
    callback.OnCallback(out);
  }

  auto log = callback.GetSimpleLog();
  auto gt = "track_id: 31, frame_id: 0\n"
            "track_id: 32, frame_id: 0\n"
            "track_id: 30, frame_id: 1\n"
            "track_id: 53, frame_id: 1\n"
            "track_id: 33, frame_id: 1\n"
            "track_id: 50, frame_id: 1\n"
            "track_id: 34, frame_id: 1\n"
            "track_id: 35, frame_id: 1\n"
            "track_id: 31, frame_id: 1\n"
            "track_id: 32, frame_id: 1\n";
  EXPECT_EQ(log, gt);
  delete flow;
}

TEST_F(XRocSelectMethodTest, CropBGR) {
  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  callback.SetImgSaveDir("./bgr_test_snaps/");
  flow->SetCallback(std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/snapshot.json");
  flow->Init();

  HobotXRoc::InputParamPtr method_param(
      new SnapShotMethodParam("snapshot_example",
                              "{\n"
                              "  \"snapshot_type\": \"first_num_best\",\n"
                              "  \"scale_rate\": 1.6,\n"
                              "  \"need_resize\": false,\n"
                              "  \"out_date_target_post_flag\" : true,\n"
                              "  \"output_width\" : 128,\n"
                              "  \"output_height\" : 128,\n"
                              "  \"repeat_post_flag\" : false,\n"
                              "  \"update_steps\": 50,\n"
                              "  \"snaps_per_track\": 1,\n"
                              "  \"max_tracks\": 256,\n"
                              "  \"max_crop_num_per_frame\": 4,\n"
                              "  \"smoothing_frame_range\": 10,\n"
                              "  \"avg_crop_num_per_frame\": 2,\n"
                              "  \"begin_post_frame_thr\": 1,\n"
                              "  \"resnap_value\": 0,\n"
                              "  \"save_original_image_frame\": false,\n"
                              "  \"snapshot_state_enable\" : true,\n"
                              "  \"report_flushed_track_flag\" : true\n"
                              "}"));
  flow->UpdateConfig("snapshot_example", method_param);
  std::string img_format = "raw_bgr";
  auto img_path = "../../test/files/video-00000001.png";
  auto smart_frame = "0 20000000 "
                     "30 0 1594 496 1671 604 -1070 -13 68 -1 -136 64975 6 0 9 5 0 "
                     "31 0 620 236 705 338 1180 4 36 -3 -300 1115 5 13 13 10 9 "
                     "32 0 295 231 371 325 1930 7 -6 7 -153 1386 3 7 11 7 9 "
                     "33 0 1028 131 1066 176 0 0 0 0 0 65136 0 0 0 0 0 "
                     "34 0 507 115 533 156 0 0 0 0 0 65136 0 0 0 0 0 "
                     "35 0 763 57 787 91 0 0 0 0 0 65136 0 0 0 0 0 "
                     "30 1 1554 446 1672 603 0 0 0 0 0 0 0 0 0 0 0 "
                     "36 1 733 96 774 142 0 0 0 0 0 0 0 0 0 0 0 "
                     "31 1 601 219 712 352 0 0 0 0 0 0 0 0 0 0 0 "
                     "37 1 384 104 419 142 0 0 0 0 0 0 0 0 0 0 0 "
                     "34 1 505 103 555 158 0 0 0 0 0 0 0 0 0 0 0 "
                     "32 1 286 214 387 335 0 0 0 0 0 0 0 0 0 0 0 "
                     "33 1 1026 118 1069 175 0 0 0 0 0 0 0 0 0 0 0 "
                     "38 1 1760 584 1890 728 0 0 0 0 0 0 0 0 0 0 0 "
                     "39 1 1162 243 1245 338 0 0 0 0 0 0 0 0 0 0 0 "
                     "40 1 575 85 610 125 0 0 0 0 0 0 0 0 0 0 0 "
                     "41 1 651 98 692 148 0 0 0 0 0 0 0 0 0 0 0 "
                     "42 1 201 83 231 116 0 0 0 0 0 0 0 0 0 0 0 "
                     "43 1 1523 245 1614 353 0 0 0 0 0 0 0 0 0 0 0 "
                     "44 1 224 160 259 198 0 0 0 0 0 0 0 0 0 0 0 "
                     "45 1 599 106 639 155 0 0 0 0 0 0 0 0 0 0 0 "
                     "46 1 978 124 1026 182 0 0 0 0 0 0 0 0 0 0 0 "
                     "35 1 764 49 800 92 0 0 0 0 0 0 0 0 0 0 0 "
                     "47 1 984 0 1012 32 0 0 0 0 0 0 0 0 0 0 0 "
                     "48 1 1253 237 1332 327 0 0 0 0 0 0 0 0 0 0 0 "
                     "49 1 483 83 511 116 0 0 0 0 0 0 0 0 0 0 0 "
                     "50 1 425 108 473 170 0 0 0 0 0 0 0 0 0 0 0 "
                     "51 1 821 293 917 406 0 0 0 0 0 0 0 0 0 0 0 "
                     "52 1 178 119 240 193 0 0 0 0 0 0 0 0 0 0 0";
  HobotXRoc::InputDataPtr select_inputdata(new HobotXRoc::InputData());
  int ret = ConstructInput(smart_frame, img_path, select_inputdata, img_format, true);
  EXPECT_EQ(ret, 0);
  HobotXRoc::InputDataPtr crop_inputdata(new HobotXRoc::InputData());
  img_path = "../../test/files/video-00000002.png";
  smart_frame = "1 20040000 "
                "30 0 1591 496 1671 607 -1050 -15 67 0 -114 64980 8 0 9 3 0 "
                "53 0 147 162 179 204 0 0 0 0 0 65136 0 0 0 0 0 "
                "33 0 1029 130 1063 176 0 0 0 0 0 65136 0 0 0 0 0 "
                "50 0 425 127 454 168 0 0 0 0 0 65136 0 0 0 0 0 "
                "34 0 504 116 531 159 0 0 0 0 0 65136 0 0 0 0 0 "
                "35 0 759 59 782 92 0 0 0 0 0 65136 0 0 0 0 0 "
                "31 0 628 234 711 336 1320 6 32 -3 -254 1124 6 9 10 8 8 "
                "32 0 296 234 378 333 1910 9 -2 7 -11 1287 6 6 11 8 10 ";
  ret = ConstructInput(smart_frame, img_path, crop_inputdata, img_format, false);
  HobotXRoc::InputParamPtr input_param(
      new SnapShotRunningParam("snapshot_example",
                               "{\n"
                               "  \"snapshot_type\": \"crop\"\n"
                               "}"));
  crop_inputdata->params_.push_back(input_param);
  EXPECT_EQ(ret, 0);
  if (ret == 0) {
    // sync SnapShot
    auto out = flow->SyncPredict(select_inputdata);
    callback.OnCallback(out);
    out = flow->SyncPredict(crop_inputdata);
    callback.OnCallback(out);
  }

  auto log = callback.GetSimpleLog();
  auto gt = "track_id: 31, frame_id: 0\n"
            "track_id: 32, frame_id: 0\n"
            "track_id: 30, frame_id: 1\n"
            "track_id: 53, frame_id: 1\n"
            "track_id: 33, frame_id: 1\n"
            "track_id: 50, frame_id: 1\n"
            "track_id: 34, frame_id: 1\n"
            "track_id: 35, frame_id: 1\n"
            "track_id: 31, frame_id: 1\n"
            "track_id: 32, frame_id: 1\n";
  EXPECT_EQ(log, gt);
  delete flow;
}

TEST_F(XRocSelectMethodTest, CropRGB) {
  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  callback.SetImgSaveDir("./rgb_test_snaps/");
  flow->SetCallback(std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/snapshot.json");
  flow->Init();

  HobotXRoc::InputParamPtr method_param(
      new SnapShotMethodParam("snapshot_example",
                              "{\n"
                              "  \"snapshot_type\": \"first_num_best\",\n"
                              "  \"need_resize\": false,\n"
                              "  \"out_date_target_post_flag\" : true,\n"
                              "  \"scale_rate\": 1.6,\n"
                              "  \"output_width\" : 128,\n"
                              "  \"output_height\" : 128,\n"
                              "  \"repeat_post_flag\" : false,\n"
                              "  \"update_steps\": 50,\n"
                              "  \"snaps_per_track\": 1,\n"
                              "  \"max_tracks\": 256,\n"
                              "  \"max_crop_num_per_frame\": 4,\n"
                              "  \"smoothing_frame_range\": 10,\n"
                              "  \"avg_crop_num_per_frame\": 2,\n"
                              "  \"begin_post_frame_thr\": 1,\n"
                              "  \"resnap_value\": 0,\n"
                              "  \"save_original_image_frame\": false,\n"
                              "  \"snapshot_state_enable\" : true,\n"
                              "  \"report_flushed_track_flag\" : true\n"
                              "}"));
  flow->UpdateConfig("snapshot_example", method_param);
  std::string img_format = "raw_rgb";
  auto img_path = "../../test/files/video-00000001.png";
  auto smart_frame = "0 20000000 "
                     "30 0 1594 496 1671 604 -1070 -13 68 -1 -136 64975 6 0 9 5 0 "
                     "31 0 620 236 705 338 1180 4 36 -3 -300 1115 5 13 13 10 9 "
                     "32 0 295 231 371 325 1930 7 -6 7 -153 1386 3 7 11 7 9 "
                     "33 0 1028 131 1066 176 0 0 0 0 0 65136 0 0 0 0 0 "
                     "34 0 507 115 533 156 0 0 0 0 0 65136 0 0 0 0 0 "
                     "35 0 763 57 787 91 0 0 0 0 0 65136 0 0 0 0 0 "
                     "30 1 1554 446 1672 603 0 0 0 0 0 0 0 0 0 0 0 "
                     "36 1 733 96 774 142 0 0 0 0 0 0 0 0 0 0 0 "
                     "31 1 601 219 712 352 0 0 0 0 0 0 0 0 0 0 0 "
                     "37 1 384 104 419 142 0 0 0 0 0 0 0 0 0 0 0 "
                     "34 1 505 103 555 158 0 0 0 0 0 0 0 0 0 0 0 "
                     "32 1 286 214 387 335 0 0 0 0 0 0 0 0 0 0 0 "
                     "33 1 1026 118 1069 175 0 0 0 0 0 0 0 0 0 0 0 "
                     "38 1 1760 584 1890 728 0 0 0 0 0 0 0 0 0 0 0 "
                     "39 1 1162 243 1245 338 0 0 0 0 0 0 0 0 0 0 0 "
                     "40 1 575 85 610 125 0 0 0 0 0 0 0 0 0 0 0 "
                     "41 1 651 98 692 148 0 0 0 0 0 0 0 0 0 0 0 "
                     "42 1 201 83 231 116 0 0 0 0 0 0 0 0 0 0 0 "
                     "43 1 1523 245 1614 353 0 0 0 0 0 0 0 0 0 0 0 "
                     "44 1 224 160 259 198 0 0 0 0 0 0 0 0 0 0 0 "
                     "45 1 599 106 639 155 0 0 0 0 0 0 0 0 0 0 0 "
                     "46 1 978 124 1026 182 0 0 0 0 0 0 0 0 0 0 0 "
                     "35 1 764 49 800 92 0 0 0 0 0 0 0 0 0 0 0 "
                     "47 1 984 0 1012 32 0 0 0 0 0 0 0 0 0 0 0 "
                     "48 1 1253 237 1332 327 0 0 0 0 0 0 0 0 0 0 0 "
                     "49 1 483 83 511 116 0 0 0 0 0 0 0 0 0 0 0 "
                     "50 1 425 108 473 170 0 0 0 0 0 0 0 0 0 0 0 "
                     "51 1 821 293 917 406 0 0 0 0 0 0 0 0 0 0 0 "
                     "52 1 178 119 240 193 0 0 0 0 0 0 0 0 0 0 0";
  HobotXRoc::InputDataPtr select_inputdata(new HobotXRoc::InputData());
  int ret = ConstructInput(smart_frame, img_path, select_inputdata, img_format, true);
  EXPECT_EQ(ret, 0);
  HobotXRoc::InputDataPtr crop_inputdata(new HobotXRoc::InputData());
  img_path = "../../test/files/video-00000002.png";
  smart_frame = "1 20040000 "
                "30 0 1591 496 1671 607 -1050 -15 67 0 -114 64980 8 0 9 3 0 "
                "53 0 147 162 179 204 0 0 0 0 0 65136 0 0 0 0 0 "
                "33 0 1029 130 1063 176 0 0 0 0 0 65136 0 0 0 0 0 "
                "50 0 425 127 454 168 0 0 0 0 0 65136 0 0 0 0 0 "
                "34 0 504 116 531 159 0 0 0 0 0 65136 0 0 0 0 0 "
                "35 0 759 59 782 92 0 0 0 0 0 65136 0 0 0 0 0 "
                "31 0 628 234 711 336 1320 6 32 -3 -254 1124 6 9 10 8 8 "
                "32 0 296 234 378 333 1910 9 -2 7 -11 1287 6 6 11 8 10 ";
  ret = ConstructInput(smart_frame, img_path, crop_inputdata, img_format, false);
  HobotXRoc::InputParamPtr input_param(
      new SnapShotRunningParam("snapshot_example",
                               "{\n"
                               "  \"snapshot_type\": \"crop\"\n"
                               "}"));
  crop_inputdata->params_.push_back(input_param);
  EXPECT_EQ(ret, 0);
  if (ret == 0) {
    // sync SnapShot
    auto out = flow->SyncPredict(select_inputdata);
    callback.OnCallback(out);
    out = flow->SyncPredict(crop_inputdata);
    callback.OnCallback(out);
  }

  auto log = callback.GetSimpleLog();
  auto gt = "track_id: 31, frame_id: 0\n"
            "track_id: 32, frame_id: 0\n"
            "track_id: 30, frame_id: 1\n"
            "track_id: 53, frame_id: 1\n"
            "track_id: 33, frame_id: 1\n"
            "track_id: 50, frame_id: 1\n"
            "track_id: 34, frame_id: 1\n"
            "track_id: 35, frame_id: 1\n"
            "track_id: 31, frame_id: 1\n"
            "track_id: 32, frame_id: 1\n";
  EXPECT_EQ(log, gt);
  delete flow;
}

TEST_F(XRocSelectMethodTest, CropI420) {
  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  callback.SetImgSaveDir("./i420_test_snaps/");
  flow->SetCallback(std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/snapshot.json");
  flow->Init();

  HobotXRoc::InputParamPtr method_param(
      new SnapShotMethodParam("snapshot_example",
                              "{\n"
                              "  \"snapshot_type\": \"first_num_best\",\n"
                              "  \"scale_rate\": 1.6,\n"
                              "  \"need_resize\": false,\n"
                              "  \"out_date_target_post_flag\" : true,\n"
                              "  \"output_width\" : 128,\n"
                              "  \"output_height\" : 128,\n"
                              "  \"repeat_post_flag\" : false,\n"
                              "  \"update_steps\": 50,\n"
                              "  \"snaps_per_track\": 1,\n"
                              "  \"max_tracks\": 256,\n"
                              "  \"max_crop_num_per_frame\": 4,\n"
                              "  \"smoothing_frame_range\": 10,\n"
                              "  \"avg_crop_num_per_frame\": 2,\n"
                              "  \"begin_post_frame_thr\": 1,\n"
                              "  \"resnap_value\": 0,\n"
                              "  \"save_original_image_frame\": false,\n"
                              "  \"snapshot_state_enable\" : true,\n"
                              "  \"report_flushed_track_flag\" : true\n"
                              "}"));
  flow->UpdateConfig("snapshot_example", method_param);
  std::string img_format = "yuv_i420";
  auto img_path = "../../test/files/video-00000001.png";
  auto smart_frame = "0 20000000 "
                     "30 0 1594 496 1671 604 -1070 -13 68 -1 -136 64975 6 0 9 5 0 "
                     "31 0 620 236 705 338 1180 4 36 -3 -300 1115 5 13 13 10 9 "
                     "32 0 295 231 371 325 1930 7 -6 7 -153 1386 3 7 11 7 9 "
                     "33 0 1028 131 1066 176 0 0 0 0 0 65136 0 0 0 0 0 "
                     "34 0 507 115 533 156 0 0 0 0 0 65136 0 0 0 0 0 "
                     "35 0 763 57 787 91 0 0 0 0 0 65136 0 0 0 0 0 "
                     "30 1 1554 446 1672 603 0 0 0 0 0 0 0 0 0 0 0 "
                     "36 1 733 96 774 142 0 0 0 0 0 0 0 0 0 0 0 "
                     "31 1 601 219 712 352 0 0 0 0 0 0 0 0 0 0 0 "
                     "37 1 384 104 419 142 0 0 0 0 0 0 0 0 0 0 0 "
                     "34 1 505 103 555 158 0 0 0 0 0 0 0 0 0 0 0 "
                     "32 1 286 214 387 335 0 0 0 0 0 0 0 0 0 0 0 "
                     "33 1 1026 118 1069 175 0 0 0 0 0 0 0 0 0 0 0 "
                     "38 1 1760 584 1890 728 0 0 0 0 0 0 0 0 0 0 0 "
                     "39 1 1162 243 1245 338 0 0 0 0 0 0 0 0 0 0 0 "
                     "40 1 575 85 610 125 0 0 0 0 0 0 0 0 0 0 0 "
                     "41 1 651 98 692 148 0 0 0 0 0 0 0 0 0 0 0 "
                     "42 1 201 83 231 116 0 0 0 0 0 0 0 0 0 0 0 "
                     "43 1 1523 245 1614 353 0 0 0 0 0 0 0 0 0 0 0 "
                     "44 1 224 160 259 198 0 0 0 0 0 0 0 0 0 0 0 "
                     "45 1 599 106 639 155 0 0 0 0 0 0 0 0 0 0 0 "
                     "46 1 978 124 1026 182 0 0 0 0 0 0 0 0 0 0 0 "
                     "35 1 764 49 800 92 0 0 0 0 0 0 0 0 0 0 0 "
                     "47 1 984 0 1012 32 0 0 0 0 0 0 0 0 0 0 0 "
                     "48 1 1253 237 1332 327 0 0 0 0 0 0 0 0 0 0 0 "
                     "49 1 483 83 511 116 0 0 0 0 0 0 0 0 0 0 0 "
                     "50 1 425 108 473 170 0 0 0 0 0 0 0 0 0 0 0 "
                     "51 1 821 293 917 406 0 0 0 0 0 0 0 0 0 0 0 "
                     "52 1 178 119 240 193 0 0 0 0 0 0 0 0 0 0 0";
  HobotXRoc::InputDataPtr select_inputdata(new HobotXRoc::InputData());
  int ret = ConstructInput(smart_frame, img_path, select_inputdata, img_format, true);
  EXPECT_EQ(ret, 0);
  HobotXRoc::InputDataPtr crop_inputdata(new HobotXRoc::InputData());
  img_path = "../../test/files/video-00000002.png";
  smart_frame = "1 20040000 "
                "30 0 1591 496 1671 607 -1050 -15 67 0 -114 64980 8 0 9 3 0 "
                "53 0 147 162 179 204 0 0 0 0 0 65136 0 0 0 0 0 "
                "33 0 1029 130 1063 176 0 0 0 0 0 65136 0 0 0 0 0 "
                "50 0 425 127 454 168 0 0 0 0 0 65136 0 0 0 0 0 "
                "34 0 504 116 531 159 0 0 0 0 0 65136 0 0 0 0 0 "
                "35 0 759 59 782 92 0 0 0 0 0 65136 0 0 0 0 0 "
                "31 0 628 234 711 336 1320 6 32 -3 -254 1124 6 9 10 8 8 "
                "32 0 296 234 378 333 1910 9 -2 7 -11 1287 6 6 11 8 10 ";
  ret = ConstructInput(smart_frame, img_path, crop_inputdata, img_format, false);
  HobotXRoc::InputParamPtr input_param(
      new SnapShotRunningParam("snapshot_example",
                               "{\n"
                               "  \"snapshot_type\": \"crop\"\n"
                               "}"));
  crop_inputdata->params_.push_back(input_param);
  EXPECT_EQ(ret, 0);
  if (ret == 0) {
    // sync SnapShot
    auto out = flow->SyncPredict(select_inputdata);
    callback.OnCallback(out);
    out = flow->SyncPredict(crop_inputdata);
    callback.OnCallback(out);
  }
  auto log = callback.GetSimpleLog();
  auto gt = "track_id: 31, frame_id: 0\n"
            "track_id: 32, frame_id: 0\n"
            "track_id: 30, frame_id: 1\n"
            "track_id: 53, frame_id: 1\n"
            "track_id: 33, frame_id: 1\n"
            "track_id: 50, frame_id: 1\n"
            "track_id: 34, frame_id: 1\n"
            "track_id: 35, frame_id: 1\n"
            "track_id: 31, frame_id: 1\n"
            "track_id: 32, frame_id: 1\n";
  EXPECT_EQ(log, gt);
  delete flow;
}

TEST_F(XRocSelectMethodTest, VerificationConfigFile) {
  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/snapshot.json");
  flow->Init();
  HobotXRoc::InputParamPtr method_param(
      new SnapShotMethodParam("snapshot_example",
                              "{\n"
                              "  \"snapshot_type\": \"first_num_best\",\n"
                              "  \"scale_rate\": 1.6,\n"
                              "  \"need_resize\": false,\n"
                              "  \"output_width\" : 128,\n"
                              "  \"output_height\" : 128,\n"
                              "  \"repeat_post_flag\" : false,\n"
                              "  \"out_date_target_post_flag\" : true,\n"
                              "  \"update_steps\": 50,\n"
                              "  \"snaps_per_track\": 1,\n"
                              "  \"max_tracks\": 256,\n"
                              "  \"max_crop_num_per_frame\": 4,\n"
                              "  \"smoothing_frame_range\": 10,\n"
                              "  \"avg_crop_num_per_frame\": 2,\n"
                              "  \"begin_post_frame_thr\": 500,\n"
                              "  \"resnap_value\": 6,\n"
                              "  \"save_original_image_frame\": false,\n"
                              "  \"snapshot_state_enable\" : true,\n"
                              "  \"report_flushed_track_flag\" : true\n"
                              "}"));
  flow->UpdateConfig("snapshot_example", method_param);
  std::string algo_res = "../../test/files/logdh5cdump.log";
  std::ifstream fin(algo_res.data(), std::ios::binary);
  if (fin.fail()) {
    std::cout << "Open track_result failed" << std::endl;
  }

  using std::chrono::high_resolution_clock;
  using std::chrono::duration;

  int count = 0;
  std::string smart_frame;
  while (getline(fin, smart_frame)) {
    HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
    auto start_time = high_resolution_clock::now();
    int ret = ConstructInput(smart_frame, "", inputdata, "", true);

    auto construct_end_time = high_resolution_clock::now();
    duration<double, std::milli> proc_cost = construct_end_time - start_time;
    LOGI << "construct cost(ms):" << proc_cost.count();
    if (ret == 0) {
      // sync SnapShot
      auto out = flow->SyncPredict(inputdata);
      auto predict_end_time = high_resolution_clock::now();
      proc_cost = predict_end_time - construct_end_time;
      LOGI << "predict cost(ms):" << proc_cost.count();
      callback.OnCallback(out);
      auto callback_end_time = high_resolution_clock::now();
      proc_cost = callback_end_time - predict_end_time;
      LOGI << "callback cost(ms):" << proc_cost.count();
    }
    auto end_time = high_resolution_clock::now();
    proc_cost = end_time - start_time;
    LOGI << "Process Frame cost(ms):" << proc_cost.count();
    count++;
  }
  auto log = callback.GetSimpleLog();
  auto gt = "track_id: 31, frame_id: 67\ntrack_id: 34, frame_id: 100\n"
            "track_id: 32, frame_id: 69\ntrack_id: 78, frame_id: 134\n"
            "track_id: 95, frame_id: 222\ntrack_id: 126, frame_id: 237\n"
            "track_id: 144, frame_id: 245\ntrack_id: 130, frame_id: 275\n"
            "track_id: 167, frame_id: 304\ntrack_id: 54, frame_id: 285\n"
            "track_id: 162, frame_id: 332\ntrack_id: 50, frame_id: 269\n"
            "track_id: 197, frame_id: 385\ntrack_id: 181, frame_id: 369\n"
            "track_id: 115, frame_id: 422\ntrack_id: 133, frame_id: 293\n"
            "track_id: 53, frame_id: 471\ntrack_id: 152, frame_id: 387\n"
            "track_id: 211, frame_id: 453\ntrack_id: 219, frame_id: 482\n"
            "track_id: 235, frame_id: 541\ntrack_id: 100, frame_id: 543\n"
            "track_id: 109, frame_id: 569\ntrack_id: 234, frame_id: 616\n"
            "track_id: 1, frame_id: 616\ntrack_id: 245, frame_id: 583\n"
            "track_id: 189, frame_id: 772\ntrack_id: 259, frame_id: 805\n"
            "track_id: 257, frame_id: 912\ntrack_id: 4, frame_id: 940\n"
            "track_id: 157, frame_id: 1129\ntrack_id: 266, frame_id: 1095\n"
            "track_id: 202, frame_id: 1231\ntrack_id: 289, frame_id: 1329\n"
            "track_id: 55, frame_id: 1296\ntrack_id: 172, frame_id: 1347\n"
            "track_id: 10, frame_id: 1380\ntrack_id: 207, frame_id: 1341\n"
            "track_id: 238, frame_id: 1300\ntrack_id: 88, frame_id: 1416\n"
            "track_id: 272, frame_id: 1413\ntrack_id: 113, frame_id: 1157\n"
            "track_id: 300, frame_id: 1437\ntrack_id: 103, frame_id: 1293\n"
            "track_id: 137, frame_id: 1342\ntrack_id: 138, frame_id: 1481\n"
            "track_id: 286, frame_id: 1474\ntrack_id: 330, frame_id: 1456\n"
            "track_id: 102, frame_id: 1492\ntrack_id: 158, frame_id: 1525\n"
            "track_id: 285, frame_id: 1515\ntrack_id: 119, frame_id: 1500\n"
            "track_id: 198, frame_id: 1424\ntrack_id: 301, frame_id: 1575\n"
            "track_id: 317, frame_id: 1439\ntrack_id: 146, frame_id: 1505\n"
            "track_id: 343, frame_id: 1584\ntrack_id: 220, frame_id: 1571\n"
            "track_id: 241, frame_id: 1618\ntrack_id: 174, frame_id: 1541\n"
            "track_id: 263, frame_id: 1631\ntrack_id: 153, frame_id: 1564\n"
            "track_id: 250, frame_id: 1666\ntrack_id: 345, frame_id: 1632\n"
            "track_id: 306, frame_id: 1647\ntrack_id: 68, frame_id: 1803\n"
            "track_id: 352, frame_id: 1751\ntrack_id: 327, frame_id: 1891\n"
            "track_id: 283, frame_id: 1718\ntrack_id: 381, frame_id: 1906\n"
            "track_id: 242, frame_id: 1796\ntrack_id: 384, frame_id: 1926\n"
            "track_id: 93, frame_id: 1901\ntrack_id: 147, frame_id: 1942\n"
            "track_id: 110, frame_id: 1962\ntrack_id: 187, frame_id: 1908\n"
            "track_id: 342, frame_id: 2003\ntrack_id: 246, frame_id: 1925\n"
            "track_id: 297, frame_id: 1996\ntrack_id: 142, frame_id: 2052\n"
            "track_id: 322, frame_id: 1763\ntrack_id: 344, frame_id: 2110\n"
            "track_id: 339, frame_id: 2172\ntrack_id: 176, frame_id: 2240\n"
            "track_id: 44, frame_id: 2359\ntrack_id: 236, frame_id: 2353\n"
            "track_id: 353, frame_id: 2390\ntrack_id: 80, frame_id: 2582\n"
            "track_id: 290, frame_id: 2623\ntrack_id: 323, frame_id: 2843\n"
            "track_id: 478, frame_id: 2822\ntrack_id: 94, frame_id: 2916\n"
            "track_id: 6, frame_id: 2917\ntrack_id: 483, frame_id: 2975\n"
            "track_id: 106, frame_id: 2989\ntrack_id: 151, frame_id: 3008\n"
            "track_id: 155, frame_id: 2958\ntrack_id: 410, frame_id: 2905\n"
            "track_id: 340, frame_id: 3050\ntrack_id: 262, frame_id: 3005\n"
            "track_id: 264, frame_id: 3083\ntrack_id: 424, frame_id: 3038\n"
            "track_id: 437, frame_id: 3088\ntrack_id: 299, frame_id: 3084\n"
            "track_id: 494, frame_id: 3026\ntrack_id: 479, frame_id: 3143\n"
            "track_id: 444, frame_id: 3289\ntrack_id: 179, frame_id: 3369\n"
            "track_id: 195, frame_id: 3080\ntrack_id: 374, frame_id: 3534\n"
            "track_id: 516, frame_id: 3427\ntrack_id: 282, frame_id: 3528\n"
            "track_id: 177, frame_id: 3593\ntrack_id: 541, frame_id: 3581\n"
            "track_id: 86, frame_id: 3553\ntrack_id: 513, frame_id: 3722\n"
            "track_id: 544, frame_id: 3651\ntrack_id: 433, frame_id: 3679\n"
            "track_id: 555, frame_id: 3794\ntrack_id: 477, frame_id: 3813\n"
            "track_id: 540, frame_id: 3758\ntrack_id: 492, frame_id: 3749\n"
            "track_id: 455, frame_id: 3793\ntrack_id: 510, frame_id: 3781\n"
            "track_id: 135, frame_id: 3796\ntrack_id: 175, frame_id: 3786\n"
            "track_id: 501, frame_id: 3669\ntrack_id: 482, frame_id: 3855\n"
            "track_id: 471, frame_id: 3854\ntrack_id: 394, frame_id: 3892\n"
            "track_id: 370, frame_id: 3943\ntrack_id: 558, frame_id: 3941\n"
            "track_id: 116, frame_id: 4010\ntrack_id: 87, frame_id: 4059\n"
            "track_id: 539, frame_id: 4314\ntrack_id: 528, frame_id: 4319\n"
            "track_id: 565, frame_id: 4324\ntrack_id: 303, frame_id: 4405\n"
            "track_id: 442, frame_id: 4445\ntrack_id: 553, frame_id: 4400\n"
            "track_id: 325, frame_id: 4529\ntrack_id: 628, frame_id: 4686\n"
            "track_id: 484, frame_id: 4716\ntrack_id: 579, frame_id: 4743\n"
            "track_id: 588, frame_id: 4804\ntrack_id: 459, frame_id: 4888\n"
            "track_id: 369, frame_id: 4825\ntrack_id: 647, frame_id: 4960\n"
            "track_id: 36, frame_id: 4895\ntrack_id: 643, frame_id: 4997\n"
            "track_id: 170, frame_id: 4931\ntrack_id: 426, frame_id: 4917\n"
            "track_id: 26, frame_id: 5039\ntrack_id: 422, frame_id: 4933\n"
            "track_id: 253, frame_id: 5005\ntrack_id: 641, frame_id: 5003\n"
            "track_id: 249, frame_id: 4992\ntrack_id: 448, frame_id: 4846\n"
            "track_id: 610, frame_id: 5293\ntrack_id: 592, frame_id: 5246\n"
            "track_id: 329, frame_id: 5694\ntrack_id: 277, frame_id: 5636\n"
            "track_id: 545, frame_id: 5655\n";
  EXPECT_EQ(log, gt);
  delete flow;
}

TEST_F(XRocSelectMethodTest, VerificationConfigUpdate) {
  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/snapshot_no_config_file.json");
  flow->Init();
  HobotXRoc::InputParamPtr method_param(
      new SnapShotMethodParam("snapshot_example",
                              "{\n"
                              "  \"snapshot_type\": \"first_num_best\",\n"
                              "  \"scale_rate\": 1.6,\n"
                              "  \"need_resize\": false,\n"
                              "  \"out_date_target_post_flag\" : true,\n"
                              "  \"output_width\" : 128,\n"
                              "  \"output_height\" : 128,\n"
                              "  \"repeat_post_flag\" : false,\n"
                              "  \"update_steps\": 50,\n"
                              "  \"snaps_per_track\": 1,\n"
                              "  \"max_tracks\": 256,\n"
                              "  \"max_crop_num_per_frame\": 4,\n"
                              "  \"smoothing_frame_range\": 10,\n"
                              "  \"avg_crop_num_per_frame\": 2,\n"
                              "  \"begin_post_frame_thr\": 500,\n"
                              "  \"resnap_value\": 6,\n"
                              "  \"save_original_image_frame\": false,\n"
                              "  \"snapshot_state_enable\" : true,\n"
                              "  \"report_flushed_track_flag\" : true\n"
                              "}"));
  flow->UpdateConfig("snapshot_example", method_param);
  std::string algo_res = "../../test/files/logdh5cdump.log";
  std::ifstream fin(algo_res.data(), std::ios::binary);
  if (fin.fail()) {
    std::cout << "Open track_result failed" << std::endl;
  }

  using std::chrono::high_resolution_clock;
  using std::chrono::duration;

  int count = 0;
  std::string smart_frame;
  while (getline(fin, smart_frame)) {
    HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
    auto start_time = high_resolution_clock::now();
    int ret = ConstructInput(smart_frame, "", inputdata, "", true);

    auto construct_end_time = high_resolution_clock::now();
    duration<double, std::milli> proc_cost = construct_end_time - start_time;
    LOGI << "construct cost(ms):" << proc_cost.count();
    if (ret == 0) {
      // sync SnapShot
      auto out = flow->SyncPredict(inputdata);
      auto predict_end_time = high_resolution_clock::now();
      proc_cost = predict_end_time - construct_end_time;
      LOGI << "predict cost(ms):" << proc_cost.count();
      callback.OnCallback(out);
      auto callback_end_time = high_resolution_clock::now();
      proc_cost = callback_end_time - predict_end_time;
      LOGI << "callback cost(ms):" << proc_cost.count();
    }
    auto end_time = high_resolution_clock::now();
    proc_cost = end_time - start_time;
    LOGI << "Process Frame cost(ms):" << proc_cost.count();
    count++;
  }
  auto log = callback.GetSimpleLog();
  auto gt = "track_id: 31, frame_id: 67\ntrack_id: 34, frame_id: 100\n"
            "track_id: 32, frame_id: 69\ntrack_id: 78, frame_id: 134\n"
            "track_id: 95, frame_id: 222\ntrack_id: 126, frame_id: 237\n"
            "track_id: 144, frame_id: 245\ntrack_id: 130, frame_id: 275\n"
            "track_id: 167, frame_id: 304\ntrack_id: 54, frame_id: 285\n"
            "track_id: 162, frame_id: 332\ntrack_id: 50, frame_id: 269\n"
            "track_id: 197, frame_id: 385\ntrack_id: 181, frame_id: 369\n"
            "track_id: 115, frame_id: 422\ntrack_id: 133, frame_id: 293\n"
            "track_id: 53, frame_id: 471\ntrack_id: 152, frame_id: 387\n"
            "track_id: 211, frame_id: 453\ntrack_id: 219, frame_id: 482\n"
            "track_id: 235, frame_id: 541\ntrack_id: 100, frame_id: 543\n"
            "track_id: 109, frame_id: 569\ntrack_id: 234, frame_id: 616\n"
            "track_id: 1, frame_id: 616\ntrack_id: 245, frame_id: 583\n"
            "track_id: 189, frame_id: 772\ntrack_id: 259, frame_id: 805\n"
            "track_id: 257, frame_id: 912\ntrack_id: 4, frame_id: 940\n"
            "track_id: 157, frame_id: 1129\ntrack_id: 266, frame_id: 1095\n"
            "track_id: 202, frame_id: 1231\ntrack_id: 289, frame_id: 1329\n"
            "track_id: 55, frame_id: 1296\ntrack_id: 172, frame_id: 1347\n"
            "track_id: 10, frame_id: 1380\ntrack_id: 207, frame_id: 1341\n"
            "track_id: 238, frame_id: 1300\ntrack_id: 88, frame_id: 1416\n"
            "track_id: 272, frame_id: 1413\ntrack_id: 113, frame_id: 1157\n"
            "track_id: 300, frame_id: 1437\ntrack_id: 103, frame_id: 1293\n"
            "track_id: 137, frame_id: 1342\ntrack_id: 138, frame_id: 1481\n"
            "track_id: 286, frame_id: 1474\ntrack_id: 330, frame_id: 1456\n"
            "track_id: 102, frame_id: 1492\ntrack_id: 158, frame_id: 1525\n"
            "track_id: 285, frame_id: 1515\ntrack_id: 119, frame_id: 1500\n"
            "track_id: 198, frame_id: 1424\ntrack_id: 301, frame_id: 1575\n"
            "track_id: 317, frame_id: 1439\ntrack_id: 146, frame_id: 1505\n"
            "track_id: 343, frame_id: 1584\ntrack_id: 220, frame_id: 1571\n"
            "track_id: 241, frame_id: 1618\ntrack_id: 174, frame_id: 1541\n"
            "track_id: 263, frame_id: 1631\ntrack_id: 153, frame_id: 1564\n"
            "track_id: 250, frame_id: 1666\ntrack_id: 345, frame_id: 1632\n"
            "track_id: 306, frame_id: 1647\ntrack_id: 68, frame_id: 1803\n"
            "track_id: 352, frame_id: 1751\ntrack_id: 327, frame_id: 1891\n"
            "track_id: 283, frame_id: 1718\ntrack_id: 381, frame_id: 1906\n"
            "track_id: 242, frame_id: 1796\ntrack_id: 384, frame_id: 1926\n"
            "track_id: 93, frame_id: 1901\ntrack_id: 147, frame_id: 1942\n"
            "track_id: 110, frame_id: 1962\ntrack_id: 187, frame_id: 1908\n"
            "track_id: 342, frame_id: 2003\ntrack_id: 246, frame_id: 1925\n"
            "track_id: 297, frame_id: 1996\ntrack_id: 142, frame_id: 2052\n"
            "track_id: 322, frame_id: 1763\ntrack_id: 344, frame_id: 2110\n"
            "track_id: 339, frame_id: 2172\ntrack_id: 176, frame_id: 2240\n"
            "track_id: 44, frame_id: 2359\ntrack_id: 236, frame_id: 2353\n"
            "track_id: 353, frame_id: 2390\ntrack_id: 80, frame_id: 2582\n"
            "track_id: 290, frame_id: 2623\ntrack_id: 323, frame_id: 2843\n"
            "track_id: 478, frame_id: 2822\ntrack_id: 94, frame_id: 2916\n"
            "track_id: 6, frame_id: 2917\ntrack_id: 483, frame_id: 2975\n"
            "track_id: 106, frame_id: 2989\ntrack_id: 151, frame_id: 3008\n"
            "track_id: 155, frame_id: 2958\ntrack_id: 410, frame_id: 2905\n"
            "track_id: 340, frame_id: 3050\ntrack_id: 262, frame_id: 3005\n"
            "track_id: 264, frame_id: 3083\ntrack_id: 424, frame_id: 3038\n"
            "track_id: 437, frame_id: 3088\ntrack_id: 299, frame_id: 3084\n"
            "track_id: 494, frame_id: 3026\ntrack_id: 479, frame_id: 3143\n"
            "track_id: 444, frame_id: 3289\ntrack_id: 179, frame_id: 3369\n"
            "track_id: 195, frame_id: 3080\ntrack_id: 374, frame_id: 3534\n"
            "track_id: 516, frame_id: 3427\ntrack_id: 282, frame_id: 3528\n"
            "track_id: 177, frame_id: 3593\ntrack_id: 541, frame_id: 3581\n"
            "track_id: 86, frame_id: 3553\ntrack_id: 513, frame_id: 3722\n"
            "track_id: 544, frame_id: 3651\ntrack_id: 433, frame_id: 3679\n"
            "track_id: 555, frame_id: 3794\ntrack_id: 477, frame_id: 3813\n"
            "track_id: 540, frame_id: 3758\ntrack_id: 492, frame_id: 3749\n"
            "track_id: 455, frame_id: 3793\ntrack_id: 510, frame_id: 3781\n"
            "track_id: 135, frame_id: 3796\ntrack_id: 175, frame_id: 3786\n"
            "track_id: 501, frame_id: 3669\ntrack_id: 482, frame_id: 3855\n"
            "track_id: 471, frame_id: 3854\ntrack_id: 394, frame_id: 3892\n"
            "track_id: 370, frame_id: 3943\ntrack_id: 558, frame_id: 3941\n"
            "track_id: 116, frame_id: 4010\ntrack_id: 87, frame_id: 4059\n"
            "track_id: 539, frame_id: 4314\ntrack_id: 528, frame_id: 4319\n"
            "track_id: 565, frame_id: 4324\ntrack_id: 303, frame_id: 4405\n"
            "track_id: 442, frame_id: 4445\ntrack_id: 553, frame_id: 4400\n"
            "track_id: 325, frame_id: 4529\ntrack_id: 628, frame_id: 4686\n"
            "track_id: 484, frame_id: 4716\ntrack_id: 579, frame_id: 4743\n"
            "track_id: 588, frame_id: 4804\ntrack_id: 459, frame_id: 4888\n"
            "track_id: 369, frame_id: 4825\ntrack_id: 647, frame_id: 4960\n"
            "track_id: 36, frame_id: 4895\ntrack_id: 643, frame_id: 4997\n"
            "track_id: 170, frame_id: 4931\ntrack_id: 426, frame_id: 4917\n"
            "track_id: 26, frame_id: 5039\ntrack_id: 422, frame_id: 4933\n"
            "track_id: 253, frame_id: 5005\ntrack_id: 641, frame_id: 5003\n"
            "track_id: 249, frame_id: 4992\ntrack_id: 448, frame_id: 4846\n"
            "track_id: 610, frame_id: 5293\ntrack_id: 592, frame_id: 5246\n"
            "track_id: 329, frame_id: 5694\ntrack_id: 277, frame_id: 5636\n"
            "track_id: 545, frame_id: 5655\n";
  EXPECT_EQ(log, gt);
  delete flow;
}

TEST_F(XRocSelectMethodTest, EmptyUserdata) {
  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/snapshot.json");
  flow->Init();
  HobotXRoc::InputParamPtr method_param(
      new SnapShotMethodParam("snapshot_example",
                              "{\n"
                              "  \"snapshot_type\": \"first_num_best\",\n"
                              "  \"scale_rate\": 1.6,\n"
                              "  \"need_resize\": false,\n"
                              "  \"out_date_target_post_flag\" : true,\n"
                              "  \"output_width\" : 128,\n"
                              "  \"output_height\" : 128,\n"
                              "  \"update_steps\": 50,\n"
                              "  \"snaps_per_track\": 1,\n"
                              "  \"max_tracks\": 256,\n"
                              "  \"max_crop_num_per_frame\": 4,\n"
                              "  \"smoothing_frame_range\": 10,\n"
                              "  \"avg_crop_num_per_frame\": 2,\n"
                              "  \"begin_post_frame_thr\": 500,\n"
                              "  \"resnap_value\": 6,\n"
                              "  \"save_original_image_frame\": false,\n"
                              "  \"snapshot_state_enable\" : true,\n"
                              "  \"report_flushed_track_flag\" : true\n"
                              "}"));
  flow->UpdateConfig("snapshot_example", method_param);
  std::string algo_res = "../../test/files/logdh5cdump.log";
  std::ifstream fin(algo_res.data(), std::ios::binary);
  if (fin.fail()) {
    std::cout << "Open track_result failed" << std::endl;
  }

  using std::chrono::high_resolution_clock;
  using std::chrono::duration;

  int count = 0;
  std::string smart_frame;
  while (getline(fin, smart_frame)) {
    HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
    auto start_time = high_resolution_clock::now();
    int ret = ConstructInputInvalidUserdata(smart_frame, "", inputdata, "", true);

    auto construct_end_time = high_resolution_clock::now();
    duration<double, std::milli> proc_cost = construct_end_time - start_time;
    LOGI << "construct cost(ms):" << proc_cost.count();
    if (ret == 0) {
      // sync SnapShot
      auto out = flow->SyncPredict(inputdata);
      auto predict_end_time = high_resolution_clock::now();
      proc_cost = predict_end_time - construct_end_time;
      LOGI << "predict cost(ms):" << proc_cost.count();
      callback.OnCallback(out);
      auto callback_end_time = high_resolution_clock::now();
      proc_cost = callback_end_time - predict_end_time;
      LOGI << "callback cost(ms):" << proc_cost.count();
    }
    auto end_time = high_resolution_clock::now();
    proc_cost = end_time - start_time;
    LOGI << "Process Frame cost(ms):" << proc_cost.count();
    count++;
  }
  auto log = callback.GetSimpleLog();
  auto gt = "track_id: 31, frame_id: 67\ntrack_id: 34, frame_id: 100\n"
            "track_id: 32, frame_id: 69\ntrack_id: 78, frame_id: 134\n"
            "track_id: 95, frame_id: 222\ntrack_id: 126, frame_id: 237\n"
            "track_id: 144, frame_id: 245\ntrack_id: 130, frame_id: 275\n"
            "track_id: 167, frame_id: 304\ntrack_id: 54, frame_id: 285\n"
            "track_id: 162, frame_id: 332\ntrack_id: 50, frame_id: 269\n"
            "track_id: 197, frame_id: 385\ntrack_id: 181, frame_id: 369\n"
            "track_id: 115, frame_id: 422\ntrack_id: 133, frame_id: 293\n"
            "track_id: 53, frame_id: 471\ntrack_id: 152, frame_id: 387\n"
            "track_id: 211, frame_id: 453\ntrack_id: 219, frame_id: 482\n"
            "track_id: 235, frame_id: 541\ntrack_id: 100, frame_id: 543\n"
            "track_id: 109, frame_id: 569\ntrack_id: 234, frame_id: 616\n"
            "track_id: 1, frame_id: 616\ntrack_id: 245, frame_id: 583\n"
            "track_id: 189, frame_id: 772\ntrack_id: 259, frame_id: 805\n"
            "track_id: 257, frame_id: 912\ntrack_id: 4, frame_id: 940\n"
            "track_id: 157, frame_id: 1129\ntrack_id: 266, frame_id: 1095\n"
            "track_id: 202, frame_id: 1231\ntrack_id: 289, frame_id: 1329\n"
            "track_id: 55, frame_id: 1296\ntrack_id: 172, frame_id: 1347\n"
            "track_id: 10, frame_id: 1380\ntrack_id: 207, frame_id: 1341\n"
            "track_id: 238, frame_id: 1300\ntrack_id: 88, frame_id: 1416\n"
            "track_id: 272, frame_id: 1413\ntrack_id: 113, frame_id: 1157\n"
            "track_id: 300, frame_id: 1437\ntrack_id: 103, frame_id: 1293\n"
            "track_id: 137, frame_id: 1342\ntrack_id: 138, frame_id: 1481\n"
            "track_id: 286, frame_id: 1474\ntrack_id: 330, frame_id: 1456\n"
            "track_id: 102, frame_id: 1492\ntrack_id: 158, frame_id: 1525\n"
            "track_id: 285, frame_id: 1515\ntrack_id: 119, frame_id: 1500\n"
            "track_id: 198, frame_id: 1424\ntrack_id: 301, frame_id: 1575\n"
            "track_id: 317, frame_id: 1439\ntrack_id: 146, frame_id: 1505\n"
            "track_id: 343, frame_id: 1584\ntrack_id: 220, frame_id: 1571\n"
            "track_id: 241, frame_id: 1618\ntrack_id: 174, frame_id: 1541\n"
            "track_id: 263, frame_id: 1631\ntrack_id: 153, frame_id: 1564\n"
            "track_id: 250, frame_id: 1666\ntrack_id: 345, frame_id: 1632\n"
            "track_id: 306, frame_id: 1647\ntrack_id: 68, frame_id: 1803\n"
            "track_id: 352, frame_id: 1751\ntrack_id: 327, frame_id: 1891\n"
            "track_id: 283, frame_id: 1718\ntrack_id: 381, frame_id: 1906\n"
            "track_id: 242, frame_id: 1796\ntrack_id: 384, frame_id: 1926\n"
            "track_id: 93, frame_id: 1901\ntrack_id: 147, frame_id: 1942\n"
            "track_id: 110, frame_id: 1962\ntrack_id: 187, frame_id: 1908\n"
            "track_id: 342, frame_id: 2003\ntrack_id: 246, frame_id: 1925\n"
            "track_id: 297, frame_id: 1996\ntrack_id: 142, frame_id: 2052\n"
            "track_id: 322, frame_id: 1763\ntrack_id: 344, frame_id: 2110\n"
            "track_id: 339, frame_id: 2172\ntrack_id: 176, frame_id: 2240\n"
            "track_id: 44, frame_id: 2359\ntrack_id: 236, frame_id: 2353\n"
            "track_id: 353, frame_id: 2390\ntrack_id: 80, frame_id: 2582\n"
            "track_id: 290, frame_id: 2623\ntrack_id: 323, frame_id: 2843\n"
            "track_id: 478, frame_id: 2822\ntrack_id: 94, frame_id: 2916\n"
            "track_id: 6, frame_id: 2917\ntrack_id: 483, frame_id: 2975\n"
            "track_id: 106, frame_id: 2989\ntrack_id: 151, frame_id: 3008\n"
            "track_id: 155, frame_id: 2958\ntrack_id: 410, frame_id: 2905\n"
            "track_id: 340, frame_id: 3050\ntrack_id: 262, frame_id: 3005\n"
            "track_id: 264, frame_id: 3083\ntrack_id: 424, frame_id: 3038\n"
            "track_id: 437, frame_id: 3088\ntrack_id: 299, frame_id: 3084\n"
            "track_id: 494, frame_id: 3026\ntrack_id: 479, frame_id: 3143\n"
            "track_id: 444, frame_id: 3289\ntrack_id: 179, frame_id: 3369\n"
            "track_id: 195, frame_id: 3080\ntrack_id: 374, frame_id: 3534\n"
            "track_id: 516, frame_id: 3427\ntrack_id: 282, frame_id: 3528\n"
            "track_id: 177, frame_id: 3593\ntrack_id: 541, frame_id: 3581\n"
            "track_id: 86, frame_id: 3553\ntrack_id: 513, frame_id: 3722\n"
            "track_id: 544, frame_id: 3651\ntrack_id: 433, frame_id: 3679\n"
            "track_id: 555, frame_id: 3794\ntrack_id: 477, frame_id: 3813\n"
            "track_id: 540, frame_id: 3758\ntrack_id: 492, frame_id: 3749\n"
            "track_id: 455, frame_id: 3793\ntrack_id: 510, frame_id: 3781\n"
            "track_id: 135, frame_id: 3796\ntrack_id: 175, frame_id: 3786\n"
            "track_id: 501, frame_id: 3669\ntrack_id: 482, frame_id: 3855\n"
            "track_id: 471, frame_id: 3854\ntrack_id: 394, frame_id: 3892\n"
            "track_id: 370, frame_id: 3943\ntrack_id: 558, frame_id: 3941\n"
            "track_id: 116, frame_id: 4010\ntrack_id: 87, frame_id: 4059\n"
            "track_id: 539, frame_id: 4314\ntrack_id: 528, frame_id: 4319\n"
            "track_id: 565, frame_id: 4324\ntrack_id: 303, frame_id: 4405\n"
            "track_id: 442, frame_id: 4445\ntrack_id: 553, frame_id: 4400\n"
            "track_id: 325, frame_id: 4529\ntrack_id: 628, frame_id: 4686\n"
            "track_id: 484, frame_id: 4716\ntrack_id: 579, frame_id: 4743\n"
            "track_id: 588, frame_id: 4804\ntrack_id: 459, frame_id: 4888\n"
            "track_id: 369, frame_id: 4825\ntrack_id: 647, frame_id: 4960\n"
            "track_id: 36, frame_id: 4895\ntrack_id: 643, frame_id: 4997\n"
            "track_id: 170, frame_id: 4931\ntrack_id: 426, frame_id: 4917\n"
            "track_id: 26, frame_id: 5039\ntrack_id: 422, frame_id: 4933\n"
            "track_id: 253, frame_id: 5005\ntrack_id: 641, frame_id: 5003\n"
            "track_id: 249, frame_id: 4992\ntrack_id: 448, frame_id: 4846\n"
            "track_id: 610, frame_id: 5293\ntrack_id: 592, frame_id: 5246\n"
            "track_id: 329, frame_id: 5694\ntrack_id: 277, frame_id: 5636\n"
            "track_id: 545, frame_id: 5655\n";
  EXPECT_EQ(log, gt);
  delete flow;
}

int main(int argc, char* argv[]) {
  SetLogLevel(HOBOT_LOG_ERROR);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
