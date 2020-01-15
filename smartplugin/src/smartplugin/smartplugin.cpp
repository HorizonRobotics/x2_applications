/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-01 20:38:52
 * @Version: v0.0.1
 * @Brief: smartplugin impl based on xroc.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 05:04:11
 */

#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "hobotlog/hobotlog.hpp"
#include "xpluginflow/message/pluginflow/flowmsg.h"
#include "xpluginflow/message/pluginflow/msg_registry.h"
#include "xpluginflow/plugin/xpluginasync.h"

#include "hobotxsdk/xroc_sdk.h"
#include "horizon/vision/util.h"
#include "horizon/vision_type/vision_error.h"
#include "horizon/vision_type/vision_type.hpp"
#include "smartplugin/convert.h"
#include "smartplugin/runtime_monitor.h"
#include "smartplugin/smart_config.h"
#include "smartplugin/smartplugin.h"
#include "xpluginflow_msgtype/protobuf/x2.pb.h"
#include "xpluginflow_msgtype/vioplugin_data.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace smartplugin {

using horizon::vision::xpluginflow::XPluginAsync;
using horizon::vision::xpluginflow::XPluginFlowMessage;
using horizon::vision::xpluginflow::XPluginFlowMessagePtr;

using horizon::vision::xpluginflow::basic_msgtype::VioMessage;
using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
using XRocImageFramePtr = HobotXRoc::XRocData<ImageFramePtr>;

using HobotXRoc::InputDataPtr;
using HobotXRoc::OutputDataPtr;
using HobotXRoc::XRocSDK;

using horizon::iot::Convertor;

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_MESSAGE)

std::string CustomSmartMessage::Serialize() {
  // serialize smart message using defined smart protobuf.
  std::string proto_str;
  x2::FrameMessage proto_frame_message;
  auto smart_msg = proto_frame_message.mutable_smart_msg_();
  smart_msg->set_timestamp_(time_stamp);
  smart_msg->set_error_code_(0);
  // user-defined output parsing declaration.
  HobotXRoc::BaseDataVector *face_boxes = nullptr;
  HobotXRoc::BaseDataVector *lmks = nullptr;
  for (const auto &output : smart_result->datas_) {
    LOGD << "output name: " << output->name_;
    if (output->name_ == "face_bbox_list" || output->name_ == "head_box" ||
        output->name_ == "body_box") {
      face_boxes = dynamic_cast<HobotXRoc::BaseDataVector *>(output.get());
      LOGD << "box size: " << face_boxes->datas_.size();
      for (int i = 0; i < face_boxes->datas_.size(); ++i) {
        auto face_box =
            std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::BBox>>(
                face_boxes->datas_[i]);
        LOGD << "x1: " << face_box->value.x1 << " y1: " << face_box->value.y1
             << " x2: " << face_box->value.x2 << " y2: " << face_box->value.y2
             << " " << output->name_ << " id: " << face_box->value.id << "\n";
        auto target = smart_msg->add_targets_();
        if (output->name_ == "face_bbox_list") {
          target->set_type_("face");
        } else if (output->name_ == "head_box") {
          target->set_type_("head");
        } else if (output->name_ == "body_box") {
          target->set_type_("body");
        }
        target->set_track_id_(face_box->value.id);
        auto proto_box = target->add_boxes_();
        if (output->name_ == "face_bbox_list") {
          proto_box->set_type_("face");
        } else if (output->name_ == "head_box") {
          proto_box->set_type_("head");
        } else if (output->name_ == "body_box") {
          proto_box->set_type_("body");
        }
        auto point1 = proto_box->mutable_top_left_();
        point1->set_x_(face_box->value.x1);
        point1->set_y_(face_box->value.y1);
        point1->set_score_(face_box->value.score);
        auto point2 = proto_box->mutable_bottom_right_();
        point2->set_x_(face_box->value.x2);
        point2->set_y_(face_box->value.y2);
        point2->set_score_(face_box->value.score);
      }
    }
    if (output->name_ == "lmk") {
      lmks = dynamic_cast<HobotXRoc::BaseDataVector *>(output.get());
      LOGD << "face lmk size: " << lmks->datas_.size();
      for (int i = 0; i < lmks->datas_.size(); ++i) {
        auto lmk = std::static_pointer_cast<
            HobotXRoc::XRocData<hobot::vision::Landmarks>>(lmks->datas_[i]);
        LOGD << "size " << lmk->value.values.size()
             << "score: " << lmk->value.score << "\n";
        auto target = smart_msg->add_targets_();
        target->set_type_("lmk");
        auto proto_points = target->add_points_();
        proto_points->set_type_("landmarks");
        for (int i = 0; i < lmk->value.values.size(); ++i) {
          auto point = proto_points->add_points_();
          point->set_x_(lmk->value.values[i].x);
          point->set_y_(lmk->value.values[i].y);
          point->set_score_(lmk->value.values[i].score);
          LOGD << "x: " << lmk->value.values[i].x
               << "y: " << lmk->value.values[i].y
               << "score: " << lmk->value.values[i].score << "\n";
        }
      }
    }
    if (output->name_ == "kps") {
      lmks = dynamic_cast<HobotXRoc::BaseDataVector *>(output.get());
      LOGD << "kps size: " << lmks->datas_.size();
      for (int i = 0; i < lmks->datas_.size(); ++i) {
        auto lmk = std::static_pointer_cast<
            HobotXRoc::XRocData<hobot::vision::Landmarks>>(lmks->datas_[i]);
        LOGD << "size " << lmk->value.values.size()
             << "score: " << lmk->value.score << "\n";
        auto target = smart_msg->add_targets_();
        target->set_type_("kps");
        auto proto_points = target->add_points_();
        proto_points->set_type_("landmarks");
        for (int i = 0; i < lmk->value.values.size(); ++i) {
          auto point = proto_points->add_points_();
          point->set_x_(lmk->value.values[i].x);
          point->set_y_(lmk->value.values[i].y);
          point->set_score_(lmk->value.values[i].score);
          LOGD << "x: " << lmk->value.values[i].x
               << "y: " << lmk->value.values[i].y
               << "score: " << lmk->value.values[i].score << "\n";
        }
      }
    }
    if (output->name_ == "age") {
      auto ages = dynamic_cast<HobotXRoc::BaseDataVector *>(output.get());
      LOGD << "age size: " << ages->datas_.size();
      for (int i = 0; i < ages->datas_.size(); ++i) {
        auto age =
            std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::Age>>(
                ages->datas_[i]);
        if (age->state_ != HobotXRoc::DataState::VALID) {
          LOGE << "-1 -1 -1";
        }
        auto target = smart_msg->mutable_targets_(i);
        auto attrs = target->add_attributes_();
        attrs->set_type_("age");
        attrs->set_value_((age->value.min + age->value.max) / 2);
        attrs->set_score_(age->value.score);

        LOGD << " " << age->value.min << " " << age->value.max;
      }
    }
    if (output->name_ == "gender") {
      auto genders = dynamic_cast<HobotXRoc::BaseDataVector *>(output.get());
      LOGD << "gender size: " << genders->datas_.size();
      for (int i = 0; i < genders->datas_.size(); ++i) {
        auto gender = std::static_pointer_cast<
            HobotXRoc::XRocData<hobot::vision::Gender>>(genders->datas_[i]);
        if (genders->state_ != HobotXRoc::DataState::VALID) {
          LOGE << "-1";
        }
        auto target = smart_msg->mutable_targets_(i);
        auto attrs = target->add_attributes_();
        attrs->set_type_("gender");
        attrs->set_value_(gender->value.value);
        attrs->set_score_(gender->value.score);
        LOGD << " " << gender->value.value;
      }
    }
    if (output->name_ == "feature_list") {
      auto feat_list = dynamic_cast<HobotXRoc::BaseDataVector*>(output.get());
      LOGD << "feature list size: " << feat_list->datas_.size();
      for (int i = 0; i < feat_list->datas_.size(); i++) {
        auto one_person_feature_list = std::static_pointer_cast<
        HobotXRoc::BaseDataVector>(feat_list->datas_[i]);
        for (int j = 0; j < one_person_feature_list->datas_.size(); j++) {
          auto feature = std::static_pointer_cast<
          HobotXRoc::XRocData<hobot::vision::Feature>>(
            one_person_feature_list->datas_[j]);
          LOGD << "feature:";
          for (int k = 0; k < feature->value.values.size(); k++) {
            LOGD << " " << feature->value.values[k];
          }
        }
      }
    }
  }
  proto_frame_message.SerializeToString(&proto_str);
  return proto_str;
}
SmartPlugin::SmartPlugin(const std::string &config_file) {
  config_file_ = config_file;
  LOGI << "smart config file:" << config_file_;
  monitor_.reset(new RuntimeMonitor());
  Json::Value cfg_jv;
  std::ifstream infile(config_file_);
  infile >> cfg_jv;
  config_.reset(new JsonConfigWrapper(cfg_jv));
  ParseConfig();
}

void SmartPlugin::ParseConfig() {
  xroc_workflow_cfg_file_ = config_->GetSTDStringValue("xroc_workflow_file");
  enable_profile_ = config_->GetBoolValue("enable_profile");
  profile_log_file_ = config_->GetSTDStringValue("profile_log_path");
  LOGI << "xroc_workflow_file:" << xroc_workflow_cfg_file_;
  LOGI << "enable_profile:" << enable_profile_
       << ", profile_log_path:" << profile_log_file_;
}

int SmartPlugin::Init() {
  // init for xroc sdk
  sdk_.reset(HobotXRoc::XRocSDK::CreateSDK());
  sdk_->SetConfig("config_file", xroc_workflow_cfg_file_);
  if (sdk_->Init() != 0) {
    return kHorizonVisionInitFail;
  }
  if (enable_profile_) {
    sdk_->SetConfig("profiler", "on");
    sdk_->SetConfig("profiler_file", profile_log_file_);
  }
  sdk_->SetCallback(
      std::bind(&SmartPlugin::OnCallback, this, std::placeholders::_1));

  RegisterMsg(TYPE_IMAGE_MESSAGE,
              std::bind(&SmartPlugin::Feed, this, std::placeholders::_1));
  return XPluginAsync::Init();
}

int SmartPlugin::Feed(XPluginFlowMessagePtr msg) {
  // feed video frame to xrocsdk.
  // 1. parse valid frame from msg
  auto valid_frame = std::static_pointer_cast<VioMessage>(msg);
  HobotXRoc::InputDataPtr input = Convertor::ConvertInput(valid_frame.get());
  SmartInput *input_wrapper = new SmartInput();
  input_wrapper->frame_info = valid_frame;
  input_wrapper->context = input_wrapper;
  monitor_->PushFrame(input_wrapper);
  if (sdk_->AsyncPredict(input) != 0) {
    return kHorizonVisionFailure;
  }

  return 0;
}

void SmartPlugin::OnCallback(HobotXRoc::OutputDataPtr xroc_out) {
  // On xroc async-predict returned,
  // transform xroc standard output to smart message.
  HOBOT_CHECK(!xroc_out->datas_.empty()) << "Empty XRoc Output";

  XRocImageFramePtr *rgb_image = nullptr;

  for (const auto &output : xroc_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == "rgb_image" || output->name_ == "image") {
      rgb_image = dynamic_cast<XRocImageFramePtr *>(output.get());
    }
  }

  auto smart_msg = std::make_shared<CustomSmartMessage>(xroc_out);
  // Set origin input named "image" as output always.
  HOBOT_CHECK(rgb_image);
  smart_msg->time_stamp = rgb_image->value->time_stamp;
  smart_msg->frame_id = rgb_image->value->frame_id;
  auto input = monitor_->PopFrame(smart_msg->frame_id);
  delete static_cast<SmartInput *>(input.context);
  // PushMsg(smart_msg);
  smart_msg->Serialize();
}
}  // namespace smartplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon
