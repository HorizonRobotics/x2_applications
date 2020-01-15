/*
 * @Description: implement of vioplugin
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-08-26 16:17:25
 * @Author: songshan.gong@horizon.ai
 * @Date: 2019-09-26 16:17:25
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 15:35:08
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#include <fstream>
#include <iostream>

#include "vioplugin/vioplugin.h"
#include "vioplugin/vioproduce.h"

#include "utils/time_helper.h"

#include "xpluginflow/message/pluginflow/msg_registry.h"
#include "xpluginflow_msgtype/hbipcplugin_data.h"

#include "xpluginflow_msgtype/protobuf/pack.pb.h"
#include "xpluginflow_msgtype/protobuf/x2.pb.h"

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_IMAGE_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_DROP_MESSAGE)

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace vioplugin {

using horizon::vision::xpluginflow::basic_msgtype::HbipcMessage;

int VioPlugin::OnGetHbipcResult(XPluginFlowMessagePtr msg) {
  int ret = 0;
  std::string proto_str;
  static int fps = 0;
  // 耗时统计
  static auto lastTime = hobot::Timer::tic();
  static int frameCount = 0;

  ++frameCount;

  auto curTime = hobot::Timer::toc(lastTime);
  // 统计数据发送帧率
  if (curTime > 1000) {
    fps = frameCount;
    frameCount = 0;
    lastTime = hobot::Timer::tic();
    LOGE << "[HbipcPlugin] fps = " << fps;
  }

  auto hbipc_message = std::static_pointer_cast<HbipcMessage>(msg);
  x2::InfoMessage proto_info_message;
  proto_info_message.ParseFromString(hbipc_message->Serialize());
  if (proto_info_message.config__size() <= 0) {
    LOGE << "PB don't have config param";
    return false;
  }
  for (auto i = 0; i < proto_info_message.config__size(); ++i) {
    auto &params = proto_info_message.config_(i);
    if (params.type_() == "Detect") {
      LOGI << "Set CP detect params";
      for (auto j = 0; j < params.shield__size(); ++j) {
        box_t box;
        auto &shield = params.shield_(j);
        if (shield.type_() == "valid_zone" ||
            shield.type_() == "invalid_zone") {
          auto &point1 = shield.top_left_();
          auto &point2 = shield.bottom_right_();
          box.x1 = point1.x_();
          box.y1 = point1.y_();
          box.x2 = point2.x_();
          box.y2 = point2.y_();
          LOGI << shield.type_() << ":" << box.x1 << " " << box.y1 << " "
               << box.x2 << " " << box.y2;
          Shields_.emplace_back(box);
        } else {
          LOGE << "invalid zone type: " << shield.type_();
        }
      }
    }
  }
  return ret;
}

VioPlugin::VioPlugin(const std::string &path) {
  config_ = GetConfigFromFile(path);
  config_->SetConfig(config_);
  HOBOT_CHECK(config_);
}

int VioPlugin::Init() {
  auto data_source_ = config_->GetValue("data_source");
  VioProduceHandle_ = VioProduce::CreateVioProduce(data_source_);
  HOBOT_CHECK(VioProduceHandle_);
  VioProduceHandle_->SetConfig(config_);
  // 注册智能帧结果
  RegisterMsg(TYPE_HBIPC_MESSAGE, std::bind(&VioPlugin::OnGetHbipcResult, this,
                                            std::placeholders::_1));
  // 调用父类初始化成员函数注册信息
  XPluginAsync::Init();
  return 0;
}

VioPlugin::~VioPlugin() { delete config_; }

int VioPlugin::Start() {
  int ret;

  auto send_frame = [&](const std::shared_ptr<VioMessage> input) {
    if (!input) {
      LOGE << "VioMessage is NULL, return";
      return -1;
    }

    PushMsg(input);
    return 0;
  };

  VioProduceHandle_->SetListener(send_frame);
  ret = VioProduceHandle_->Start();
  if (ret < 0) {
    LOGF << "VioPlugin start failed, err: " << ret << std::endl;
    return -1;
  }

  return 0;
}

int VioPlugin::Stop() {
  VioProduceHandle_->Stop();
  return 0;
}

VioConfig *VioPlugin::GetConfigFromFile(const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    LOGF << "Open config file " << path << " failed";
    return nullptr;
  }
  std::stringstream ss;
  ss << ifs.rdbuf();
  ifs.close();
  std::string content = ss.str();
  Json::Value value;
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> reader(builder.newCharReader());
  try {
    bool ret = reader->parse(content.c_str(), content.c_str() + content.size(),
                             &value, &error);
    if (ret) {
      auto *config = new VioConfig(path, value);
      return config;
    } else {
      return nullptr;
    }
  } catch (std::exception &e) {
    return nullptr;
  }
}

}  // namespace vioplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon
