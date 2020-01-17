/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     test_xplugin.cpp
 * \Author   Yingmin Li
 * \Mail     yingmin.li@horizon.ai
 * \Version  1.0.0.0
 * \Date     2019-07-29
 * \Brief    implement of test_xplugin.cpp
 * \DO NOT MODIFY THIS COMMENT, \
 * \WHICH IS AUTO GENERATED BY EDITOR
 * -------------------------------------------
 */
#include <chrono>
#include <string>
#include <thread>
#include "gtest/gtest.h"
#include "hobotlog/hobotlog.hpp"
#include "xpluginflow/message/pluginflow/flowmsg.h"
#include "xpluginflow/plugin/xpluginasync.h"
#include "xpluginflow/message/pluginflow/msg_registry.h"

using std::chrono::milliseconds;
using horizon::vision::xpluginflow::XPluginAsync;
using horizon::vision::xpluginflow::XPluginFlowMessage;
using horizon::vision::xpluginflow::XPluginFlowMessagePtr;

namespace {

#define TYPE_VIO_MESSAGE "XPLUGIN_VIO_MESSAGE"
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_VIO_MESSAGE)

struct VioMessage : XPluginFlowMessage {
  VioMessage() { type_ = TYPE_VIO_MESSAGE; }
  std::string Serialize() override { return std::string(); }
};
class TestXRocPlugin : public XPluginAsync {
 public:
  explicit TestXRocPlugin(int thead_num) : XPluginAsync(thead_num) {
    LOGD << "TestXRocPlugin cons";
  }
  TestXRocPlugin() = default;
  ~TestXRocPlugin() override = default;
  std::string desc() const override {
    return "TestXRocPlugin";
  }
  int Init() override {
    RegisterMsg(TYPE_VIO_MESSAGE, std::bind(&TestXRocPlugin::MsgProcess,
                                            this, std::placeholders::_1));
    return XPluginAsync::Init();
  }
  int MsgProcess(XPluginFlowMessagePtr msg) {
    LOGI << "Process in testxRocPlugin";
    return 0;
  }
};

class TestVioPlugin : public XPluginAsync {
 public:
  TestVioPlugin() : XPluginAsync() { LOGI << "TestVioPlugin cons"; }
  ~TestVioPlugin() override = default;
  int Init() override {
    LOGD << "input plugin do not register any msgtype";
    return XPluginAsync::Init();
  }

  std::string desc() const {
    return "TestVioPlugin";
  }

  int Start() {
    while (1) {
      auto viomsg = std::make_shared<VioMessage>();
      PushMsg(viomsg);
      LOGD << "push msg to bus";
      std::this_thread::sleep_for(milliseconds(40));
    }
  }
};
TEST(xpluginflow, xplugin) {
  SetLogLevel(HOBOT_LOG_DEBUG);
  auto vio_plugin = std::make_shared<TestVioPlugin>();
  auto xroc_plugin = std::make_shared<TestXRocPlugin>(2);
  vio_plugin->Init();
  xroc_plugin->Init();
  vio_plugin->Start();
}

}  // namespace