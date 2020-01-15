/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-04 02:41:22
 * @Version: v0.0.1
 * @Brief: smartplugin declaration
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-30 00:45:01
 */

#ifndef INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_
#define INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_

#include <memory>
#include <string>

#include "xpluginflow/message/pluginflow/flowmsg.h"
#include "xpluginflow/plugin/xpluginasync.h"

#include "hobotxsdk/xroc_sdk.h"
#include "smartplugin/runtime_monitor.h"
#include "smartplugin/smart_config.h"

#include "xpluginflow_msgtype/smartplugin_data.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace smartplugin {

using horizon::vision::xpluginflow::XPluginAsync;
using horizon::vision::xpluginflow::XPluginFlowMessage;
using horizon::vision::xpluginflow::XPluginFlowMessagePtr;

using HobotXRoc::InputDataPtr;
using HobotXRoc::OutputDataPtr;
using HobotXRoc::XRocSDK;
using horizon::vision::xpluginflow::basic_msgtype::SmartMessage;


struct CustomSmartMessage : SmartMessage {
  explicit CustomSmartMessage(
    HobotXRoc::OutputDataPtr out) : smart_result(out) {
    type_ = TYPE_SMART_MESSAGE;
  }
  std::string Serialize() override;

 private:
  HobotXRoc::OutputDataPtr smart_result;
};

class SmartPlugin : public XPluginAsync {
 public:
  SmartPlugin() = default;
  explicit SmartPlugin(const std::string& config_file);

  void SetConfig(const std::string& config_file) { config_file_ = config_file; }

  ~SmartPlugin() = default;
  int Init() override;

 private:
  int Feed(XPluginFlowMessagePtr msg);
  void OnCallback(HobotXRoc::OutputDataPtr out);
  void ParseConfig();

  std::shared_ptr<XRocSDK> sdk_;
  std::string config_file_;
  std::shared_ptr<RuntimeMonitor> monitor_;
  std::shared_ptr<JsonConfigWrapper> config_;
  std::string xroc_workflow_cfg_file_;
  bool enable_profile_{false};
  std::string profile_log_file_;
};

}  // namespace smartplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon
#endif  // INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_
