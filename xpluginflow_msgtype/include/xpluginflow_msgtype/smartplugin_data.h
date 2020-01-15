/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-09-29 01:50:41
 * @Version: v0.0.1
 * @Brief: smart message declaration.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 02:49:28
 */

#ifndef XPLUGINFLOW_MSGTYPE_SMARTPLUGIN_DATA_H_
#define XPLUGINFLOW_MSGTYPE_SMARTPLUGIN_DATA_H_

#include "xpluginflow/message/pluginflow/flowmsg.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace basic_msgtype {

#define TYPE_SMART_MESSAGE "XPLUGIN_SMART_MESSAGE"

struct SmartMessage : XPluginFlowMessage {
  uint64_t time_stamp;
  uint64_t frame_id;
  SmartMessage() { type_ = TYPE_SMART_MESSAGE; }
  virtual ~SmartMessage() = default;

  std::string Serialize() override { return "Default smart message"; };
};

}  // namespace basic_msgtype
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon

#endif  // XPLUGINFLOW_MSGTYPE_SMARTPLUGIN_DATA_H_