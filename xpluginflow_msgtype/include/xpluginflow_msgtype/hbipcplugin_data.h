/*
 * @Description: implement of hbipcplugin.h
 * @Author: yingmin.li@horizon.ai
 * @Date: 2019-08-24 11:29:24
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 16:11:08
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#ifndef XPLUGINFLOW_MSGTYPE_HBIPCPLUGIN_DATA_H_
#define XPLUGINFLOW_MSGTYPE_HBIPCPLUGIN_DATA_H_

#include "xpluginflow/message/pluginflow/flowmsg.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace basic_msgtype {

#define TYPE_HBIPC_MESSAGE "XPLUGIN_HBIPC_MESSAGE"

struct HbipcMessage : XPluginFlowMessage {
  HbipcMessage() { type_ = TYPE_HBIPC_MESSAGE; }
  std::string Serialize() override { return "Default hbipc message"; }
  virtual ~HbipcMessage() = default;
};

}  // namespace basic_msgtype
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon

#endif  // HBIPCPLUGIN_INCLUDE_HBIPCPLUGIN_HBIPCPLUGIN_H_
