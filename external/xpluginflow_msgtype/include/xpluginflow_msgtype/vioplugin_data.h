/*
 * @Description: implement of  vio data header
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-10-14 16:35:21
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 16:11:58
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#ifndef XPLUGINFLOW_MSGTYPE_VIOPLUGIN_DATA_H_
#define XPLUGINFLOW_MSGTYPE_VIOPLUGIN_DATA_H_

#include <memory>

#include "hb_vio_interface.h"
#include "horizon/vision_type/vision_error.h"
#include "horizon/vision_type/vision_msg.h"
#include "horizon/vision_type/vision_type.h"
#include "xpluginflow/message/pluginflow/flowmsg.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace basic_msgtype {

#define TYPE_IMAGE_MESSAGE "XPLUGIN_IMAGE_MESSAGE"
#define TYPE_DROP_MESSAGE "XPLUGIN_DROP_MESSAGE"

struct VioMessage : public XPluginFlowMessage {
 public:
  VioMessage() { type_ = TYPE_IMAGE_MESSAGE; };
  virtual ~VioMessage() = default;

  // image frames number
  uint32_t num_ = 0;
  // sequence id, would increment automatically
  uint64_t sequence_id_ = 0;
  // time stamp
  uint64_t time_stamp_ = 0;
  // is valid uri
  bool is_valid_uri_ = true;
  // image frames
  HorizonVisionImageFrame **image_ = nullptr;
  // free source image
  void FreeImage();
  // serialize proto
  std::string Serialize() override { return "Default vio message"; };
  // multi
  mult_img_info_t *multi_info_ = nullptr;
};

}  // namespace basic_msgtype
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon

#endif  // XPLUGINFLOW_MSGTYPE_VIOPLUGIN_DATA_H_
