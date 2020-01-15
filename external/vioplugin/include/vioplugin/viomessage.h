/*
 * @Description: implement of vioplugin
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-08-26 16:17:25
 * @Author: songshan.gong@horizon.ai
 * @Date: 2019-09-26 16:17:25
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 16:27:39
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#ifndef INCLUDE_VIOMESSAGE_VIOMESSAGE_H_
#define INCLUDE_VIOMESSAGE_VIOMESSAGE_H_

#include <memory>

#include "hb_vio_interface.h"
#include "x2_camera.h"

#include "hobot_vision/blocking_queue.hpp"

#include "xpluginflow_msgtype/vioplugin_data.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace vioplugin {
using horizon::vision::xpluginflow::basic_msgtype::VioMessage;

struct ImageVioMessage : VioMessage {
 public:
  ImageVioMessage() = delete;
  explicit ImageVioMessage(HorizonVisionImageFrame **image_frame,
                           uint32_t img_num, bool is_valid = true,
                           mult_img_info_t *info = nullptr);
  ~ImageVioMessage(){};

  // serialize proto
  std::string Serialize() { return "No need serialize"; };

  void FreeImage();
};

struct DropVioMessage : VioMessage {
 public:
  DropVioMessage() = delete;
  explicit DropVioMessage(uint64_t timestamp, uint64_t seq_id);
  ~DropVioMessage(){};

  // serialize proto
  std::string Serialize() override;
};

}  // namespace vioplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon

#endif
