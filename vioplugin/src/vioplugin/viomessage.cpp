/*
 * @Description: implement of vioplugin
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-08-26 16:17:25
 * @Author: songshan.gong@horizon.ai
 * @Date: 2019-09-26 16:17:25
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 15:34:58
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#include <iostream>

#include "hobotlog/hobotlog.hpp"
#include "horizon/vision_type/vision_type_util.h"
#include "vioplugin/viomessage.h"
#include "xpluginflow_msgtype/protobuf/x2.pb.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace vioplugin {

ImageVioMessage::ImageVioMessage(HorizonVisionImageFrame **image_frame,
                                 uint32_t img_num, bool is_valid,
                                 mult_img_info_t *info) {
  type_ = TYPE_IMAGE_MESSAGE;
  num_ = img_num;
  time_stamp_ = image_frame[0]->time_stamp;
  sequence_id_ = image_frame[0]->frame_id;
  image_ = image_frame;
  is_valid_uri_ = is_valid;
  multi_info_ = info;
}

DropVioMessage::DropVioMessage(uint64_t timestamp, uint64_t seq_id) {
  type_ = TYPE_DROP_MESSAGE;
  time_stamp_ = timestamp;
  sequence_id_ = seq_id;
}

std::string DropVioMessage::Serialize() {
  std::string smart_str;
  x2::SmartFrameMessage smart_frame_message;

  LOGI << "Drop Serialize";
  smart_frame_message.set_timestamp_(time_stamp_);
  smart_frame_message.set_error_code_(1);
  smart_frame_message.SerializeToString(&smart_str);

  return smart_str;
}

void ImageVioMessage::FreeImage() {
  if (image_) {
    // free image
    if (num_ == 1) {
      auto *img_info = reinterpret_cast<img_info_t *>(image_[0]->image.data);
      hb_vio_free(img_info);
      HorizonVisionFreeImageFrame(image_[0]);
    } else if (num_ == 2) {
      hb_vio_mult_free(multi_info_);
      std::free(multi_info_);
      for (uint32_t i = 0; i < num_; ++i) {
        std::free(image_[i]);
      }
    }
    delete[] image_;
  }
  image_ = nullptr;
}

}  // namespace vioplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon
