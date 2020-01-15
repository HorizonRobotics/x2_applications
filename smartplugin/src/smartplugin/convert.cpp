/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-02 04:05:52
 * @Version: v0.0.1
 * @Brief: implemenation of converter.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-08-02 06:26:01
 */

#include "hobotxsdk/xroc_data.h"
#include "horizon/vision/util.h"

#include "hobotlog/hobotlog.hpp"
#include "smartplugin/convert.h"
#include "xpluginflow_msgtype/vioplugin_data.h"

namespace horizon {
namespace iot {

using horizon::vision::util::ImageFrameConversion;

HobotXRoc::InputDataPtr Convertor::ConvertInput(const VioMessage *input) {
  HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
  HOBOT_CHECK(input != nullptr && input->num_ > 0 && input->is_valid_uri_);

  // \todo need a better way to identify mono or semi cameras
  for (uint32_t image_index = 0; image_index < 1; ++image_index) {
    HobotXRoc::BaseDataPtr xroc_input_data;
    if (input->num_ > image_index) {
      auto xroc_img = ImageFrameConversion(input->image_[image_index]);
      xroc_input_data = HobotXRoc::BaseDataPtr(xroc_img);
      LOGI << "Input Frame ID = " << xroc_img->value->frame_id
           << ", Timestamp = " << xroc_img->value->time_stamp;
    } else {
      xroc_input_data = std::make_shared<HobotXRoc::BaseData>();
      xroc_input_data->state_ = HobotXRoc::DataState::INVALID;
    }

    if (image_index == uint32_t{0}) {
      if (input->num_ == 1) {
        xroc_input_data->name_ = "image";
      } else {
        xroc_input_data->name_ = "rgb_image";
      }
    } else {
      xroc_input_data->name_ = "nir_image";
    }
    LOGI << "input name:" << xroc_input_data->name_;
    inputdata->datas_.emplace_back(xroc_input_data);
  }

  return inputdata;
}

}  // namespace iot
}  // namespace horizon
