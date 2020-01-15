/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-04 03:07:26
 * @Version: v0.0.1
 * @Brief: runtime monitor implementation
 * @Note:  extracted from repo xperson's global_config.cpp
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-08-22 23:46:17
 */

#include "smartplugin/runtime_monitor.h"
#include <memory>
#include <mutex>
#include "hobotlog/hobotlog.hpp"
#include "horizon/vision/util.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace smartplugin {
using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
using XRocImageFramePtr = HobotXRoc::XRocData<ImageFramePtr>;

void RuntimeMonitor::PushFrame(const SmartInput *input) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  HOBOT_CHECK(input) << "Null HorizonVisionImageFrame";
  auto frame_info = input->frame_info;
  HOBOT_CHECK(frame_info->num_ > 0);
  auto image0 = frame_info->image_[0];
  input_frames_[image0->frame_id].image_num = frame_info->num_;
  input_frames_[image0->frame_id].img = frame_info->image_;
  input_frames_[image0->frame_id].context = input->context;
}

RuntimeMonitor::InputFrameData RuntimeMonitor::PopFrame(
    const int32_t &frame_id) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  InputFrameData input;
  auto itr = input_frames_.find(frame_id);
  if (itr != input_frames_.end()) {
    input = itr->second;
    LOGI << "Pop frame " << frame_id;
    input_frames_.erase(itr);
  }
  return input;
}

RuntimeMonitor::RuntimeMonitor() { Reset(); }

bool RuntimeMonitor::Reset() { return true; }

}  // namespace smartplugin
}  // namespace xpluginflow
}  //  namespace vision
}  //  namespace horizon
