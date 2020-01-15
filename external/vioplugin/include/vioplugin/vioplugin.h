/*
 * @Description: implement of vioplugin
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-08-26 16:17:25
 * @Author: songshan.gong@horizon.ai
 * @Date: 2019-09-26 16:17:25
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-15 11:14:07
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#ifndef INCLUDE_VIOPLUGIN_VIOPLUGIN_H_
#define INCLUDE_VIOPLUGIN_VIOPLUGIN_H_

#include "json/json.h"
#include "vioplugin/vioproduce.h"
#include "xpluginflow/plugin/xpluginasync.h"

#include "horizon/vision_type/vision_type.hpp"

#define VIO_CAMERA "vio_camera"
#define VIO_FEEDBACK "vio_feedback"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace vioplugin {

using box_t = hobot::vision::BBox_<uint32_t>;

class VioPlugin : public xpluginflow::XPluginAsync {
 public:
  VioPlugin() = delete;
  explicit VioPlugin(const std::string &path);
  ~VioPlugin() override;
  int Init() override;
  int Start() override;
  int Stop() override;
  std::string desc() const { return "VioPlugin"; }

 private:
  VioConfig *GetConfigFromFile(const std::string &path);
  int OnGetHbipcResult(const XPluginFlowMessagePtr msg);

 private:
  VioConfig *config_;
  std::shared_ptr<VioProduce> VioProduceHandle_;
  std::vector<box_t> Shields_;
};

}  // namespace vioplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon
#endif
