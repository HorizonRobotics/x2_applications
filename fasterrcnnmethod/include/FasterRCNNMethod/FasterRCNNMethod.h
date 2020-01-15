/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     FasterRCNN Method
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#ifndef INCLUDE_FASTERRCNNMETHOD_FASTERRCNNMETHOD_H_
#define INCLUDE_FASTERRCNNMETHOD_FASTERRCNNMETHOD_H_

#include <string>
#include <vector>
#include <memory>
#include <map>

#include "hobotxroc/method.h"

namespace faster_rcnn_method {
class FasterRCNNImp;
}

namespace HobotXRoc {

class FasterRCNNParam : public HobotXRoc::InputParam {
 public:
  explicit FasterRCNNParam(const std::string &module_name) : HobotXRoc::InputParam(module_name) {
  }
  std::string Format() override {
    return "";
  }
  /// 0 means return all faces found. 1 means return biggest face only
  int max_face_count = 0;
};

using FasterRCNNParamPtr = std::shared_ptr<FasterRCNNParam>;

class FasterRCNNMethod : public Method {
 public:
  FasterRCNNMethod(): Method() {
  }
  virtual ~FasterRCNNMethod() {
  }
  // return 0 for successed, -1 for failed.
  int Init(const std::string &config_file_path) override;

  std::vector<std::vector<BaseDataPtr>> DoProcess(
    const std::vector<std::vector<BaseDataPtr>> &input,
    const std::vector<HobotXRoc::InputParamPtr> &param) override;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;

  void OnProfilerChanged(bool on) override {
  }

  MethodInfo GetMethodInfo() override {
    MethodInfo method_info;
    method_info.is_thread_safe_ = false;
    method_info.is_need_reorder = true;
    return method_info;
  };

 private:
  std::shared_ptr<faster_rcnn_method::FasterRCNNImp> faster_rcnn_imp_;
  FasterRCNNParamPtr method_param_;
};

}  // namespace HobotXRoc

#endif  // INCLUDE_FASTERRCNNMETHOD_FASTERRCNNMETHOD_H_
