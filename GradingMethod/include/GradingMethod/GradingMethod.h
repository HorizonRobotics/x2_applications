/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Grading Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.25
 */

#ifndef GRADINGMETHOD_GRADINGMETHOD_H_
#define GRADINGMETHOD_GRADINGMETHOD_H_

#include <string>
#include <vector>

#include "hobotxroc/method.h"


namespace HobotXRoc {

class Grading {
 public:
  virtual int GradingInit(const std::string &config_file_path) = 0;

  virtual int ProcessFrame(const std::vector<BaseDataPtr> &in,
                           const InputParamPtr &param,
                           std::vector<BaseDataPtr> &out) = 0;

  virtual void GradingFinalize() = 0;

  virtual InputParamPtr GetParameter() = 0;

  virtual int UpdateParameter(const std::string &content) = 0;
};


class GradingMethod : public Method {
 public:
  int Init(const std::string &config_file_path) override;

  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<HobotXRoc::InputParamPtr> &param) override;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;

  void OnProfilerChanged(bool on) override { }

  MethodInfo GetMethodInfo() override {
    MethodInfo method_info;
    method_info.is_thread_safe_ = false;
    method_info.is_need_reorder = false;
    return method_info;
  };

 private:
  std::shared_ptr<Grading> grading_;
};

}  // namespace HobotXRoc

#endif  // GRADINGMETHOD_GRADINGMETHOD_H_
