/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Grading Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.16
 * @date      2019.05.22
 */

#ifndef SNAPSHOTMETHOD_SNAPSHOTMETHOD_H_
#define SNAPSHOTMETHOD_SNAPSHOTMETHOD_H_

#include <string>
#include <utility>
#include <vector>
#include <map>
#include <memory>

#include "hobotxroc/method.h"

namespace HobotXRoc {

struct SnapShotParam;

class SnapShot {
 public:
  virtual int Init(std::shared_ptr<SnapShotParam> config) = 0;

  virtual std::vector<BaseDataPtr> ProcessFrame(
      const std::vector<BaseDataPtr> &in,
      const InputParamPtr &param) = 0;

  virtual void Finalize() = 0;

  virtual int UpdateParameter(const std::string &content) = 0;

  virtual void Reset() {}

 protected:
  std::shared_ptr<SnapShotParam> snapshot_config_param_;
};

class SnapShotMethod : public Method {
 public:
  int Init(const std::string &config_file_path) override;

  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<HobotXRoc::InputParamPtr> &param) override;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override { return "0.0.34"; }

  void OnProfilerChanged(bool on) override {}

  MethodInfo GetMethodInfo() override {
    MethodInfo method_info;
    method_info.is_thread_safe_ = false;
    method_info.is_need_reorder = true;
    return method_info;
  };

 protected:
  std::shared_ptr<SnapShotParam> method_config_param_;
  std::shared_ptr<SnapShotParam> default_method_config_param_;
 private:
  std::vector<BaseDataPtr> ProcessOneBatch(const std::vector<BaseDataPtr> &in,
                                           const InputParamPtr &param);

  std::map<std::string, std::shared_ptr<SnapShot>> strategy_map_;
};
}  // namespace HobotXRoc
#endif  // SNAPSHOTMETHOD_SNAPSHOTMETHOD_H_
