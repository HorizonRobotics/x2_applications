/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     BBoxFilter Method
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.23
 */

#ifndef TEST_INCLUDE_METHOD_BBOX_FILTER_H_
#define TEST_INCLUDE_METHOD_BBOX_FILTER_H_

#include <atomic>
#include <string>
#include <vector>
#include "hobotxroc/data_types/filter_param.h"
#include "hobotxroc/method.h"
#include "json/json.h"

namespace HobotXRoc {

class BBoxFilter : public Method {
 public:
  int Init(const std::string &config_file_path) override;

  std::vector<std::vector<BaseDataPtr>>
  DoProcess(const std::vector<std::vector<BaseDataPtr>> &input,
            const std::vector<HobotXRoc::InputParamPtr> &param) override;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;
  void OnProfilerChanged(bool on) override;
 private:
  std::atomic<float> area_threshold_;

  // for thread model test
  bool has_thread_params_{false};
  std::string thr_policy_;
  int thr_priority_;
};

}  // namespace HobotXRoc

#endif  // TEST_INCLUDE_METHOD_BBOX_FILTER_H_
