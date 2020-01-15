/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      b_box_filter.h
 * @brief     BBoxFilter class
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-03
 */

#ifndef XROC_TUTORIALS_STAGE1_METHOD_B_BOX_FILTER_H_
#define XROC_TUTORIALS_STAGE1_METHOD_B_BOX_FILTER_H_

#include "../filter_param.h"
#include "hobotxroc/method.h"
#include <atomic>
#include <string>
#include <vector>

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
};

} // namespace HobotXRoc

#endif // XROC_TUTORIALS_STAGE1_METHOD_B_BOX_FILTER_H_
