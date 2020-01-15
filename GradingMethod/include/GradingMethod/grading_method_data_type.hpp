/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     grading method data type header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.05.24
 */

#ifndef GRADINGMETHOD_GRADING_METHOD_DATA_TYPE_HPP_
#define GRADINGMETHOD_GRADING_METHOD_DATA_TYPE_HPP_

#include <string>

namespace HobotXRoc {

#define SET_GRADING_METHOD_PARAM(json_cfg, type, key)                        \
        if (json_cfg.isMember(#key) && json_cfg[#key].is##type())            \
            key = json_cfg[#key].as##type()


struct GradingParam : public HobotXRoc::InputParam {
  std::string grading_type = "";
  explicit GradingParam(const std::string &content): InputParam("GradingMethod") {
    is_enable_this_method_ = true;
    is_json_format_ = true;
    method_name_ = "GradingMethod";
  }
  Json::Value config_jv;
  virtual int UpdateParameter(const std::string &content);
  std::string Format() override;
};
}  // namespace HobotXRoc

#endif // GRADINGMETHOD_GRADING_METHOD_DATA_TYPE_HPP_
