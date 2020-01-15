/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief provides base data struct for xsoul framework
 * @author    jianbo.qin
 * @email     jianbo.qin@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.14
 */

#ifndef HOBOTXROC_DATA_TYPES_FILTER_PARAM_H_
#define HOBOTXROC_DATA_TYPES_FILTER_PARAM_H_

#include <string>

#include "hobotxsdk/xroc_data.h"

namespace HobotXRoc {

typedef struct _FilterParam__isset {
  _FilterParam__isset() : threshold(false) {}

  bool threshold : 1;
} _FilterParam__isset;

class FilterParam : public InputParam {
 public:
  explicit FilterParam(std::string method_name) : InputParam(method_name) {
    threshold_ = 2500.0;
  }
  virtual ~FilterParam() = default;

  virtual std::string Format() {
    return std::string("threshold") + std::to_string(threshold_);
  }

  void SetThreshold(float thres) {
    threshold_ = thres;
    is_set_.threshold = true;
  }

  bool HasThreshold() { return is_set_.threshold; }

  float GetThreshold() { return threshold_; }

 private:
  _FilterParam__isset is_set_;
  float threshold_;
};

}  // namespace HobotXRoc
#endif  // HOBOTXROC_DATA_TYPES_FILTER_PARAM_H_
