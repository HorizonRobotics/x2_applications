/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      method_factory.h
 * @brief     MethodFactory class definition
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-03
 */

#ifndef XROC_TUTORIALS_STAGE1_METHOD_FACTORY_H_
#define XROC_TUTORIALS_STAGE1_METHOD_FACTORY_H_

#include <string>
#include "hobotxroc/method.h"
#include "hobotxsdk/xroc_data.h"

namespace HobotXRoc {

class MethodFactory {
 public:
  /// Method 工厂方法
  static MethodPtr CreateMethod(const std::string &method_name);
};

}  // namespace HobotXRoc

#endif  // XROC_TUTORIALS_STAGE1_METHOD_FACTORY_H_
