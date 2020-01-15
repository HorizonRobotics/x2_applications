/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     method factory
 * @file      method_fatory.h
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef HOBOTXROC_METHOD_FACTORY_H_
#define HOBOTXROC_METHOD_FACTORY_H_

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

#endif  // HOBOTXROC_METHOD_FACTORY_H_
