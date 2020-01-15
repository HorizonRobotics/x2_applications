/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Method interface of xroc framework
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.01.17
 */

#include "hobotxroc/method.h"

int HobotXRoc::Method::UpdateParameter(HobotXRoc::InputParamPtr ptr) {
  return 0;
}

HobotXRoc::MethodInfo HobotXRoc::Method::GetMethodInfo() {
  return MethodInfo();
}

HobotXRoc::Method::~Method() = default;

