/**
 * @file method_factory.cpp
 * @author your name (you@domain.com)
 * @brief DO NOT MODIFY THIS FILE, WHICH IS AUTO GENERATED BY COMPILER
 * @version 0.1
 * @date 2018-11-23
 *
 * @copyright Copyright (c) 2018
 *
 */

#include "hobotxroc/method_factory.h"

#include "FasterRCNNMethod/FasterRCNNMethod.h"
#include <iostream>
namespace HobotXRoc {
MethodPtr MethodFactory::CreateMethod(const std::string &method_name) {
  std::cout << "MethodFactory::CreateMethod:" << method_name << std::endl;
  if ("FasterRCNNMethod" == method_name) {
    return MethodPtr(new FasterRCNNMethod());
  } else {
    return MethodPtr();
  }
}
}  // namespace HobotXRoc
