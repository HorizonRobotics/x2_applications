/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: PostPredictor.cpp
 * @Brief: definition of the PostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 15:18:10
 */

#include "CNNMethod/PostPredictor/PostPredictor.h"
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "CNNMethod/CNNConst.h"
#include "CNNMethod/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/profiler.h"

namespace HobotXRoc {

int32_t PostPredictor::Init(std::shared_ptr<CNNMethodConfig> config) {
  if (nullptr == config) {
    return -1;
  }
  model_name_ = config->GetSTDStringValue("model_name");
  output_slot_size_ = config->GetIntValue("output_size");
  HOBOT_CHECK(output_slot_size_ > 0);
  return 0;
}

}  // namespace HobotXRoc
