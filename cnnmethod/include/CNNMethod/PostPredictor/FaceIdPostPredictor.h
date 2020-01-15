/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: FaceIdPostPredictor.h
 * @Brief: declaration of the FaceIdPostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 15:13:07
 */

#ifndef INCLUDE_CNNMETHOD_POSTPREDICTOR_FACEIDPOSTPREDICTOR_H_
#define INCLUDE_CNNMETHOD_POSTPREDICTOR_FACEIDPOSTPREDICTOR_H_

#include <vector>
#include "CNNMethod/PostPredictor/PostPredictor.h"

namespace HobotXRoc {

class FaceIdPostPredictor : public PostPredictor {
 public:
  virtual void Do(CNNMethodRunData *run_data);

 private:
  BaseDataPtr
    FaceFeaturePostPro(const std::vector<std::vector<int8_t>> &mxnet_outs);
};
}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_POSTPREDICTOR_FACEIDPOSTPREDICTOR_H_
