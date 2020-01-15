/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: FaceQualityPostPredictor.h
 * @Brief: declaration of the FaceQualityPostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-09-26 21:38:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-09-26 21:38:28
 */

#ifndef INCLUDE_CNNMETHOD_POSTPREDICTOR_FACEQUALITYPOSTPREDICTOR_H_
#define INCLUDE_CNNMETHOD_POSTPREDICTOR_FACEQUALITYPOSTPREDICTOR_H_

#include <memory>
#include <vector>
#include "CNNMethod/PostPredictor/PostPredictor.h"

namespace HobotXRoc {

class FaceQualityPostPredictor : public PostPredictor {
 public:
  virtual int32_t Init(std::shared_ptr<CNNMethodConfig> config);
  virtual void Do(CNNMethodRunData *run_data);
  virtual void UpdateParam(std::shared_ptr<CNNMethodConfig> config);

 private:
  void FaceQualityPostPro(const std::vector<std::vector<int8_t>> &mxnet_outs,
                          std::vector<BaseDataPtr> *output);
  float threshold_ = 0.0f;
};
}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_POSTPREDICTOR_FACEQUALITYPOSTPREDICTOR_H_
