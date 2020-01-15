/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: AntiSpfPostPredictor.h
 * @Brief: declaration of the AntiSpfPostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 15:13:07
 */

#ifndef INCLUDE_CNNMETHOD_POSTPREDICTOR_ANTISPFPOSTPREDICTOR_H_
#define INCLUDE_CNNMETHOD_POSTPREDICTOR_ANTISPFPOSTPREDICTOR_H_

#include <memory>
#include <vector>
#include "CNNMethod/PostPredictor/PostPredictor.h"

namespace HobotXRoc {

class AntiSpfPostPredictor : public PostPredictor {
 public:
  virtual int32_t Init(std::shared_ptr<CNNMethodConfig> config);
  virtual void Do(CNNMethodRunData *run_data);
  virtual void UpdateParam(std::shared_ptr<CNNMethodConfig> config);

 private:
  float anti_spf_threshold_ = 0.0f;
  BaseDataPtr
    FaceAntiSpfPostPro(const std::vector<std::vector<int8_t>> &mxnet_out,
                       int channel_size);
};
}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_POSTPREDICTOR_ANTISPFPOSTPREDICTOR_H_
