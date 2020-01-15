/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: ImgInputPredictor.h
 * @Brief: declaration of the ImgInputPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-16 14:52:31
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-16 16:22:44
 */

#ifndef INCLUDE_CNNMETHOD_PREDICTOR_IMGINPUTPREDICTOR_H_
#define INCLUDE_CNNMETHOD_PREDICTOR_IMGINPUTPREDICTOR_H_

#include <memory>
#include "CNNMethod/Predictor/Predictor.h"
#include "CNNMethod/util/CNNMethodData.h"

namespace HobotXRoc {

class ImgInputPredictor : public Predictor {
 public:
  virtual int32_t Init(std::shared_ptr<CNNMethodConfig> config);
  virtual void Do(CNNMethodRunData *run_data);
  virtual void UpdateParam(std::shared_ptr<CNNMethodConfig> config);

 private:
  float expand_scale_ = 0.0f;
  NormMethod norm_method_ = NormMethod::BPU_MODEL_NORM_BY_NOTHING;
  FilterMethod filter_method_ = FilterMethod::NO_FILTER;
  int rotate_degree_ = 0;
};
}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_PREDICTOR_IMGINPUTPREDICTOR_H_
