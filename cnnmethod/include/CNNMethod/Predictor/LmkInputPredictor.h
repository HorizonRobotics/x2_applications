/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: LmkInputPredictor.h
 * @Brief: declaration of the LmkInputPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-16 14:52:31
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-16 16:22:44
 */

#ifndef INCLUDE_CNNMETHOD_PREDICTOR_LMKINPUTPREDICTOR_H_
#define INCLUDE_CNNMETHOD_PREDICTOR_LMKINPUTPREDICTOR_H_

#include "CNNMethod/Predictor/Predictor.h"
#include "CNNMethod/util/CNNMethodData.h"

namespace HobotXRoc {

class LmkInputPredictor : public Predictor {
 public:
  virtual void Do(CNNMethodRunData *run_data);
};
}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_PREDICTOR_LMKINPUTPREDICTOR_H_
