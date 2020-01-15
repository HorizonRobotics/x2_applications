/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: PostPredictorFactory.h
 * @Brief: declaration of the PostPredictorFactory
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 16:01:54
 */

#ifndef INCLUDE_CNNMETHOD_POSTPREDICTOR_POSTPREDICTORFACTORY_H_
#define INCLUDE_CNNMETHOD_POSTPREDICTOR_POSTPREDICTORFACTORY_H_

#include "CNNMethod/PostPredictor/AgeGenderPostPredictor.h"
#include "CNNMethod/PostPredictor/AntiSpfPostPredictor.h"
#include "CNNMethod/PostPredictor/FaceIdPostPredictor.h"
#include "CNNMethod/PostPredictor/FaceQualityPostPredictor.h"
#include "CNNMethod/PostPredictor/LmkPosePostPredictor.h"
#include "CNNMethod/PostPredictor/PostPredictor.h"

namespace HobotXRoc {

class PostPredictorFactory {
 public:
  static PostPredictor* GetPostPredictor(PostFun post_fun) {
    switch (post_fun) {
      case PostFun::FACE_ID: return new FaceIdPostPredictor();
      case PostFun::ANTI_SPF: return new AntiSpfPostPredictor();
      case PostFun::LMK_POSE: return new LmkPosePostPredictor();
      case PostFun::AGE_GENDER: return new AgeGenderPostPredictor();
      case PostFun::FACE_QUALITY: return new FaceQualityPostPredictor();
    }
    return nullptr;
  }
};

}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_POSTPREDICTOR_POSTPREDICTORFACTORY_H_
