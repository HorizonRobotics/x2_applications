/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: LmkPosePostPredictor.h
 * @Brief: declaration of the LmkPosePostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 15:13:07
 */

#ifndef INCLUDE_CNNMETHOD_POSTPREDICTOR_LMKPOSEPOSTPREDICTOR_H_
#define INCLUDE_CNNMETHOD_POSTPREDICTOR_LMKPOSEPOSTPREDICTOR_H_

#include <vector>
#include "CNNMethod/PostPredictor/PostPredictor.h"

namespace HobotXRoc {

class LmkPosePostPredictor : public PostPredictor {
 public:
  virtual void Do(CNNMethodRunData *run_data);

 private:
  void HandleLmkPose(const std::vector<std::vector<int8_t>> &mxnet_outs,
                     const hobot::vision::BBox &box,
                     const std::vector<std::vector<uint32_t>> &nhwc,
                     std::vector<BaseDataPtr> *output);

  BaseDataPtr LmkPostPro(const std::vector<std::vector<int8_t>> &mxnet_outs,
                         const hobot::vision::BBox &box,
                         const std::vector<std::vector<uint32_t>> &nhwc);

  BaseDataPtr PosePostPro(const std::vector<int8_t> &mxnet_out);
};
}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_POSTPREDICTOR_LMKPOSEPOSTPREDICTOR_H_
