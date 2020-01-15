/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     first_num_best header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.16
 * @date      2019.05.22
 */

#ifndef SNAPSHOTMETHOD_STRATEGY_CROP_H_
#define SNAPSHOTMETHOD_STRATEGY_CROP_H_

#include <string>
#include <vector>
#include <list>
#include <map>
#include <memory>

#include "horizon/vision_type/vision_type.hpp"
#include "SnapShotMethod/SnapShotMethod.h"
#include "SnapShotMethod/image_utils/image_utils.hpp"
#include "SnapShotMethod/snapshot_data_type/snapshot_data_type.hpp"

namespace HobotXRoc {

class Crop : public SnapShot {
 public:
  int Init(std::shared_ptr<SnapShotParam> config) override;

  std::vector<BaseDataPtr> ProcessFrame(const std::vector<BaseDataPtr> &in,
                                        const InputParamPtr &param) override;

  void Finalize() override;

  int UpdateParameter(const std::string &content) override;

 private:
  std::vector<BaseDataPtr> CropAndPost(const std::vector<BaseDataPtr> &in);
};
}  // namespace HobotXRoc

#endif  // SNAPSHOTMETHOD_STRATEGY_CROP_H_
